#include "core/PoseController.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

bool poseIsFinite(const Pose &p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)
        && std::isfinite(p.a) && std::isfinite(p.b) && std::isfinite(p.c);
}

} // namespace

void PoseController::configure(const AppConfig &cfg)
{
    m_cfg = cfg;
    // 不信任配置：AppConfig 不做范围校验，负的 vmax 会让 clampAbs 收到
    // lo > hi——形式上是 UB，libstdc++ 上会返回 hi，于是机器人朝远离目标
    // 的方向走。取绝对值挡住这一类。
    const double cycleS = std::fabs(cfg.cycleMs) / 1000.0;
    m_stepLimitPos = std::fabs(cfg.vmaxPosMmS) * cycleS;
    m_stepLimitRot = std::fabs(cfg.vmaxRotDegS) * cycleS;

    // 轨迹时长在 setTarget 时经 m_cfg.targetTrajectoryMs 使用（≤0 = 立即完成）。

    // 非正周期会让步长上限为 0，表现为静默不动而 state() 仍报 Tracking。
    // 用粘滞标志承载，而不是 Fault 状态——Fault 会被随后的 beginSession 清除。
    m_configInvalid = !(cfg.cycleMs > 0.0);
    if (m_configInvalid) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral("invalid cycleMs %1; must be > 0")
                            .arg(cfg.cycleMs);
    }
}

void PoseController::resetToActual(const Pose &actual)
{
    m_target = actual;
    m_state  = TrackState::Idle;
    m_faultReason.clear();
    // 归零/会话开始：轨迹立即完成（目标=实际），避免假误差；
    // m_lastActual 同步为实际，保证下一次 setTarget 从当前实际出发。
    m_traj.setGoal(actual, actual, 0);
    m_lastActual = actual;
    // m_accum 刻意保留，理由见头文件注释
    // m_anchor / m_displacement / m_haveAnchor 同样刻意不动：会话内的归零
    // 不移动原点，否则第 2 层的预算又能被反复领取。只有 beginSession 才换锚点。
}

void PoseController::beginSession(const Pose &actual)
{
    resetToActual(actual);
    m_accum        = Pose{};
    m_anchor       = actual;      // 锁存 RIst₀：第 2 层从此以它为原点
    m_displacement = Pose{};
    m_haveAnchor   = true;
}

void PoseController::setTracking(bool on)
{
    if (on) {
        // Fault 必须先经 resetToActual 清除，不能直接重新使能
        if (m_state == TrackState::Idle && !m_configInvalid)
            m_state = TrackState::Tracking;
    } else if (m_state == TrackState::Tracking) {
        m_state = TrackState::Idle;
    }
}

void PoseController::forceFault(const QString &reason)
{
    m_state = TrackState::Fault;
    m_faultReason = reason;
}

Pose PoseController::step(const Pose &actual)
{
    // 粘滞的配置故障：每个周期都重新宣告，因为 resetToActual/beginSession
    // 会清掉 Fault 状态，但不该让一个非法配置就此隐身。
    if (m_configInvalid) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral("invalid cycleMs %1; must be > 0")
                            .arg(m_cfg.cycleMs);
        return Pose{};
    }

    // 记录本周期实际，供下一次 setTarget 作为轨迹起点（从实际出发）。
    // 必须在 Tracking 守卫之前：机器人可在 Ready（未使能跟踪）下手动移动，
    // 若只在 Tracking 路径更新，随后开始的轨迹会从陈旧位姿起步。
    m_lastActual = actual;

    if (m_state != TrackState::Tracking)
        return Pose{};

    // 非有限值守卫：clampAbs 基于比较，会传播 NaN 而非限界它。一旦累积量
    // 变成 NaN，此后 NaN > limit 恒为 false，第 2 层限值将永久失效，而
    // state() 仍报 Tracking——界面会显示一个"健康"的控制器。
    if (!poseIsFinite(actual) || !poseIsFinite(m_target)) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral(
            "non-finite pose component in actual or target");
        return Pose{};
    }

    // 目标来源：轨迹未完成时用轨迹采样（五次多项式 + Slerp，起点速度 0），
    // 完成即最终目标（固定时长语义：到点即达，而非指数逼近永远追不上）。
    // 先采样后推进：目标变化后的首周期采样 = 起点（= 实际），增量 0，
    // 运动平滑起步；推进用配置周期换算的秒数。
    Pose errSrc = m_traj.isFinished() ? m_target : m_traj.sample();
    if (!m_traj.isFinished())
        m_traj.advance(m_cfg.cycleMs / 1000.0);

    // 位置误差：逐轴差（无奇异问题）。姿态误差：SO(3) 最短旋转（旋转向量，
    // 世界坐标 rad）——奇异/边界目标下不再逐轴 wrap 跳变。
    const Pose errPos = poseSub(errSrc, actual);   // 仅用 x/y/z
    const poseops::Quat qA = poseops::quatFromABC(actual.a, actual.b, actual.c);
    const poseops::Quat qT = poseops::quatFromABC(errSrc.a, errSrc.b, errSrc.c);
    const poseops::Quat qE = poseops::quatError(qT, qA);
    double rotErr[3];
    poseops::rotVecFromQuat(qE, rotErr);

    // 第 1 层限值：位置按欧氏范数限幅（三轴同时到限时合成速度不超 √3×——
    // 逐轴 clamp 会让对角运动达到 √3×limit），姿态按旋转向量范数限幅。
    Pose d;
    d.x = m_cfg.kpPos * errPos.x;
    d.y = m_cfg.kpPos * errPos.y;
    d.z = m_cfg.kpPos * errPos.z;
    const double posNorm = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (m_stepLimitPos <= 0.0) {
        d.x = d.y = d.z = 0.0;   // vmax_pos=0：位置被阻止（与姿态路径一致）
    } else if (posNorm > m_stepLimitPos) {
        const double s = m_stepLimitPos / posNorm;
        d.x *= s; d.y *= s; d.z *= s;
    }
    double dRot[3] = {m_cfg.kpRot * rotErr[0],
                      m_cfg.kpRot * rotErr[1],
                      m_cfg.kpRot * rotErr[2]};
    const double rotNorm = std::sqrt(dRot[0]*dRot[0] + dRot[1]*dRot[1] + dRot[2]*dRot[2]);
    // m_stepLimitRot 是 deg/周期，dRot 范数是 rad——阈值须换算成 rad 再比较，
    // 否则等效步长限幅放大约 57.3×（安全相关，见 attitude_stepLimitRespectsDegPerCycle）。
    const double rotLimitRad = m_stepLimitRot * kDegToRad;
    if (rotLimitRad <= 0.0) {
        dRot[0] = dRot[1] = dRot[2] = 0.0;   // vmax_rot=0：旋转被阻止（与位置路径一致）
    } else if (rotNorm > rotLimitRad) {
        const double s = rotLimitRad / rotNorm;
        dRot[0] *= s; dRot[1] *= s; dRot[2] *= s;
    }

    // RKorr 姿态输出：Δ欧拉 = E⁻¹(actual A,B,C)·dRot。奇异时退化为一阶近似 + 限幅。
    double invE[3][3];
    if (poseops::invEulerRate(actual.a, actual.b, actual.c, invE)) {
        d.a = (invE[0][0]*dRot[0] + invE[0][1]*dRot[1] + invE[0][2]*dRot[2]) * kRadToDeg;
        d.b = (invE[1][0]*dRot[0] + invE[1][1]*dRot[1] + invE[1][2]*dRot[2]) * kRadToDeg;
        d.c = (invE[2][0]*dRot[0] + invE[2][1]*dRot[1] + invE[2][2]*dRot[2]) * kRadToDeg;
    } else {
        // B≈±84° 以外：E⁻¹ 含 1/cosB 无界。用阻尼 E⁻¹（1/cosB → 1/max(|cosB|,0.1)，
        // 保号）替代一阶近似——一阶近似忽略欧拉耦合、方向错误，会让 B 卡在阈值
        // 边界无法推进（实测卡在 -84.3°）。阻尼 E⁻¹ 方向始终正确、增益 ≤10×，
        // 已被范数限幅兜底。
        const double aR = actual.a * kDegToRad, bR = actual.b * kDegToRad;
        const double sa = std::sin(aR), ca = std::cos(aR), sb = std::sin(bR);
        const double cbS = std::cos(bR);
        const double cbD = std::copysign(std::max(std::fabs(cbS), 0.1), cbS);
        const double tb = sb / cbD;
        d.a = (tb * ca * dRot[0] + tb * sa * dRot[1] + dRot[2]) * kRadToDeg;
        d.b = (-sa * dRot[0] + ca * dRot[1]) * kRadToDeg;
        d.c = (ca / cbD * dRot[0] + sa / cbD * dRot[1]) * kRadToDeg;
    }

    // 与 buildSen 的 4 位小数量化对齐：幅值小于线上量化步长的增量会被格式化
    // 成 0.0000，机器人实际不动。若仍把它计入累积，收敛静止后账本会持续
    // 漂移（实测约 9mm/小时），虚假耗尽第 2 层预算并让一次健康的长时间运行
    // 无故故障。账本必须只记录真正发得出去的量。
    constexpr double kWireQuantum = 1e-4;
    if (std::fabs(d.x) < kWireQuantum) d.x = 0.0;
    if (std::fabs(d.y) < kWireQuantum) d.y = 0.0;
    if (std::fabs(d.z) < kWireQuantum) d.z = 0.0;
    if (std::fabs(d.a) < kWireQuantum) d.a = 0.0;
    if (std::fabs(d.b) < kWireQuantum) d.b = 0.0;
    if (std::fabs(d.c) < kWireQuantum) d.c = 0.0;

    // 第 2 层限值：累积修正量。越限则转 Fault 并停止累加。
    const Pose next{
        m_accum.x + d.x, m_accum.y + d.y, m_accum.z + d.z,
        m_accum.a + d.a, m_accum.b + d.b, m_accum.c + d.c,
    };

    // 兜住所有非有限来源：上面的守卫只覆盖 actual/target，而增益本身
    // （kpPos/kpRot）若为非有限值同样会污染 next，进而让 next > limit 恒假、
    // 第 2 层永久失效。在此一次性挡住当前与将来的所有路径。
    if (!poseIsFinite(next)) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral(
            "non-finite accumulated correction (check Kp and limits)");
        return Pose{};
    }

    // 第 2 层限值改用"控制器回传的实际位姿相对会话锚点的位移"，而不是
    // "主机发出的命令增量之和"。两者原点不同：命令和的原点会随主机对
    // 会话边界的判断而漂移（一次比 KRC 的 Timeout 更短的通信间隙就足以
    // 让主机误判为新会话并凭空发放新预算），而 POSCORR 的限值始终相对
    // RSI 启动位姿测量。用 RIst 锚点后二者共享原点，且免疫丢包与误判。
    // 前提：KRL 做原地 BCO 后静止，此后 TCP 的全部位移都来自 RSI 修正，
    // 所以 ‖RIst − RIst₀‖ 就是已施加的修正量。
    m_displacement = m_haveAnchor ? poseSub(actual, m_anchor) : Pose{};

    if (!poseIsFinite(m_displacement)) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral(
            "non-finite displacement from session anchor");
        return Pose{};
    }

    // 【第 2 层已按用户决定移除（2026-08-01）】：不再检查累积位移/命令和是否越限。
    // m_displacement / m_accum 仍计算并暴露（accumulated()/commandedSum()），
    // 仅供 UI「累积修正」显示。KRC 侧层 4/5（POSCORR ±25 / POSCORRMON 45）是唯一兜底。
    m_accum = next;
    return d;
}
