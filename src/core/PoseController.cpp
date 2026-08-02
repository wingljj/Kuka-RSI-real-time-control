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
    // 这两个是"一个完整周期的满预算"上限，不是每帧照发的常数：step() 按
    // 实测帧间隔在 [0, 1] 区间内按比例发放（见 step() 里的长注释）。
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

Pose PoseController::step(const Pose &actual, double sinceLastStepMs)
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

    // ── 本周期的步长预算：按实际流逝时间发放，上限一个配置周期 ──
    //
    // 为什么不能按配置周期发放：RSI 是增量接口，"这一帧该发多少"本来就是
    // "距上次发送过去了多久 × 允许速度"。用配置周期是个隐含假设——每帧恰好
    // 间隔一个周期。主机线程停顿（OS 调度、GC 等）时这个假设破裂：KRC 继续
    // 按 12ms 发包，几十个数据报积压在接收缓冲里，主机恢复后在几毫秒墙钟内
    // 连续排空；而每一帧都带着间隙前那个陈旧位姿（KRC 没收到修正、机器人
    // 没动），误差始终顶格。于是每帧都吐满额增量：实测一次 500ms 停顿排空
    // 41 帧 = 24.6mm，正是 KRC 侧 POSCORR 硬限（25mm）的量级。
    //
    // 为什么不能改用"实测周期 < 阈值就不发"：实测按真实 12ms 边界配速的
    // 118 帧里，有 10 帧（8.5%）的实测间隔低于 1ms——主机被抢占一次（实测
    // 最大 62.8ms），期间到达的几帧随后被连续排空。"排空积压"与"被短暂抢占
    // 后追赶"在时序上是同一件事，只是规模不同（3 帧 vs 41 帧），没有阈值能
    // 分开它们。按流逝时间发放则不需要分：0.07ms 后到达就只值 0.07ms 的预算，
    // 6ms 后到达就值半个，悬崖被换成连续函数，不存在误伤。
    //
    // 为什么必须封在一个配置周期：KRC 在一个 IPO 周期内施加这一帧的增量，
    // 增量大小就是那个周期里的速度。间隔 500ms 不代表这一帧可以走 25mm——
    // 那只会让机器人在 12ms 内跑出 40 倍速。封顶还带来一条便于论证的性质：
    // 任何一帧发出的增量都 ≤ 封顶前的值（比例 ∈[0,1]），所以本机制不可能
    // 引入新的越界路径，只可能少发。
    const double cycleMsAbs = std::fabs(m_cfg.cycleMs);   // >0：m_configInvalid 已挡住 ≤0
    // 非有限值不能进比例计算：NaN 会一路传播到限值，让"范数 > 限值"恒为假、
    // 第 1 层静默失效——与本文件里其它非有限守卫同一类问题。按 0 处理。
    const double elapsedMs = (std::isfinite(sinceLastStepMs) && sinceLastStepMs > 0.0)
                                 ? std::min(sinceLastStepMs, cycleMsAbs)
                                 : 0.0;
    // elapsedMs == cycleMsAbs 时比例恰为 1.0（同值相除），限值与封顶前逐位相同。
    const double budget       = elapsedMs / cycleMsAbs;
    const double stepLimitPos = m_stepLimitPos * budget;
    const double stepLimitRot = m_stepLimitRot * budget;

    // 第 1 层限值：位置按欧氏范数限幅（三轴同时到限时合成速度不超 √3×——
    // 逐轴 clamp 会让对角运动达到 √3×limit），姿态按旋转向量范数限幅。
    Pose d;
    d.x = m_cfg.kpPos * errPos.x;
    d.y = m_cfg.kpPos * errPos.y;
    d.z = m_cfg.kpPos * errPos.z;
    const double posNorm = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (stepLimitPos <= 0.0) {
        // vmax_pos=0（位置被阻止，与姿态路径一致）或本帧预算为 0（排空/无法
        // 测量间隔）：两种情况下都不许发位置修正。
        d.x = d.y = d.z = 0.0;
    } else if (posNorm > stepLimitPos) {
        const double s = stepLimitPos / posNorm;
        d.x *= s; d.y *= s; d.z *= s;
    }
    double dRot[3] = {m_cfg.kpRot * rotErr[0],
                      m_cfg.kpRot * rotErr[1],
                      m_cfg.kpRot * rotErr[2]};
    const double rotNorm = std::sqrt(dRot[0]*dRot[0] + dRot[1]*dRot[1] + dRot[2]*dRot[2]);
    // m_stepLimitRot 是 deg/周期，dRot 范数是 rad——阈值须换算成 rad 再比较，
    // 否则等效步长限幅放大约 57.3×（安全相关，见 attitude_stepLimitRespectsDegPerCycle）。
    // 姿态与位置必须共用同一个 budget：只收紧一条路，排空时机器人照样能靠
    // 姿态修正走满 41 帧的角度预算。
    const double rotLimitRad = stepLimitRot * kDegToRad;
    if (rotLimitRad <= 0.0) {
        dRot[0] = dRot[1] = dRot[2] = 0.0;   // vmax_rot=0 或本帧零预算：旋转被阻止
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
