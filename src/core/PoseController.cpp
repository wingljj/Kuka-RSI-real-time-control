#include "core/PoseController.h"

#include <algorithm>
#include <cmath>

namespace {

double clampAbs(double v, double limit)
{
    return std::clamp(v, -limit, limit);
}

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

    // 平滑系数：仅当时间常数 > 0 才启用低通，否则直通（保持旧行为）。
    const double tauS = cfg.targetSmoothingMs > 0.0
                            ? cfg.targetSmoothingMs / 1000.0
                            : 0.0;
    m_alpha = (tauS <= 0.0 || cycleS <= 0.0) ? 1.0 : cycleS / (cycleS + tauS);

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
    m_smoothTarget = actual;   // 归零/会话开始同步平滑目标，避免假误差
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

    // 误差源：默认原始目标；平滑启用时用低通后的平滑目标（每周期指数逼近）。
    // m_smoothTarget 只在 resetToActual/beginSession 同步到 actual，此处每周期
    // 累加，绝不直接赋值——否则会丢历史。τ=0 时 m_alpha=1，一步到位等价无平滑。
    Pose errSrc = m_target;
    if (m_alpha < 1.0) {
        m_smoothTarget.x += m_alpha * (m_target.x - m_smoothTarget.x);
        m_smoothTarget.y += m_alpha * (m_target.y - m_smoothTarget.y);
        m_smoothTarget.z += m_alpha * (m_target.z - m_smoothTarget.z);
        // 姿态取最短角路径：目标跳变 -179→+179 应走经 180 的 2°，而非经 0 的 358°
        m_smoothTarget.a += m_alpha * wrap180(m_target.a - m_smoothTarget.a);
        m_smoothTarget.b += m_alpha * wrap180(m_target.b - m_smoothTarget.b);
        m_smoothTarget.c += m_alpha * wrap180(m_target.c - m_smoothTarget.c);
        errSrc = m_smoothTarget;
    }
    // 误差：位置直接相减，姿态取最短角路径
    const Pose err = poseSub(errSrc, actual);

    // 第 1 层限值：单周期增量
    Pose d;
    d.x = clampAbs(m_cfg.kpPos * err.x, m_stepLimitPos);
    d.y = clampAbs(m_cfg.kpPos * err.y, m_stepLimitPos);
    d.z = clampAbs(m_cfg.kpPos * err.z, m_stepLimitPos);
    d.a = clampAbs(m_cfg.kpRot * err.a, m_stepLimitRot);
    d.b = clampAbs(m_cfg.kpRot * err.b, m_stepLimitRot);
    d.c = clampAbs(m_cfg.kpRot * err.c, m_stepLimitRot);

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

    // 位置用欧几里得范数：逐轴判限会让三轴同时到限时的合成位移达到
    // √3 × limit（30mm → 51.96mm），越过 POSCORR 的 ~50mm 硬限，使第 2 层
    // 形同虚设、第一个停机的反而是第 4 层 RSI 错误停机。
    const double posNorm = std::hypot(m_displacement.x,
                                      m_displacement.y,
                                      m_displacement.z);
    // 姿态监控取两源保守值：
    //  (1) RIst 锚点位移逐轴最大 —— RIst 姿态角本身可能折返（±180°），主机
    //      无法得知真实累计圈数，仅靠它会在多圈旋转时漏掉；
    //  (2) 主机未折返累计命令增量（commandedSum）逐轴最大 —— 不折返，反映
    //      "主机以为发出去了多少修正"。
    // 取二者较大。高估是安全方向：宁可因丢包导致的高估提前 Fault，也不漏报。
    const double rotDisp = std::max({std::fabs(m_displacement.a),
                                     std::fabs(m_displacement.b),
                                     std::fabs(m_displacement.c)});
    const double rotCmd  = std::max({std::fabs(m_accum.a),
                                     std::fabs(m_accum.b),
                                     std::fabs(m_accum.c)});
    const double rotMax  = std::max(rotDisp, rotCmd);

    // 限值同样不信任配置：负的累积限值会让 posNorm(>=0) > limit 恒真，
    // 零误差时也立刻故障。与 vmax/cycleMs 的处理保持一致。
    const double accumLimPos = std::fabs(m_cfg.accumLimitPosMm);
    const double accumLimRot = std::fabs(m_cfg.accumLimitRotDeg);

    if (posNorm > accumLimPos) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral(
            "displacement from session anchor %1 mm exceeds limit %2 mm")
            .arg(posNorm, 0, 'f', 3)
            .arg(accumLimPos, 0, 'f', 3);
        return Pose{};
    }
    if (rotMax > accumLimRot) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral(
            "max per-axis accumulated rotation %1 deg exceeds limit %2 deg")
            .arg(rotMax, 0, 'f', 3)
            .arg(accumLimRot, 0, 'f', 3);
        return Pose{};
    }

    m_accum = next;
    return d;
}
