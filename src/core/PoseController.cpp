#include "core/PoseController.h"

#include <algorithm>
#include <cmath>

namespace {

double clampAbs(double v, double limit)
{
    return std::clamp(v, -limit, limit);
}

} // namespace

void PoseController::configure(const AppConfig &cfg)
{
    m_cfg = cfg;
    const double cycleS = cfg.cycleMs / 1000.0;
    m_stepLimitPos = cfg.vmaxPosMmS * cycleS;
    m_stepLimitRot = cfg.vmaxRotDegS * cycleS;
}

void PoseController::resetToActual(const Pose &actual)
{
    m_target = actual;
    m_accum  = Pose{};
    m_state  = TrackState::Idle;
    m_faultReason.clear();
}

void PoseController::setTracking(bool on)
{
    if (on) {
        // Fault 必须先经 resetToActual 清除，不能直接重新使能
        if (m_state == TrackState::Idle)
            m_state = TrackState::Tracking;
    } else if (m_state == TrackState::Tracking) {
        m_state = TrackState::Idle;
    }
}

Pose PoseController::step(const Pose &actual)
{
    if (m_state != TrackState::Tracking)
        return Pose{};

    // 误差：位置直接相减，姿态取最短角路径
    const Pose err = poseSub(m_target, actual);

    // 第 1 层限值：单周期增量
    Pose d;
    d.x = clampAbs(m_cfg.kpPos * err.x, m_stepLimitPos);
    d.y = clampAbs(m_cfg.kpPos * err.y, m_stepLimitPos);
    d.z = clampAbs(m_cfg.kpPos * err.z, m_stepLimitPos);
    d.a = clampAbs(m_cfg.kpRot * err.a, m_stepLimitRot);
    d.b = clampAbs(m_cfg.kpRot * err.b, m_stepLimitRot);
    d.c = clampAbs(m_cfg.kpRot * err.c, m_stepLimitRot);

    // 第 2 层限值：累积修正量。越限则转 Fault 并停止累加。
    const Pose next{
        m_accum.x + d.x, m_accum.y + d.y, m_accum.z + d.z,
        m_accum.a + d.a, m_accum.b + d.b, m_accum.c + d.c,
    };

    const double posMax = std::max({std::fabs(next.x), std::fabs(next.y),
                                    std::fabs(next.z)});
    const double rotMax = std::max({std::fabs(next.a), std::fabs(next.b),
                                    std::fabs(next.c)});

    if (posMax > m_cfg.accumLimitPosMm) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral(
            "accumulated translation %1 mm exceeds limit %2 mm")
            .arg(posMax, 0, 'f', 3)
            .arg(m_cfg.accumLimitPosMm, 0, 'f', 3);
        return Pose{};
    }
    if (rotMax > m_cfg.accumLimitRotDeg) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral(
            "accumulated rotation %1 deg exceeds limit %2 deg")
            .arg(rotMax, 0, 'f', 3)
            .arg(m_cfg.accumLimitRotDeg, 0, 'f', 3);
        return Pose{};
    }

    m_accum = next;
    return d;
}
