#include "core/TargetTrajectory.h"

#include <cmath>

namespace {
// 五次多项式：s(u)=10u³-15u⁴+6u⁵，s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0。
double quintic(double u)
{
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}
} // namespace

void TargetTrajectory::setGoal(const Pose &start, const Pose &goal, double durationMs)
{
    m_start = start;
    m_goal  = goal;
    m_q0    = poseops::quatFromABC(start.a, start.b, start.c);
    m_q1    = poseops::quatFromABC(goal.a, goal.b, goal.c);
    m_durS  = durationMs > 0.0 ? durationMs / 1000.0 : 0.0;
    m_tS    = 0.0;
    m_u     = m_durS > 0.0 ? 0.0 : 1.0;
}

void TargetTrajectory::advance(double dtS)
{
    if (m_u >= 1.0 || dtS <= 0.0)
        return;
    m_tS += dtS;
    m_u = std::min(1.0, m_tS / m_durS);
}

Pose TargetTrajectory::sample() const
{
    if (m_u >= 1.0)
        return m_goal;
    const double s = quintic(m_u);
    Pose p;
    p.x = m_start.x + s * (m_goal.x - m_start.x);
    p.y = m_start.y + s * (m_goal.y - m_start.y);
    p.z = m_start.z + s * (m_goal.z - m_start.z);
    const poseops::Quat q = poseops::quatSlerp(m_q0, m_q1, s);
    poseops::abcFromQuat(q, &p.a, &p.b, &p.c);
    return p;
}
