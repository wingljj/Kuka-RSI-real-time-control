#pragma once
#include "core/Pose.h"
#include "core/PoseOps.h"

// 目标轨迹：位置五次多项式 + 姿态 Slerp（速度/加速度连续），增量式推进。
// 纯 O(1) 算术、POD 状态，可在实时路径调用（无分配/阻塞/IO）。
class TargetTrajectory
{
public:
    void setGoal(const Pose &start, const Pose &goal, double durationMs);
    void advance(double dtS);          // 每周期推进轨迹时间
    Pose sample() const;               // 当前平滑目标（完成时 = goal）
    bool isFinished() const { return m_u >= 1.0; }

private:
    Pose m_start, m_goal;
    poseops::Quat m_q0, m_q1;
    double m_tS    = 0.0;
    double m_durS  = 0.0;
    double m_u     = 1.0;   // 归一化进度 [0,1]，未启动即完成
};
