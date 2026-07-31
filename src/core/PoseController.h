#pragma once
#include <QString>
#include "core/AppConfig.h"
#include "core/Pose.h"

enum class TrackState { Idle, Tracking, Fault };

// 纯计算，无 IO、无 Qt 信号槽。可在通信线程内直接调用。
class PoseController
{
public:
    void configure(const AppConfig &cfg);

    void setTarget(const Pose &t) { m_target = t; }
    Pose target() const { return m_target; }

    // 目标置为实际、累积清零、状态回 Idle。
    // 用于「收到首帧」与「停止跟踪」两处，保证误差瞬时归零。
    void resetToActual(const Pose &actual);

    void setTracking(bool on);

    // 计算本周期增量。非 Tracking 状态一律返回零增量，
    // 但调用方仍必须把结果回包给 KRC。
    Pose step(const Pose &actual);

    TrackState state() const { return m_state; }
    Pose accumulated() const { return m_accum; }
    QString faultReason() const { return m_faultReason; }

private:
    AppConfig  m_cfg = AppConfig::defaults();
    Pose       m_target;
    Pose       m_accum;
    TrackState m_state = TrackState::Idle;
    QString    m_faultReason;

    double m_stepLimitPos = 0.0;   // mm  / 周期
    double m_stepLimitRot = 0.0;   // deg / 周期
};
