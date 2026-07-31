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

    // 误差归零：目标置为实际、状态回 Idle、清除故障原因。
    // 【刻意保留累积修正量】——RELATIVE 模式下 KRC 侧已施加的修正不会因
    // 主机侧归零而消失。若在此清零，反复「停止跟踪→归零→使能」就能不断
    // 领取新的累积预算，把总修正一路推过 POSCORR 的 ~50mm 硬限，而界面上
    // 第 2 层始终显示一个很小的累积值。用于会话内的「停止跟踪」「归零」。
    void resetToActual(const Pose &actual);

    // RSI 会话开始：在 resetToActual 之上额外清零累积修正量。
    // 仅当 KRL 程序重新启动、KRC 侧修正确实已回零时才可调用。
    void beginSession(const Pose &actual);

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

    // 配置非法（如 cycleMs <= 0）。粘滞：resetToActual/beginSession 都不清除它，
    // 因为生产调用顺序恰是 configure → beginSession(首帧)，若用 Fault 状态承载
    // 就会在第一次 step() 之前被擦掉，保护形同不存在。只有一次有效的
    // configure() 能解除。
    bool m_configInvalid = false;
};
