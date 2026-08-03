#pragma once
#include <QString>
#include "core/AppConfig.h"
#include "core/Pose.h"
#include "core/PoseOps.h"
#include "core/TargetTrajectory.h"

enum class TrackState { Idle, Tracking, Fault };

// 纯计算，无 IO、无 Qt 信号槽。可在通信线程内直接调用。
class PoseController
{
public:
    void configure(const AppConfig &cfg);

    // 目标变化时从"当前实际位姿"启动固定时长轨迹（位置五次多项式 + 姿态
    // Slerp），到点即达；目标未变化则轨迹继续/保持，不重启。
    void setTarget(const Pose &t)
    {
        if (t.x != m_target.x || t.y != m_target.y || t.z != m_target.z
            || t.a != m_target.a || t.b != m_target.b || t.c != m_target.c) {
            m_traj.setGoal(m_lastActual, t, m_cfg.targetTrajectoryMs);
            m_target = t;
        }
    }
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
    //
    // sinceLastStepMs = 距上一次调用 step() 真正过去的墙钟毫秒（不是配置周期）。
    // 它是本控制器唯一的时间源：步长限值按它发放，目标轨迹也按同一个（夹到
    // 一个配置周期以内的）值推进。两者共用一个时间基不是巧合而是要求——分开
    // 用会让积压排空只收紧幅值却照旧推进轨迹，恢复后第一帧直接顶格（详见
    // step() 实现里的长注释）。
    // 为什么按墙钟而不是配置周期：RSI 是增量接口，这一帧该发多少取决于"距上次
    // 发送过去了多久 × 允许速度"；而且只有这样 vmax 才是真正的速度上限——按
    // 配置周期发放时它只是"每帧配额"，cycleMs 配成 12ms 而真实 IPO 节拍是 4ms
    // 就等于 3 倍超速（见 step()）。
    // 参数刻意做成必填而不给默认值——给一个默认值就等于给"满预算"，那正是
    // 积压排空时最危险的取值，绝不能让新调用方靠遗忘拿到它。
    // 无法测量时（如看门狗刚清掉时间基准）必须传 0：宁可这一帧不动、轨迹也
    // 不推进，也不能把一段无从核实的静默当成满预算。负值/非有限值同样按 0 处理。
    Pose step(const Pose &actual, double sinceLastStepMs);

    TrackState state() const { return m_state; }
    // 现在返回的是相对会话锚点的位移（见 displacement()），而非命令增量之和；
    // 旧含义请用 commandedSum()。
    Pose accumulated() const { return m_displacement; }

    // 第 2 层限值所用的量：当前实际位姿相对会话锚点 RIst₀ 的位移。
    // 这与 POSCORR 的 ~50mm 硬限共享原点（都相对 RSI 启动位姿测量），
    // 所以 layer 2 与 layer 4 终于可比较。
    Pose displacement() const { return m_displacement; }

    // 主机累计发出的命令增量之和。保留作交叉校验：它与 displacement()
    // 的背离说明"主机以为发出去了但机器人没走到"，是丢包的直接证据。
    Pose commandedSum() const { return m_accum; }

    QString faultReason() const { return m_faultReason; }

    // 外部（网络层）注入的锁存故障：写失败、KRC Delay 增长等。与内部判定的
    // Fault 一样，必须经 resetToActual 才能清除，绝不能被 setTracking(true) 绕过。
    void forceFault(const QString &reason);

private:
    AppConfig  m_cfg = AppConfig::defaults();
    Pose       m_target;
    Pose       m_accum;
    TrackState m_state = TrackState::Idle;
    QString    m_faultReason;

    // 目标轨迹（仅 Tracking 生效）：setTarget 目标变化时从 m_lastActual 启动，
    // step 每周期采样作为误差源并推进。时长 ≤0 = 立即完成 = 直通。
    TargetTrajectory m_traj;
    Pose             m_lastActual;   // 最近一帧实际位姿（轨迹起点）

    Pose m_anchor;                  // 会话首帧锁存的 RIst₀
    Pose m_displacement;            // 当前实际位姿相对 m_anchor 的位移
    bool m_haveAnchor = false;

    double m_stepLimitPos = 0.0;   // mm  / 周期（满预算上限；实发按实测间隔按比例发放）
    double m_stepLimitRot = 0.0;   // deg / 周期（同上）

    // 配置非法（如 cycleMs <= 0）。粘滞：resetToActual/beginSession 都不清除它，
    // 因为生产调用顺序恰是 configure → beginSession(首帧)，若用 Fault 状态承载
    // 就会在第一次 step() 之前被擦掉，保护形同不存在。只有一次有效的
    // configure() 能解除。
    bool m_configInvalid = false;
};
