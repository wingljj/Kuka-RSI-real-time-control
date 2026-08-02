#include "ui/UiLogic.h"

#include <QStringList>
#include <cmath>

namespace uilogic {

AlarmEdge currentAlarms(const StatusSnapshot &s)
{
    AlarmEdge a;
    // 累计超限只在 Tracking 下有意义：不跟踪时控制器不发增量，
    // 累计值是上一段跟踪留下的历史，不构成「现在出事了」。
    a.accumOverLimit = s.accumOverLimit && s.state == ControlState::Tracking;
    // 丢包计数在断开时是上一次会话的残值，同样不构成当前告警。
    a.packetLoss     = s.missedCount > 0 && s.connected;
    return a;
}

AlarmEdge risingEdges(const AlarmEdge &prev, const StatusSnapshot &s)
{
    const AlarmEdge now = currentAlarms(s);
    AlarmEdge e;
    e.accumOverLimit = now.accumOverLimit && !prev.accumOverLimit;
    e.packetLoss     = now.packetLoss     && !prev.packetLoss;
    return e;
}

ButtonStates buttonStates(const StatusSnapshot &s, bool listening)
{
    ButtonStates b;

    // 地址与端口只在未绑定时可编辑：运行中改它们不会生效，
    // 只会让界面显示的地址与实际绑定的地址不符——那比不给改更糟。
    b.connEditable = !listening;
    b.startListen  = !listening;
    b.stopListen   = listening;

    // Fault 是锁存的：PoseController::setTracking(true) 在 Fault 下不转
    // Tracking，必须先经复位清除。所以 Fault 下「使能跟踪」必须禁用，
    // 否则点了没反应，操作员会以为是程序卡死。
    b.resetFault  = (s.state == ControlState::Fault);
    b.enableTrack = (s.state == ControlState::Ready);

    // 停止永远应该比启动更容易触发。停止跟踪的可用范围因此比 Tracking 更宽：
    // publishSnapshot 的优先级是 Fault > StaleFrame > Syncing > Tracking > Ready，
    // 于是跟踪中一旦反馈跳变，ControlState 会变成 StaleFrame 或 Syncing，
    // 而 PoseController 内部仍是 TrackState::Tracking、仍在发增量
    // （forceFault 只在连续 stale 达到 staleFrameLimit 时才触发）。
    // 那正是最需要能停的时刻：反馈已经异常而机器人还在动。若这里只认
    // Tracking，按钮恰好在此时变灰，操作员只能干看着。
    // 注意反向不成立：enableTrack 必须严格限于 Ready，放宽启用条件
    // 等于允许在状态未知时开始发增量。
    b.stopTrack   = (s.state == ControlState::Tracking
                     || s.state == ControlState::StaleFrame
                     || s.state == ControlState::Syncing);

    return b;
}

// ── 数值格式化 ──

namespace {

const char *kAxisName[6] = {"X", "Y", "Z", "A", "B", "C"};

} // namespace

QString axisUnit(int axis)
{
    return (axis < 3) ? QStringLiteral(" mm") : QStringLiteral(" deg");
}

QString formatValue(double v, int axis)
{
    return QStringLiteral("%1%2").arg(v, 0, 'f', 3).arg(axisUnit(axis));
}

QString formatRkorr(double v, int axis)
{
    // 与 buildSen 的量化步长对齐：小于半个步长的量线上就是 0，
    // 显示成 0 是如实反映，不是精度损失。
    constexpr double kWireQuantum = 5e-5;
    if (std::fabs(v) < kWireQuantum)
        return QStringLiteral("0.0000%1").arg(axisUnit(axis));
    return QStringLiteral("%1%2%3")
        .arg(v > 0 ? "+" : "")
        .arg(v, 0, 'f', 4)
        .arg(axisUnit(axis));
}

QString deltaPreview(const double target[6], const Pose &actual)
{
    const double act[6] = {actual.x, actual.y, actual.z,
                           actual.a, actual.b, actual.c};
    // 先收集，再决定文案。原实现把前缀直接写进结果串，
    // 于是 isEmpty() 永不为真、「无偏差」分支永不可达。
    QStringList parts;
    for (int i = 0; i < 6; ++i) {
        const double d = target[i] - act[i];
        if (std::fabs(d) > 0.005)
            parts << QStringLiteral("%1 %2%3")
                         .arg(kAxisName[i])
                         .arg(d, 0, 'f', (i < 3) ? 2 : 3)
                         .arg(axisUnit(i));
    }
    if (parts.isEmpty())
        return QStringLiteral("目标与当前位姿无偏差");
    return QStringLiteral("目标 − 当前：") + parts.join(QStringLiteral("　"));
}

} // namespace uilogic
