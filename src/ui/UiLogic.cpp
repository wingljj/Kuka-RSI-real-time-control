#include "ui/UiLogic.h"

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

} // namespace uilogic
