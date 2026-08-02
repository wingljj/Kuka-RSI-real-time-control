#pragma once
#include "net/SharedState.h"

// 界面侧的纯判定逻辑。抽出来单独成文件的理由：这些判断原先散在
// MainWindow::onRefresh 里，而 onRefresh 每 20ms 跑一次、又只能靠肉眼验证，
// 于是「持续为真的告警每帧记一条」这类错误可以长期不被发现。纯函数可单测。

// 一组告警的布尔状态。既用作「当前是否告警」，也用作「上一帧的告警状态」。
struct AlarmEdge
{
    bool accumOverLimit = false;
    bool packetLoss     = false;
};

// 按钮与输入框的启用状态。由控制状态和监听状态共同决定。
struct ButtonStates
{
    bool resetFault   = false;
    bool enableTrack  = false;
    bool stopTrack    = false;
    bool startListen  = false;
    bool stopListen   = false;
    bool connEditable = false;   // IP / 端口是否可编辑
};

namespace uilogic {

// ── 告警 ──

// 当前帧的告警状态。
AlarmEdge currentAlarms(const StatusSnapshot &s);

// 上升沿：仅在「上一帧为假、本帧为真」时返回真。
// 事件日志只该在告警「发生」时记一条，而不是在告警「持续」的每一帧都记。
// 后者会在 4 秒内把 200 条上限刷满，把之前的真实事件全部挤掉。
AlarmEdge risingEdges(const AlarmEdge &prev, const StatusSnapshot &s);

// ── 控件启用状态 ──

// 按钮启用状态。
ButtonStates buttonStates(const StatusSnapshot &s, bool listening);

} // namespace uilogic
