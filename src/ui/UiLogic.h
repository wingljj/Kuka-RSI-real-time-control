#pragma once
#include <QString>
#include "core/Pose.h"
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

// 按钮启用状态。停止跟踪的可用范围刻意宽于 Tracking，理由见 .cpp。
ButtonStates buttonStates(const StatusSnapshot &s, bool listening);

// ── 数值格式化 ──

// 轴单位。0-2 为位置（mm），3-5 为姿态（deg）。
QString axisUnit(int axis);

// 位姿 / 误差 / 目标的显示格式：3 位小数 + 单位。
QString formatValue(double v, int axis);

// RKorr 增量的显示格式：4 位小数 + 单位。位数必须与 RsiCodec::buildSen
// 的线上量化一致——幅值小于该量化步长的增量会被格式化成 0，机器人实际
// 不动；用 3 位显示会让操作员看到一个「在动」的数字而机器静止。
// 非零值带正负号，零值不带（未跟踪时每帧都是零，满屏 "+0.0000" 很吵）。
QString formatRkorr(double v, int axis);

// 差值预览文案：列出目标与实际偏差超过 0.005 的轴。全部在容差内时
// 返回「无偏差」。
QString deltaPreview(const double target[6], const Pose &actual);

} // namespace uilogic
