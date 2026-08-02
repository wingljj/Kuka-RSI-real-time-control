#pragma once
#include <QFont>
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

// 丢包告警的迟滞状态。
//
// 为什么丢包需要迟滞而累计超限不需要：accumOverLimit 是「累积量 ≥ 限值」，
// 条件成立期间每帧都为真，是一段电平，边沿触发足以把它压成一条。
// missedCount 不是——RsiWorker 在每个正常帧把它归零（见 RsiWorker.cpp 的
// `case IpocEvent::Normal: m_missed = 0`），于是间歇丢包在快照里是一列
// 0/1/0/1 的脉冲，每个脉冲都是一次货真价实的上升沿。只做边沿触发挡不住它：
// krc_simulator --drop 5 实测 8.5 秒记了 142 条，200 条上限照样在半分钟内
// 刷满、把之前的真实事件挤掉——正是缺陷 F 要消除的后果。
// 所以要先把脉冲拉平成电平：一次丢包点亮「链路正在丢包」，之后必须连续
// 干净 clearFrames 帧才算恢复，整段只记一条。
struct LossHold
{
    bool active      = false;   // 是否处于「链路正在丢包」这一段
    int  quietFrames = 0;       // 连续无丢包的刷新帧数
};

namespace uilogic {

// ── 告警 ──

// 当前帧的告警状态。
AlarmEdge currentAlarms(const StatusSnapshot &s);

// 两个告警状态之间的上升沿。prev 必须是「上一帧的告警状态」，
// 不是上一帧 risingEdges 的返回值——后者在告警持续期间恒为假，
// 存错会让每一帧都成为上升沿，刷屏原样复现。
AlarmEdge edgesBetween(const AlarmEdge &prev, const AlarmEdge &now);

// 上升沿：仅在「上一帧为假、本帧为真」时返回真。
// 事件日志只该在告警「发生」时记一条，而不是在告警「持续」的每一帧都记。
// 后者会在 4 秒内把 200 条上限刷满，把之前的真实事件全部挤掉。
AlarmEdge risingEdges(const AlarmEdge &prev, const StatusSnapshot &s);

// 推进迟滞状态并返回带迟滞的告警电平：packetLoss 取拉平后的电平，
// 其余字段同 currentAlarms。每个刷新帧必须且只能调用一次（它会推进
// h 的内部计数）。clearFrames 是判定「恢复」所需的连续干净帧数，由调用方
// 按刷新周期折算，必须 ≥ 1。
AlarmEdge currentAlarmsHeld(LossHold &h, const StatusSnapshot &s, int clearFrames);

// ── 控件启用状态 ──

// 按钮启用状态。停止跟踪的可用范围刻意宽于 Tracking，理由见 .cpp。
ButtonStates buttonStates(const StatusSnapshot &s, bool listening);

// ── 数值格式化 ──

// RKorr 增量是否小到线上就是零。颜色判定与格式化必须用同一个门限，
// 否则会出现「显示 0.0000 却标成非零色」这类自相矛盾。
bool isRkorrZero(double v);

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

// ── 字体 ──

// 数值显示用的系统等宽字体。硬编码 "Consolas" 在没装该字体的机器上会
// 静默回退到比例字体，小数点从此不对齐——而对齐正是读数列存在的理由。
QFont monospaceFont();

} // namespace uilogic
