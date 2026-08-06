#pragma once
#include <QColor>
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
    // Fault 锁存(2026-08-06 P1-5):faultReason 是排查的第一手信息,而 Fault
    // 常伴随断链(看门狗)或被随手归零抹掉——必须在发生的当帧进事件日志。
    // 刻意不要求 s.connected:看门狗发布的 Fault 快照就是断链的。
    bool faultLatched   = false;
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
    // 「读取当前值」：把 actual 抄进目标输入框是否有意义。
    // 这一项原先没有守卫，未连接时点它会把全零的 actual 写进目标框，
    // 操作员再点「应用目标」就把目标设成了 BASE 原点——机器人朝原点走。
    bool readActual   = false;
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

// ── 误差列：为什么它需要一套自己的格式化与说明 ──
//
// poseops::errorPoseDeg 装进 Pose::a/b/c 的不是 A/B/C 三个欧拉角之差，而是
// 「把当前姿态转到目标姿态」那一次最短旋转（SO(3)）分解到世界坐标 X/Y/Z 轴
// 上的三个分量。两者只在小角度下才近似相等，大角度、尤其万向节死锁附近
// 差得极远：现场报过一例，目标 ABC=(70, 90, 70)、当前全零，误差列打出
// 0 / 90 / 0，操作员按行首的 A/B/C 读成「B 差 90 度，A 和 C 一点没差」，
// 认定是算法 bug。数字其实完全正确——B=90° 时 A 与 C 绕同一条轴转，
// (70, 90, 70) 与 (0, 90, 0) 是同一个姿态，最短路径就是绕世界 Y 轴转 90°。
//
// 骗人的是标签：误差列的后三行与位姿列、目标列共用行首那个 A/B/C。
// 所以这里给误差列单独一套轴标签（Rx/Ry/Rz）和说明文案，让「这一列的
// 后三行与行首的 A/B/C 不是同一种量」在界面上无须悬停就能看见。
//
// 刻意不改成逐轴欧拉角差：旋转向量正是 PoseController::step 实际会走的
// 路径（它用四元数误差 → rotVecFromQuat 算增量），换成欧拉角差等于让界面
// 显示一个机器不会执行的量。

// 误差列的轴标签。位置三行为 X/Y/Z（与行首相同，就是简单相减）；
// 姿态三行为 Rx/Ry/Rz——世界坐标轴，与行首的 A/B/C 不是同一种量。
QString errorAxisLabel(int axis);

// 误差列的显示格式。位置三行同 formatValue；姿态三行前置 Rx/Ry/Rz。
// 前缀写进单元格文本而不是只挂 tooltip：操作员是扫一眼读数，不会去悬停，
// 而这一列被读错的代价是把正确结果当成 bug 报上来（已经发生过一次）。
QString formatError(double v, int axis);

// 误差列表头的 tooltip：解释这一列的姿态三行是什么、与欧拉角差差在哪，
// 并给出现场那组 (70, 90, 70) → 0/90/0 的实例。
QString errorColumnTooltip();

// 误差列单元格的 tooltip。姿态三行给出该行分量的确切含义；
// 位置三行返回空串——它们就是简单相减，加说明只会稀释真正需要读的那三条。
QString errorCellTooltip(int axis);

// 差值预览文案：列出目标与实际偏差超过 0.005 的轴。全部在容差内时
// 返回「无偏差」。
QString deltaPreview(const double target[6], const Pose &actual);

// ── 字体 ──

// 数值显示用的系统等宽字体。硬编码 "Consolas" 在没装该字体的机器上会
// 静默回退到比例字体，小数点从此不对齐——而对齐正是读数列存在的理由。
QFont monospaceFont();

// ── 语义色 ──

// 状态语义等级。与 QSS 时代的 ok/warn/fault/idle 一一对应，
// 但现在只用来选一个 QColor，不再拼样式表字符串。
// 刻意没有「信息/蓝色」这一档：原先用蓝表示的就绪/同步中/等待首帧都是
// 「没出问题、还没开始跑」，与 Idle 同类，而多一档就多一次两处色板分叉的机会。
enum class Severity { Idle, Ok, Warn, Fault };

// 语义色。取值来自原 QSS 色板，但施加方式改为 QPalette /
// QTableWidgetItem::setForeground——样式表会级联到后代部件，
// 本次审查里 "QFrame{...}" 污染卡片内每个 QLabel 就是这么来的
//（QLabel 继承自 QFrame）。直接设颜色没有级联，也就没有这一类错误。
QColor severityColor(Severity s);

} // namespace uilogic
