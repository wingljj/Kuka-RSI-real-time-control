#include "ui/UiLogic.h"

#include <QFontDatabase>
#include <QStringList>
#include <algorithm>
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

AlarmEdge edgesBetween(const AlarmEdge &prev, const AlarmEdge &now)
{
    AlarmEdge e;
    e.accumOverLimit = now.accumOverLimit && !prev.accumOverLimit;
    e.packetLoss     = now.packetLoss     && !prev.packetLoss;
    return e;
}

AlarmEdge risingEdges(const AlarmEdge &prev, const StatusSnapshot &s)
{
    return edgesBetween(prev, currentAlarms(s));
}

AlarmEdge currentAlarmsHeld(LossHold &h, const StatusSnapshot &s, int clearFrames)
{
    AlarmEdge a = currentAlarms(s);

    // 断开即结束这一段：重连后的第一次丢包应当重新记一条，而不是被
    // 上一次会话残留的 active 吞掉。
    if (!s.connected) {
        h.active      = false;
        h.quietFrames = 0;
        a.packetLoss  = false;
        return a;
    }

    if (a.packetLoss) {
        h.active      = true;
        h.quietFrames = 0;
    } else if (h.active) {
        // 恢复必须靠「连续干净」而不是「这一帧干净」：missedCount 在每个
        // 正常帧被归零，只看单帧的话 0/1/0/1 的脉冲串每个 0 都算恢复，
        // 下一个 1 又是新的一段，等于没有迟滞。
        if (++h.quietFrames >= std::max(1, clearFrames))
            h.active = false;
    }

    a.packetLoss = h.active;
    return a;
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

// ── 误差列（旋转向量）──

QString errorAxisLabel(int axis)
{
    // 位置三行沿用 X/Y/Z：误差就是 target − actual，与行首同义，
    // 给它们也换个名字只会让真正需要警觉的那三行淹没在噪声里。
    static const char *kRotLabel[3] = {"Rx", "Ry", "Rz"};
    return (axis < 3) ? QString::fromLatin1(kAxisName[axis])
                      : QString::fromLatin1(kRotLabel[axis - 3]);
}

QString formatError(double v, int axis)
{
    if (axis < 3)
        return formatValue(v, axis);
    // 姿态三行把 Rx/Ry/Rz 写进单元格本身。选前缀而非仅 tooltip 的理由：
    // 这一列每 20ms 刷新一次、操作员是扫读，tooltip 要悬停才出现，
    // 而误读的代价是把一个正确结果当成 bug 报上来。前缀让「这不是 A/B/C」
    // 在视线扫过的那一瞬间就成立。
    return QStringLiteral("%1 %2").arg(errorAxisLabel(axis),
                                       formatValue(v, axis));
}

QString errorColumnTooltip()
{
    return QStringLiteral(
        "误差列的含义\n"
        "\n"
        "位置 X/Y/Z：目标 − 当前，直接相减。\n"
        "\n"
        "姿态 Rx/Ry/Rz：把当前姿态一次转到目标姿态的最短旋转（SO(3)），\n"
        "分解到世界坐标 X/Y/Z 轴上的三个分量，单位度。\n"
        "它不是 A、B、C 三个欧拉角各自的差值——两者只在小角度下才接近，\n"
        "在大角度、尤其万向节死锁（B≈±90°）附近可以完全不同。\n"
        "\n"
        "例：目标 ABC=(70, 90, 70)，当前 ABC=(0, 0, 0)，本列显示 0 / 90 / 0。\n"
        "这是对的：B=90° 时 A 与 C 绕同一条轴旋转，(70, 90, 70) 与 (0, 90, 0)\n"
        "是同一个姿态，最短路径就是绕世界 Y 轴转 90°。\n"
        "\n"
        "本列显示的就是控制器实际会走的那条路径（RKorr 输出列即由它算出），\n"
        "所以它比逐轴欧拉角差更能说明机器人接下来会怎么动。");
}

QString errorCellTooltip(int axis)
{
    if (axis < 3)
        return QString();   // 位置行就是相减，无需解释
    static const char *kWorldAxis[3] = {"X", "Y", "Z"};
    return QStringLiteral(
               "%1：最短旋转绕世界 %2 轴的分量（度）。\n"
               "不是 %3 轴的欧拉角差值。")
        .arg(errorAxisLabel(axis),
             QString::fromLatin1(kWorldAxis[axis - 3]),
             QString::fromLatin1(kAxisName[axis]));
}

bool isRkorrZero(double v)
{
    // 与 buildSen 的量化步长对齐：小于半个步长的量线上就是 0，
    // 显示成 0 是如实反映，不是精度损失。
    //
    // 刻意不叫 kWireQuantum：PoseController.cpp 里另有一个同名常量，值是
    // 1e-4（线上量化步长本身，用作死区——小于它的增量线上发不出去，不该
    // 计入累积账本）。这里要的是「4 位小数四舍五入到 0」的阈值，即那个
    // 步长的一半。两者语义不同、值也必须不同：若有人 grep 到两个同名常量
    // 值不一致、顺手「统一」成 1e-4，界面就会把 0.00007 显示成 0.0000，
    // 而线上真发的是 0.0001、机器人真的在动。取不同的名字断掉这个念头。
    constexpr double kWireRoundToZero = 5e-5;
    return std::fabs(v) < kWireRoundToZero;
}

QString formatRkorr(double v, int axis)
{
    if (isRkorrZero(v))
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
        // 姿态轴取最短角路径，位置轴裸减。预览必须与控制器实际会走的路径
        // 一致，否则界面在教操作员一件与机器行为相反的事：PoseController::step
        // 用四元数算最短旋转，目标 A=-179.999 / 实际 A=179.999 时它只走 0.002°，
        // 而裸减法会把这显示成 -359.998°。这不是理论边界——KUKA 工具朝下时
        // C≈±180 是常见姿态，RSI 反馈在 180 附近换符号是常态，而目标输入框
        // 的范围恰好是 -180..180，两者一撞就出「偏差 360 度」。
        const double d = (i < 3) ? (target[i] - act[i])
                                 : wrap180(target[i] - act[i]);
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

QFont monospaceFont()
{
    // 向系统要等宽字体，而不是点名 Consolas：QFont 找不到指定族时不会报错，
    // 只会悄悄换成默认比例字体，于是「数值列右对齐」看起来还在、小数点却
    // 参差不齐。系统字体在任何 Qt 支持的平台上都保证存在。
    return QFontDatabase::systemFont(QFontDatabase::FixedFont);
}

} // namespace uilogic
