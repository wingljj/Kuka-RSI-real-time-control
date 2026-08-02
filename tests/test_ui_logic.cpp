#include <QtTest>
#include "ui/UiLogic.h"

namespace {

// 构造一个处于 Tracking 且累计超限的快照
StatusSnapshot tracking(bool over, int missed)
{
    StatusSnapshot s;
    s.state          = ControlState::Tracking;
    s.connected      = true;
    s.accumOverLimit = over;
    s.missedCount    = missed;
    return s;
}

} // namespace

class TestUiLogic : public QObject
{
    Q_OBJECT
private slots:
    // ── 告警边沿触发（缺陷 F）──

    void alarmFiresOnceOnRisingEdge()
    {
        AlarmEdge prev;                       // 全 false
        const StatusSnapshot s = tracking(true, 0);

        const AlarmEdge first = uilogic::risingEdges(prev, s);
        QVERIFY(first.accumOverLimit);        // 第一次：记一条

        prev = uilogic::currentAlarms(s);
        const AlarmEdge second = uilogic::risingEdges(prev, s);
        QVERIFY(!second.accumOverLimit);      // 持续为真：不再记
    }

    void alarmRefiresAfterClearing()
    {
        AlarmEdge prev = uilogic::currentAlarms(tracking(true, 0));

        // 告警消失
        const StatusSnapshot ok = tracking(false, 0);
        QVERIFY(!uilogic::risingEdges(prev, ok).accumOverLimit);
        prev = uilogic::currentAlarms(ok);

        // 再次出现：应重新记一条
        const StatusSnapshot bad = tracking(true, 0);
        QVERIFY(uilogic::risingEdges(prev, bad).accumOverLimit);
    }

    void accumAlarmOnlyWhenTracking()
    {
        // 非 Tracking 下 accumOverLimit 不该产生告警：控制器没在发增量
        StatusSnapshot s = tracking(true, 0);
        s.state = ControlState::Ready;
        QVERIFY(!uilogic::currentAlarms(s).accumOverLimit);
    }

    void lossAlarmNeedsConnection()
    {
        StatusSnapshot s = tracking(false, 5);
        QVERIFY(uilogic::currentAlarms(s).packetLoss);
        s.connected = false;                  // 断开时的 missedCount 无意义
        QVERIFY(!uilogic::currentAlarms(s).packetLoss);
    }

    // 缺陷 F 有两条告警通道，边沿逻辑必须逐条测。只测 accumOverLimit 时，
    // 把 packetLoss 的 `&& !prev` 整个删掉测试仍全绿——而持续丢包下
    // missedCount>0 && connected 每帧为真，日志照样 4 秒刷满。
    void lossAlarmFiresOnceOnRisingEdge()
    {
        AlarmEdge prev;                       // 全 false
        const StatusSnapshot s = tracking(false, 3);

        const AlarmEdge first = uilogic::risingEdges(prev, s);
        QVERIFY(first.packetLoss);            // 第一次：记一条

        prev = uilogic::currentAlarms(s);
        const AlarmEdge second = uilogic::risingEdges(prev, s);
        QVERIFY(!second.packetLoss);          // 持续为真：不再记
    }

    void lossAlarmRefiresAfterClearing()
    {
        AlarmEdge prev = uilogic::currentAlarms(tracking(false, 3));

        // 丢包恢复：missedCount 归零
        const StatusSnapshot ok = tracking(false, 0);
        QVERIFY(!uilogic::risingEdges(prev, ok).packetLoss);
        prev = uilogic::currentAlarms(ok);

        // 再次丢包：应重新记一条
        const StatusSnapshot bad = tracking(false, 3);
        QVERIFY(uilogic::risingEdges(prev, bad).packetLoss);
    }

    // ── 丢包迟滞（缺陷 F 的另一半）──
    //
    // 边沿触发只压得住「持续为真」的告警。missedCount 不是那样的信号：
    // RsiWorker 在每个正常帧把它归零，于是间歇丢包在快照里是 0/1/0/1 的
    // 脉冲串，每个脉冲都是货真价实的上升沿。这一组测试锁住迟滞把脉冲拉平
    // 成电平这件事——没有它，krc_simulator --drop 5 实测 8.5 秒记 142 条。

    void intermittentLossLogsOnceNotPerPulse()
    {
        LossHold h;
        AlarmEdge prev;
        int logged = 0;
        // 模拟 --drop 5：每 5 帧丢 1 帧，跑 300 个刷新帧
        for (int i = 0; i < 300; ++i) {
            const StatusSnapshot s = tracking(false, (i % 5 == 0) ? 1 : 0);
            const AlarmEdge now = uilogic::currentAlarmsHeld(h, s, 50);
            if (uilogic::edgesBetween(prev, now).packetLoss)
                ++logged;
            prev = now;
        }
        // 纯边沿触发在这里会记 60 条；迟滞把整段并成一条
        QCOMPARE(logged, 1);
    }

    void lossHoldRecoversAfterQuietPeriod()
    {
        LossHold h;
        AlarmEdge prev;
        const int kClear = 10;
        int logged = 0;

        auto feed = [&](int missed) {
            const StatusSnapshot s = tracking(false, missed);
            const AlarmEdge now = uilogic::currentAlarmsHeld(h, s, kClear);
            if (uilogic::edgesBetween(prev, now).packetLoss)
                ++logged;
            prev = now;
        };

        feed(1);
        QCOMPARE(logged, 1);
        for (int i = 0; i < kClear; ++i) feed(0);   // 安静足够久 → 这一段结束
        feed(1);                                   // 新的一段：必须再记一条
        QCOMPARE(logged, 2);
    }

    // 恢复必须要求「连续」干净，而不是「这一帧」干净。若把 quietFrames
    // 的累加换成单帧判定，0/1/0/1 的每个 0 都算恢复，迟滞形同虚设。
    void lossHoldNeedsConsecutiveQuietFrames()
    {
        LossHold h;
        AlarmEdge prev;
        int logged = 0;
        auto feed = [&](int missed) {
            const StatusSnapshot s = tracking(false, missed);
            const AlarmEdge now = uilogic::currentAlarmsHeld(h, s, 10);
            if (uilogic::edgesBetween(prev, now).packetLoss)
                ++logged;
            prev = now;
        };
        feed(1);
        for (int i = 0; i < 30; ++i) { feed(0); feed(0); feed(1); }  // 干净帧数永不连续到 10
        QCOMPARE(logged, 1);
    }

    // 断开必须结束这一段：否则重连后的第一次丢包被上一次会话的 active
    // 吞掉，操作员在新会话里看不到任何丢包记录。
    void disconnectEndsLossSegment()
    {
        LossHold h;
        AlarmEdge prev;
        int logged = 0;
        auto feed = [&](const StatusSnapshot &s) {
            const AlarmEdge now = uilogic::currentAlarmsHeld(h, s, 1000);
            if (uilogic::edgesBetween(prev, now).packetLoss)
                ++logged;
            prev = now;
        };
        feed(tracking(false, 1));
        QCOMPARE(logged, 1);
        StatusSnapshot down = tracking(false, 0);
        down.connected = false;
        feed(down);
        feed(tracking(false, 1));       // 重连后再丢包：必须重新记
        QCOMPARE(logged, 2);
    }

    // 累计超限走的仍是纯电平，不该被丢包迟滞影响。
    void heldAlarmsLeaveAccumUntouched()
    {
        LossHold h;
        const StatusSnapshot s = tracking(true, 0);
        QVERIFY(uilogic::currentAlarmsHeld(h, s, 50).accumOverLimit);
    }

    // ── 按钮启用状态（缺陷 H 与状态机）──

    void faultEnablesOnlyReset()
    {
        StatusSnapshot s;
        s.state = ControlState::Fault;
        s.connected = true;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(b.resetFault);
        QVERIFY(!b.enableTrack);              // Fault 必须先复位
        // stopTrack 是 Tracking || StaleFrame || Syncing 的 OR 链，而 OR 链
        // 天然诱导后来者继续追加分支。这里必须钉死 Fault 不在链内：
        // 「停止跟踪」走 onZeroToActual → PoseController::resetToActual，
        // 而 resetToActual 会清除 Fault。一旦有人加上 `|| Fault`，停止跟踪
        // 就成了一次绕过 onResetFault 日志记录的隐式故障复位——故障发生过
        // 这件事会从事件日志里消失。
        QVERIFY(!b.stopTrack);                // Fault 下停止跟踪也无意义
    }

    // 复位不该受监听状态影响：Fault 是锁存的，停止监听不会清除它。
    // 若 resetFault 附加 `&& listening`，操作员停掉监听后就再也清不掉故障，
    // 只能重启程序。
    void resetFaultIgnoresListening()
    {
        StatusSnapshot s;
        s.state = ControlState::Fault;
        QVERIFY(uilogic::buttonStates(s, true).resetFault);
        QVERIFY(uilogic::buttonStates(s, false).resetFault);
    }

    void readyEnablesTracking()
    {
        StatusSnapshot s;
        s.state = ControlState::Ready;
        s.connected = true;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(b.enableTrack);
        QVERIFY(!b.resetFault);
        QVERIFY(!b.stopTrack);
    }

    void trackingEnablesOnlyStop()
    {
        StatusSnapshot s;
        s.state = ControlState::Tracking;
        s.connected = true;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(b.stopTrack);
        QVERIFY(!b.enableTrack);
    }

    // StaleFrame 下 ControlState 已不是 Tracking，但 PoseController 内部仍是
    // TrackState::Tracking、仍在发增量。此时最需要能停——反馈已经异常而机器人
    // 还在动。若 stopTrack 只认 Tracking，这里按钮是灰的，操作员只能干看着。
    void staleFrameStillAllowsStop()
    {
        StatusSnapshot s;
        s.state = ControlState::StaleFrame;
        s.connected = true;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(b.stopTrack);
        QVERIFY(!b.enableTrack);              // 使能仍严格限于 Ready
        QVERIFY(!b.resetFault);
    }

    // Syncing 同理：首帧对齐的瞬间，控制器已进入跟踪路径。
    void syncingStillAllowsStop()
    {
        StatusSnapshot s;
        s.state = ControlState::Syncing;
        s.connected = true;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(b.stopTrack);
        QVERIFY(!b.enableTrack);
        QVERIFY(!b.resetFault);
    }

    void notListeningLocksEverything()
    {
        StatusSnapshot s;                     // Disconnected
        const ButtonStates b = uilogic::buttonStates(s, false);
        QVERIFY(b.startListen);
        QVERIFY(!b.stopListen);
        QVERIFY(b.connEditable);              // 仅未监听时可改 IP/端口
        QVERIFY(!b.enableTrack);
    }

    void listeningLocksConnConfig()
    {
        StatusSnapshot s;
        s.state = ControlState::WaitingFirstFrame;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(!b.startListen);
        QVERIFY(b.stopListen);
        QVERIFY(!b.connEditable);
    }

    // ── 数值格式化 ──

    void formatValueUsesThreeDecimals()
    {
        QCOMPARE(uilogic::formatValue(1280.4, 0), QString("1280.400 mm"));
        QCOMPARE(uilogic::formatValue(-0.5, 4),   QString("-0.500 deg"));
    }

    void formatRkorrUsesFourDecimals()
    {
        // 4 位小数 = RsiCodec::buildSen 的线上量化位数。显示 3 位会把
        // 0.00005 这种「线上就是 0」的量显示成 0.000，看不出差别。
        QCOMPARE(uilogic::formatRkorr(0.00312, 0), QString("+0.0031 mm"));
        QCOMPARE(uilogic::formatRkorr(-0.00312, 3), QString("-0.0031 deg"));
    }

    void formatRkorrZeroHasNoSign()
    {
        // 零增量是常态（未跟踪时每帧都是零），带个 "+0.0000" 很吵
        QCOMPARE(uilogic::formatRkorr(0.0, 0), QString("0.0000 mm"));
    }

    // 上一条只测了严格的 0.0，而 0.0 走不走 kWireRoundToZero 分支输出都是
    // "0.0000 mm"——把量化阈值改成 0 测试照样绿，这个常量等于没测。
    // 真正需要钉死的是「线上会被量化成 0、但数值上不是 0」的那一段：
    // 幅值小于量化步长时机器人根本不动，界面若打出 "+0.0000"/"-0.0000"，
    // 操作员会以为有一个方向明确的微小修正正在发出。
    void formatRkorrSubQuantumHasNoSign()
    {
        QCOMPARE(uilogic::formatRkorr(0.00003, 0),  QString("0.0000 mm"));
        QCOMPARE(uilogic::formatRkorr(-0.00003, 3), QString("0.0000 deg"));
    }

    // 阈值必须是 5e-5（半个线上步长），不能是 PoseController 里那个同名的
    // 1e-4（步长本身）。0.00007 正落在两者之间：线上四舍五入后真发 0.0001、
    // 机器人真的会动，界面必须显示 0.0001 而不是 0.0000。少了这条，把阈值
    // 从 5e-5 「统一」成 1e-4 的改动测试拦不住。
    void formatRkorrKeepsValuesAboveHalfQuantum()
    {
        QCOMPARE(uilogic::formatRkorr(0.00007, 0), QString("+0.0001 mm"));
    }

    // 颜色判定（MainWindow::onRefresh）与格式化（formatRkorr）必须共用同一个
    // 零值门限。两处各写一个 5e-5 时，改一处漏一处就会出现「显示 0.0000 却
    // 标成非零色」这类自相矛盾的格。这里断言二者永不打架：isRkorrZero 为真
    // 的值格式化后必然不带正负号（即被显示成零），为假的必然带号。
    void isRkorrZeroAgreesWithFormatRkorr()
    {
        const double samples[] = {0.0,      3e-5,   -3e-5,  4.9e-5, -4.9e-5,
                                  5.1e-5,  -5.1e-5, 7e-5,   1e-4,   -1e-4,
                                  0.00312, -0.00312};
        for (double v : samples) {
            for (int axis = 0; axis < 6; ++axis) {
                const QString t = uilogic::formatRkorr(v, axis);
                const bool signed_ = t.startsWith('+') || t.startsWith('-');
                QVERIFY2(uilogic::isRkorrZero(v) != signed_,
                         qPrintable(QStringLiteral("v=%1 axis=%2 text=%3")
                                        .arg(v, 0, 'g', 6).arg(axis).arg(t)));
            }
        }
    }

    // 门限本身也要钉死在半个线上步长上：只测「两者一致」时，把 isRkorrZero
    // 改成恒真、formatRkorr 跟着永远返回 0.0000，上一条照样绿。
    void isRkorrZeroUsesHalfWireQuantum()
    {
        QVERIFY(uilogic::isRkorrZero(4.9e-5));
        QVERIFY(!uilogic::isRkorrZero(5.1e-5));
        QVERIFY(!uilogic::isRkorrZero(7e-5));   // 线上真发 0.0001，机器人真的在动
    }

    // ── 差值预览（缺陷 K）──

    void deltaPreviewReportsNoDeviation()
    {
        // 原代码里 delta 起手就写入了前缀，isEmpty() 永不为真，
        // 「无偏差」是死代码
        Pose actual;
        actual.x = 100.0;
        const double target[6] = {100.0, 0, 0, 0, 0, 0};
        QVERIFY(uilogic::deltaPreview(target, actual).contains("无偏差"));
    }

    void deltaPreviewListsDeviatingAxesOnly()
    {
        Pose actual;
        actual.x = 100.0;
        actual.z = 50.0;
        const double target[6] = {105.0, 0, 50.0, 0, 0, 0};
        const QString s = uilogic::deltaPreview(target, actual);
        QVERIFY(s.contains("X"));
        QVERIFY(!s.contains("Z"));       // Z 无偏差，不该列出
        QVERIFY(!s.contains("无偏差"));
    }

    // 上面两条都用「精确相等」当无偏差样本，于是把容差 0.005 改成 0 测试
    // 仍全绿——容差本身没被测到。而容差是有实际含义的：目标框是 3 位小数
    // 输入，实际位姿是 RSI 反馈的连续量，两者永远不会位到位精确相等。
    // 没有容差的话「无偏差」分支在真机上永不可达，缺陷 K 就以另一种形式
    // 复活了。这里用一个亚容差偏差把这条线钉住。
    void deltaPreviewIgnoresSubToleranceDeviation()
    {
        Pose actual;
        actual.y = 0.001;                // 远小于 0.005，属于噪声不是偏差
        const double target[6] = {0, 0, 0, 0, 0, 0};
        QVERIFY(uilogic::deltaPreview(target, actual).contains("无偏差"));
    }

    // 上面几条都只用 contains("X") 做断言，于是一整类错误测不出来：
    // 符号反转（标签写「目标 − 当前」，反了操作员就把修正方向读反）、
    // 轴名取错下标（全打成 X）、单位取错下标（角度轴显示 mm）。
    // 用一个非 X 的角度轴造偏差 + QCOMPARE 整串，一次把三者钉死。
    void deltaPreviewSignAxisNameAndUnit()
    {
        Pose actual;                     // 全零
        const double target[6] = {0, 0, 0, 0, 0, 5.0};   // C 目标 +5
        QCOMPARE(uilogic::deltaPreview(target, actual),
                 QString("目标 − 当前：C 5.000 deg"));
    }

    // 循环上界必须覆盖到 C：写成 i < 5 时，操作员改了 C 目标而界面回
    // 「无偏差」——最坏的一种错，因为它看起来像「已确认无事」。
    void deltaPreviewCoversLastAxis()
    {
        Pose actual;
        const double target[6] = {0, 0, 0, 0, 0, 1.0};
        QVERIFY(!uilogic::deltaPreview(target, actual).contains("无偏差"));
    }

    // 缺陷 C1：姿态轴必须走最短角路径。目标 A=-179.999、实际 A=179.999 时
    // 裸减法得 -359.998°，而 PoseController::step 用四元数只会走 0.002°——
    // 预览与控制器行为直接矛盾，界面在教操作员一件与机器相反的事。
    // KUKA 工具朝下时 C≈±180 是常见姿态，RSI 反馈在 180 附近换符号是常态，
    // 目标输入框范围又恰好是 -180..180，所以这是日常路径而非理论边界。
    void deltaPreviewUsesShortestAnglePath()
    {
        // 审查者给的原始例子：真实最短路径是 0.002°，而它恰好落在 0.005 容差
        // 之内，所以正确输出是「无偏差」。裸减法会得到 -359.998（远超容差）
        // 并打出 "A -359.998 deg"——两者对操作员的含义天差地别。
        Pose near;
        near.a = 179.999;
        const double tNear[6] = {0, 0, 0, -179.999, 0, 0};
        const QString sNear = uilogic::deltaPreview(tNear, near);
        QVERIFY(sNear.contains("无偏差"));
        QVERIFY(!sNear.contains("359"));   // 绕远路的那个数字不该出现

        // 超出容差的环绕：真实最短路径 2°，必须显示 2 而不是 -358。
        // 上面那条只能证明「没打出 359」，证不了环绕后的数值算得对，
        // 因为 wrap180 换成恒返回 0 它也绿。这条把数值本身钉死。
        Pose far;
        far.a = 179.0;
        const double tFar[6] = {0, 0, 0, -179.0, 0, 0};
        QCOMPARE(uilogic::deltaPreview(tFar, far),
                 QString("目标 − 当前：A 2.000 deg"));
    }

    // 位置轴反过来必须保持裸减：X 走直线，没有 360 环绕一说。
    // 若有人图省事把 wrap180 套到全部 6 轴，1280mm 的目标会被折成 -160mm。
    void deltaPreviewDoesNotWrapLinearAxes()
    {
        Pose actual;                     // 全零
        const double target[6] = {1280.0, 0, 0, 0, 0, 0};
        QCOMPARE(uilogic::deltaPreview(target, actual),
                 QString("目标 − 当前：X 1280.00 mm"));
    }
};

QTEST_MAIN(TestUiLogic)
#include "test_ui_logic.moc"
