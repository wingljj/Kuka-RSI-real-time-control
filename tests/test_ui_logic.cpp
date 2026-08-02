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

    // 上一条只测了严格的 0.0，而 0.0 走不走 kWireQuantum 分支输出都是
    // "0.0000 mm"——把量化阈值改成 0 测试照样绿，这个常量等于没测。
    // 真正需要钉死的是「线上会被量化成 0、但数值上不是 0」的那一段：
    // 幅值小于量化步长时机器人根本不动，界面若打出 "+0.0000"/"-0.0000"，
    // 操作员会以为有一个方向明确的微小修正正在发出。
    void formatRkorrSubQuantumHasNoSign()
    {
        QCOMPARE(uilogic::formatRkorr(0.00003, 0),  QString("0.0000 mm"));
        QCOMPARE(uilogic::formatRkorr(-0.00003, 3), QString("0.0000 deg"));
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
};

QTEST_MAIN(TestUiLogic)
#include "test_ui_logic.moc"
