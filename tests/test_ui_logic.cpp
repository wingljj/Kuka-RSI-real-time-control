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
};

QTEST_MAIN(TestUiLogic)
#include "test_ui_logic.moc"
