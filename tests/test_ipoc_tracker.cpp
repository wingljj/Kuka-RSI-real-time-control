#include <QtTest>
#include "core/IpocTracker.h"

class TestIpocTracker : public QObject
{
    Q_OBJECT
private slots:
    void firstFrame_returnsFirst()
    {
        IpocTracker t;
        const IpocEvent ev = t.classify(1000);
        QCOMPARE(int(ev.kind), int(IpocEvent::First));
        QVERIFY(t.haveFirst());
        QCOMPARE(t.lastGood(), quint64(1000));
    }

    void normalSequence_returnsNormal()
    {
        IpocTracker t;
        t.classify(1000);
        const IpocEvent ev = t.classify(1001);
        QCOMPARE(int(ev.kind), int(IpocEvent::Normal));
        QCOMPARE(ev.gapCount, quint64(0));
        QCOMPARE(t.lastGood(), quint64(1001));
    }

    void duplicate_doesNotAdvanceLastGood()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1001);
        const IpocEvent ev = t.classify(1001);   // 重复
        QCOMPARE(int(ev.kind), int(IpocEvent::Duplicate));
        QCOMPARE(t.lastGood(), quint64(1001));   // 不推进
    }

    void backward_doesNotAdvanceLastGood()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1001);
        const IpocEvent ev = t.classify(999);    // 回退
        QCOMPARE(int(ev.kind), int(IpocEvent::Backward));
        QCOMPARE(t.lastGood(), quint64(1001));
    }

    void gap_countsMissingCycles()
    {
        IpocTracker t;
        t.classify(1000);
        const IpocEvent ev = t.classify(1004);   // 缺 1001-1003
        QCOMPARE(int(ev.kind), int(IpocEvent::Gap));
        QCOMPARE(ev.gapCount, quint64(3));
        QCOMPARE(t.lastGood(), quint64(1004));   // Gap 帧推进
    }

    void gapOfOne_isNormal()
    {
        IpocTracker t;
        t.classify(1000);
        const IpocEvent ev = t.classify(1001);
        QCOMPARE(int(ev.kind), int(IpocEvent::Normal));
        QCOMPARE(ev.gapCount, quint64(0));
    }

    void gap_thenNormal_stillWorks()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1004);                        // Gap
        const IpocEvent ev = t.classify(1005);
        QCOMPARE(int(ev.kind), int(IpocEvent::Normal));
        QCOMPARE(t.lastGood(), quint64(1005));
    }

    void reset_startsFreshSequence()
    {
        IpocTracker t;
        t.classify(1000);
        t.reset();
        QVERIFY(!t.haveFirst());
        const IpocEvent ev = t.classify(500);    // 重置后首帧，即使更小
        QCOMPARE(int(ev.kind), int(IpocEvent::First));
    }

    void duplicateThenNormal_resumesCorrectly()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1001);
        t.classify(1001);                        // Duplicate，lastGood 仍 1001
        const IpocEvent ev = t.classify(1002);   // 正常
        QCOMPARE(int(ev.kind), int(IpocEvent::Normal));
    }
};
QTEST_MAIN(TestIpocTracker)
#include "test_ipoc_tracker.moc"
