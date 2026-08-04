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
        t.classify(1001);                        // 建立节拍：步长 1
        const IpocEvent ev = t.classify(1005);   // 缺 1002-1004
        QCOMPARE(int(ev.kind), int(IpocEvent::Gap));
        QCOMPARE(ev.gapCount, quint64(3));
        QCOMPARE(t.lastGood(), quint64(1005));   // Gap 帧推进
    }

    // ── 真机 KRC 节拍：IPOC 是毫秒计数，每帧 +cycle_ms（4ms 模式即 +4）──
    // 现场事故（2026-08-04）：+4 的健康流被逐帧判成 Gap(3)，连续丢包计数
    // 永不清零，一到 watchdog_miss_limit 就静默停跟踪——机械臂纹丝不动、
    // 无任何报错。32535 / 83208 两个现场读数都能被 3 整除即此症状。
    void realKrc4msCadence_isNormal()
    {
        IpocTracker t;
        t.classify(1000);
        QCOMPARE(int(t.classify(1004).kind), int(IpocEvent::Normal));
        QCOMPARE(int(t.classify(1008).kind), int(IpocEvent::Normal));
        QCOMPARE(int(t.classify(1012).kind), int(IpocEvent::Normal));
        QCOMPARE(t.lastGood(), quint64(1012));
    }

    void realKrc12msCadence_isNormal()
    {
        IpocTracker t;
        t.classify(5000);
        QCOMPARE(int(t.classify(5012).kind), int(IpocEvent::Normal));
        QCOMPARE(int(t.classify(5024).kind), int(IpocEvent::Normal));
    }

    void gapAtStep4Cadence_countsMissingFrames()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1004);                        // 建立节拍：步长 4
        const IpocEvent ev = t.classify(1016);   // 缺 1008、1012 两帧
        QCOMPARE(int(ev.kind), int(IpocEvent::Gap));
        QCOMPARE(ev.gapCount, quint64(2));
        QCOMPARE(t.lastGood(), quint64(1016));
    }

    // 首个增量恰好是一次真实丢包（学到偏大的步长）时，之后出现的更小增量
    // 必须被采纳为新步长并判 Normal——错误只影响启动瞬间，且方向是少计。
    void smallerDelta_adoptsNewStep()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1008);                        // 实为步长4丢一帧，被学成步长8
        QCOMPARE(int(t.classify(1012).kind), int(IpocEvent::Normal)); // 步长修正为 4
        const IpocEvent ev = t.classify(1024);   // 缺 1016、1020
        QCOMPARE(int(ev.kind), int(IpocEvent::Gap));
        QCOMPARE(ev.gapCount, quint64(2));
    }

    void reset_clearsLearnedStep()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1004);                        // 步长 4
        t.reset();
        t.classify(2000);                        // First
        QCOMPARE(int(t.classify(2001).kind), int(IpocEvent::Normal)); // 重新学步长 1
        const IpocEvent ev = t.classify(2005);
        QCOMPARE(int(ev.kind), int(IpocEvent::Gap));
        QCOMPARE(ev.gapCount, quint64(3));
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
