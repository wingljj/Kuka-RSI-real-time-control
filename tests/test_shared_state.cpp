#include <QtTest>
#include <vector>
#include "net/SharedState.h"

class TestSharedState : public QObject
{
    Q_OBJECT
private slots:
    void ring_emptyCopiesNothing()
    {
        SampleRing r;
        std::vector<ChartSample> out(8);
        QCOMPARE(r.copyOut(out.data(), 8), 0);
    }

    void ring_partiallyFilledPreservesOrder()
    {
        SampleRing r;
        for (int i = 0; i < 5; ++i)
            r.push(ChartSample{double(i), double(i) * 2.0, double(i) * 3.0});
        std::vector<ChartSample> out(8);
        QCOMPARE(r.copyOut(out.data(), 8), 5);
        for (int i = 0; i < 5; ++i) {
            QCOMPARE(out[i].tSec, double(i));
            QCOMPARE(out[i].posErrNorm, double(i) * 2.0);
            QCOMPARE(out[i].rotErrNorm, double(i) * 3.0);
        }
    }

    void ring_exactlyFullPreservesOrder()
    {
        SampleRing r;
        for (int i = 0; i < SampleRing::kCapacity; ++i)
            r.push(ChartSample{double(i), 0.0, 0.0});
        std::vector<ChartSample> out(SampleRing::kCapacity);
        QCOMPARE(r.copyOut(out.data(), SampleRing::kCapacity),
                 SampleRing::kCapacity);
        QCOMPARE(out.front().tSec, 0.0);
        QCOMPARE(out.back().tSec, double(SampleRing::kCapacity - 1));
    }

    void ring_wrappedKeepsNewestInOrder()
    {
        SampleRing r;
        // 多写 100 个，最旧的 100 个应被覆盖
        const int total = SampleRing::kCapacity + 100;
        for (int i = 0; i < total; ++i)
            r.push(ChartSample{double(i), 0.0, 0.0});
        std::vector<ChartSample> out(SampleRing::kCapacity);
        QCOMPARE(r.copyOut(out.data(), SampleRing::kCapacity),
                 SampleRing::kCapacity);
        // 最旧的应是 total - kCapacity，最新的是 total - 1，且严格递增
        QCOMPARE(out.front().tSec, double(total - SampleRing::kCapacity));
        QCOMPARE(out.back().tSec, double(total - 1));
        for (int i = 1; i < SampleRing::kCapacity; ++i)
            QVERIFY(out[i].tSec > out[i - 1].tSec);
    }

    void ring_maxCountLimitsButKeepsNewest()
    {
        SampleRing r;
        for (int i = 0; i < 50; ++i)
            r.push(ChartSample{double(i), 0.0, 0.0});
        std::vector<ChartSample> out(10);
        QCOMPARE(r.copyOut(out.data(), 10), 10);
        // 截断时必须保留最新的 10 个，而不是最旧的
        QCOMPARE(out.front().tSec, 40.0);
        QCOMPARE(out.back().tSec, 49.0);
    }

    void ring_clearEmptiesIt()
    {
        SampleRing r;
        for (int i = 0; i < 20; ++i)
            r.push(ChartSample{double(i), 0.0, 0.0});
        r.clear();
        std::vector<ChartSample> out(8);
        QCOMPARE(r.copyOut(out.data(), 8), 0);
        // clear 之后仍可正常写入
        r.push(ChartSample{99.0, 0.0, 0.0});
        QCOMPARE(r.copyOut(out.data(), 8), 1);
        QCOMPARE(out[0].tSec, 99.0);
    }

    void state_defaultSnapshotIsDisconnected()
    {
        SharedState s;
        const StatusSnapshot snap = s.snapshot();
        QVERIFY(!snap.connected);
        QCOMPARE(snap.frameCount, quint64(0));
        QCOMPARE(snap.state, ControlState::Disconnected);
    }

    void state_publishRoundTrips()
    {
        SharedState s;
        StatusSnapshot in;
        in.actual = Pose{1, 2, 3, 4, 5, 6};
        in.error  = Pose{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
        in.ipoc   = 123456789ULL;
        in.state  = ControlState::Fault;
        in.faultReason = "boom";
        in.missedCount = 7;
        in.measuredCycleMs = 12.5;
        // 瞬时与最大是两个独立字段，刻意给不同的值：它们曾经共用 maxReplyUs
        // 一个字段，界面上「当前」与「最大」两行因此永远相同，一次尖峰会
        // 永久污染「当前」读数并把卡片永远锁在告警色。
        in.replyUs    = 180.0;
        in.maxReplyUs = 250.0;
        in.frameCount = 999;
        in.connected = true;
        s.publish(in);

        const StatusSnapshot out = s.snapshot();
        QCOMPARE(out.actual.x, 1.0);
        QCOMPARE(out.actual.c, 6.0);
        QCOMPARE(out.error.a, 0.4);
        QCOMPARE(out.ipoc, quint64(123456789));
        QCOMPARE(out.state, ControlState::Fault);
        QCOMPARE(out.faultReason, QString("boom"));
        QCOMPARE(out.missedCount, 7);
        QCOMPARE(out.measuredCycleMs, 12.5);
        QCOMPARE(out.replyUs, 180.0);
        QCOMPARE(out.maxReplyUs, 250.0);
        QCOMPARE(out.frameCount, quint64(999));
        QVERIFY(out.connected);
    }

    void ring_rejectsNonPositiveMaxCount()
    {
        SampleRing r;
        for (int i = 0; i < 10; ++i)
            r.push(ChartSample{double(i), 0.0, 0.0});
        std::vector<ChartSample> out(8);
        // 负数或零必须直接返回 0，而不是返回负数或触发有符号溢出
        QCOMPARE(r.copyOut(out.data(), 0), 0);
        QCOMPARE(r.copyOut(out.data(), -1), 0);
        QCOMPARE(r.copyOut(nullptr, 8), 0);
    }

    void ring_wrapBoundaryExactSplit()
    {
        // 让 start 恰好落在末尾，迫使 memcpy 走两段路径且第二段长度为 n-1
        SampleRing r;
        const int total = SampleRing::kCapacity + 1;   // m_head == 1
        for (int i = 0; i < total; ++i)
            r.push(ChartSample{double(i), 0.0, 0.0});
        std::vector<ChartSample> out(SampleRing::kCapacity);
        QCOMPARE(r.copyOut(out.data(), SampleRing::kCapacity),
                 SampleRing::kCapacity);
        QCOMPARE(out.front().tSec, 1.0);
        QCOMPARE(out.back().tSec, double(total - 1));
        for (int i = 1; i < SampleRing::kCapacity; ++i)
            QVERIFY(out[i].tSec > out[i - 1].tSec);
    }

    void ring_singleElementAfterWrap()
    {
        SampleRing r;
        for (int i = 0; i < SampleRing::kCapacity + 5; ++i)
            r.push(ChartSample{double(i), 0.0, 0.0});
        std::vector<ChartSample> out(1);
        QCOMPARE(r.copyOut(out.data(), 1), 1);
        // 只要一个点时必须是最新的那个
        QCOMPARE(out[0].tSec, double(SampleRing::kCapacity + 4));
    }
};

QTEST_MAIN(TestSharedState)
#include "test_shared_state.moc"
