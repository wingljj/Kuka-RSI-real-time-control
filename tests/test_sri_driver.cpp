#include <QtTest>
#include <cstring>
#include "core/SriDriver.h"
#include "core/SriProtocol.h"
#include "core/Wrench.h"

// SriDriver 的白盒协议集成单测：不碰 TCP——以友元身份直接驱动
// processFrames()/drainAccumulator()，验证通道符号/力矩系数、窗口均值
// 与 stale 语义。TCP 连接、AT 握手与重连行为属真机集成验证范围。
class TestSriDriver : public QObject
{
    Q_OBJECT

    static SriFrame frameWith(const float v[6])
    {
        SriFrame f;
        std::memcpy(f.values, v, sizeof(f.values));
        return f;
    }

    static std::vector<SriFrame> oneFrame(const float v[6])
    {
        return {frameWith(v)};
    }

private slots:
    void channelSignsAndTorqueScaleApplied()
    {
        SriDriver drv;
        ForceSensorConfig cfg;
        cfg.channelSigns = {1, -1, 1, -1, 1, 1};
        cfg.torqueScale = 2.0;
        drv.configure(cfg);

        const float v[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
        drv.processFrames(oneFrame(v));
        drv.m_connected = true;  // 白盒：模拟握手完成、进入流式

        WrenchFrame w;
        drv.drainAccumulator(w);
        QVERIFY(w.fresh);
        QCOMPARE(w.fx, 1.0);    // 符号 +1
        QCOMPARE(w.fy, -2.0);   // 符号 -1
        QCOMPARE(w.fz, 3.0);    // 符号 +1
        QCOMPARE(w.mx, -8.0);   // 符号 -1 × 力矩系数 2
        QCOMPARE(w.my, 10.0);   // 符号 +1 × 2
        QCOMPARE(w.mz, 12.0);   // 符号 +1 × 2
    }

    void windowMeanAcrossFrames()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        drv.m_connected = true;

        const float a[6] = {2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f};
        const float b[6] = {4.0f, 6.0f, 8.0f, 10.0f, 12.0f, 14.0f};
        drv.processFrames(oneFrame(a));
        drv.processFrames(oneFrame(b));

        WrenchFrame w;
        drv.drainAccumulator(w);
        QVERIFY(w.fresh);
        QCOMPARE(w.fx, 3.0);
        QCOMPARE(w.fy, 5.0);
        QCOMPARE(w.fz, 7.0);
        QCOMPARE(w.mx, 9.0);
        QCOMPARE(w.my, 11.0);
        QCOMPARE(w.mz, 13.0);
    }

    void drainWithoutFramesMarksStale()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        drv.m_connected = true;

        WrenchFrame w;
        drv.drainAccumulator(w);
        QVERIFY(!w.fresh);
        QCOMPARE(drv.staleCount(), 1);

        drv.drainAccumulator(w);
        QCOMPARE(drv.staleCount(), 2);
    }

    void freshDrainResetsStaleCounter()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        drv.m_connected = true;

        WrenchFrame w;
        drv.drainAccumulator(w);  // stale
        drv.drainAccumulator(w);  // stale
        QCOMPARE(drv.staleCount(), 2);

        const float v[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        drv.processFrames(oneFrame(v));
        drv.drainAccumulator(w);
        QVERIFY(w.fresh);
        QCOMPARE(drv.staleCount(), 0);
    }

    void latestReturnsLastDrainedWrench()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        // 未 drain 前 latest 无新鲜数据
        QVERIFY(!drv.latest().fresh);

        drv.m_connected = true;
        const float v[6] = {5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f};
        drv.processFrames(oneFrame(v));

        WrenchFrame w;
        drv.drainAccumulator(w);
        const WrenchFrame l = drv.latest();
        QVERIFY(l.fresh);
        QCOMPARE(l.fx, 5.0);
        QCOMPARE(l.my, 9.0);
        QCOMPARE(l.mz, 10.0);
    }

    void drainClearsAccumulator()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        drv.m_connected = true;

        const float v[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        drv.processFrames(oneFrame(v));

        WrenchFrame w;
        drv.drainAccumulator(w);
        QVERIFY(w.fresh);

        // 第二次 drain 无新帧 → 陈旧（累加器已被清空，均值不会重复输出）
        drv.drainAccumulator(w);
        QVERIFY(!w.fresh);
        QCOMPARE(drv.staleCount(), 1);
    }

    void framesIgnoredWhileDisconnected()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        // 未连接（m_connected == false）：即使已累加帧，drain 也不得输出 fresh
        const float v[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
        drv.processFrames(oneFrame(v));

        WrenchFrame w;
        drv.drainAccumulator(w);
        QVERIFY(!w.fresh);
        QCOMPARE(drv.staleCount(), 1);
    }

    void latestInitiallyNotFresh()
    {
        SriDriver drv;
        const WrenchFrame l = drv.latest();
        QVERIFY(!l.fresh);
        QCOMPARE(l.fx, 0.0);
        QCOMPARE(l.mz, 0.0);
        QCOMPARE(drv.staleCount(), 0);
        QVERIFY(!drv.isConnected());
    }

    // ── 握手状态机（handleAtResponse，白盒）──

    void handshakeAdvancesAcrossSplitResponse()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        drv.m_sock = new QTcpSocket(&drv);  // 仅占位对象：write() 进缓冲区，不发网络
        drv.m_hsState = SriDriver::HsSentFormat;

        static const char line1[] = "(A01,A02,A03,A04,A05,A06);E;\r\n";
        static const char line2[] = "OK\r\n";
        const auto *p = reinterpret_cast<const uint8_t *>(line1);

        // 行1 跨两次 handleAtResponse 到达
        QVERIFY(!drv.handleAtResponse(p, 9));
        QCOMPARE(drv.m_hsState, SriDriver::HsSentFormat);  // 未凑满行：状态不变

        QVERIFY(drv.handleAtResponse(p + 9, sizeof(line1) - 1 - 9));
        QCOMPARE(drv.m_hsState, SriDriver::HsSentRate);    // (A01, 校验通过 → 已发 SMPF

        QVERIFY(drv.handleAtResponse(
            reinterpret_cast<const uint8_t *>(line2), sizeof(line2) - 1));
        QCOMPARE(drv.m_hsState, SriDriver::HsStreaming);   // 无 ERROR → 已发 GSD
        QVERIFY(drv.isConnected());
    }

    void handshakeRejectsWrongFormatResponse()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        drv.m_sock = new QTcpSocket(&drv);
        drv.m_hsState = SriDriver::HsSentFormat;

        QSignalSpy faultSpy(&drv, &SriDriver::fault);
        static const char bad[] = "ERROR: unknown command\r\n";
        QVERIFY(drv.handleAtResponse(
            reinterpret_cast<const uint8_t *>(bad), sizeof(bad) - 1));
        QCOMPARE(faultSpy.count(), 1);                     // 发出 fault
        QCOMPARE(drv.m_hsState, SriDriver::HsSentFormat);  // 状态不推进
        QVERIFY(drv.m_sock == nullptr);                    // socket 已关闭
    }

    void handshakeRejectsErrorRateResponse()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        drv.m_sock = new QTcpSocket(&drv);
        drv.m_hsState = SriDriver::HsSentRate;

        QSignalSpy faultSpy(&drv, &SriDriver::fault);
        static const char bad[] = "ERROR\r\n";
        QVERIFY(drv.handleAtResponse(
            reinterpret_cast<const uint8_t *>(bad), sizeof(bad) - 1));
        QCOMPARE(faultSpy.count(), 1);
        QVERIFY(drv.m_sock == nullptr);
    }

    void handshakeProcessesMultipleLinesInOneCall()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        drv.m_sock = new QTcpSocket(&drv);
        drv.m_hsState = SriDriver::HsSentFormat;

        static const char two[] = "(A01,A02,A03,A04,A05,A06);E;\r\nOK\r\n";
        QVERIFY(drv.handleAtResponse(
            reinterpret_cast<const uint8_t *>(two), sizeof(two) - 1));
        QCOMPARE(drv.m_hsState, SriDriver::HsStreaming);   // 一次调用推进两行
        QVERIFY(drv.isConnected());
    }

    void handshakeBufferOverflowFailsSafely()
    {
        SriDriver drv;
        drv.configure(ForceSensorConfig{});
        drv.m_sock = new QTcpSocket(&drv);
        drv.m_hsState = SriDriver::HsSentFormat;

        QSignalSpy faultSpy(&drv, &SriDriver::fault);
        uint8_t garbage[130];
        std::memset(garbage, 'x', sizeof(garbage));        // 超 127B 且无 '\n'
        QVERIFY(drv.handleAtResponse(garbage, sizeof(garbage)));
        QCOMPARE(faultSpy.count(), 1);                     // 缓冲满按整行校验 → 拒绝
        QVERIFY(drv.m_sock == nullptr);
    }

    // ── 连接失败重连（onSocketError，白盒）──

    void reconnectScheduledOnSocketError()
    {
        SriDriver drv;
        drv.m_running = true;            // 白盒：模拟 start()
        drv.m_reconnectDelayS = 0.5;
        drv.onSocketError();
        QVERIFY(drv.m_reconnectTimer->isActive());
        QCOMPARE(drv.m_reconnectDelayS, 1.0);

        // 同一次故障 errorOccurred 与 disconnected 双发不得把退避翻倍两次
        drv.onSocketError();
        drv.onDisconnected();
        QCOMPARE(drv.m_reconnectDelayS, 1.0);
        QVERIFY(drv.m_reconnectTimer->isActive());
    }
};
QTEST_MAIN(TestSriDriver)
#include "test_sri_driver.moc"
