#pragma once
#include <QMutex>
#include <QObject>
#include <QTcpSocket>
#include <QTimer>
#include <vector>
#include "core/AppConfig.h"
#include "core/SriProtocol.h"
#include "core/Wrench.h"

// SRI 六维力传感器 TCP 读取驱动。
//
// 线程模型：SriDriver 自身持有 QTcpSocket，socket 事件（connected/readyRead/
// disconnected/重连定时器）在其事件循环线程处理；窗口均值累加器由该线程累加，
// 由通信线程按 RSI 周期 drainAccumulator() 取走均值。latest()/isConnected()/
// staleCount() 加锁，可跨线程调用。
//
// 握手序列（TCP 连接成功后）：
//   1. 发 "AT+SGDM=?\r\n"，等待响应行含 "(A01,..."（数据通道列表）
//   2. 发 "AT+SMPF=?\r\n"，等待响应（拒绝 "ERROR"）
//   3. 发 "AT+GSD\r\n" 启动推流，此后 onReadyRead 走帧解析
// 握手失败 → fault() + 断连 + 指数退避重连（0.5s→…→5s 封顶）。
class SriDriver : public QObject
{
    Q_OBJECT
public:
    explicit SriDriver(QObject *parent = nullptr);
    ~SriDriver() override;

    void configure(const ForceSensorConfig &cfg);

    bool isRunning() const;

    // Thread-safe: latest wrench snapshot
    WrenchFrame latest() const;
    bool isConnected() const;
    int staleCount() const;

    // 通信线程按 RSI 周期调用：把窗口均值取走写入 out。
    // 有帧且已连接 → out.fresh=true（六通道均值），stale 计数清零；
    // 否则 out.fresh=false、stale 计数 +1（掉线/超时/首帧前）。
    // 每次调用都会清空累加器。非槽函数——由 RsiWorker 直接调用。
    void drainAccumulator(WrenchFrame &out);

public slots:
    void start();
    void stop();

signals:
    void connected();
    void disconnected(const QString &reason);
    void fault(const QString &reason);

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void onReconnect();

private:
    // 白盒单测（tests/test_sri_driver.cpp）：不碰 TCP，直接驱动
    // processFrames()/m_connected 验证协议集成行为。
    friend class TestSriDriver;

    enum HandshakeState { HsInit, HsSentFormat, HsSentRate, HsStreaming };

    void connectToSensor();
    void closeSocket();
    void scheduleReconnect();
    void processFrames(const std::vector<SriFrame> &frames);
    // AT 握手响应处理：逐字节积累到 m_hsBuf，凑满一行（'\n' 或缓冲满）即校验
    // 并推进状态机；失败则 fault + 断连 + 排程重连。返回 true 表示已消费一行。
    bool handleAtResponse(const uint8_t *data, size_t len);

    ForceSensorConfig m_cfg;
    QTcpSocket *m_sock = nullptr;
    QTimer    *m_reconnectTimer = nullptr;
    SriFrameParser m_parser;

    // Window-mean accumulator for anti-aliasing (downsample 2kHz → 250Hz)
    double m_accForce[3]  = {0.0, 0.0, 0.0};
    double m_accTorque[3] = {0.0, 0.0, 0.0};
    int    m_accCount      = 0;

    HandshakeState m_hsState = HsInit;
    char m_hsBuf[128];   // AT 握手响应行缓冲（固定大小，实时路径零堆分配）
    int  m_hsLen = 0;

    mutable QMutex m_mutex;
    WrenchFrame m_latest;
    bool m_connected = false;
    int  m_staleCounter = 0;
    double m_reconnectDelayS = 0.5;
    bool m_running = false;
};
