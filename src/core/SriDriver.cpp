#include "core/SriDriver.h"
#include <algorithm>
#include <cmath>
#include <cstring>

static const char kQueryFormat[] = "AT+SGDM=?\r\n";
static const char kQueryRate[]   = "AT+SMPF=?\r\n";
static const char kStartStream[] = "AT+GSD\r\n";
// SGDM=? 的期望响应形如 "(A01,A02,A03,A04,A05,A06);E;"。判定只要求通道列表以
// (A01, 开头——真实传感器可能给出不同的通道集合/顺序。
static const char kFormatMarker[] = "(A01,";

SriDriver::SriDriver(QObject *parent)
    : QObject(parent)
{
    m_reconnectTimer = new QTimer(this);
    m_reconnectTimer->setSingleShot(true);
    connect(m_reconnectTimer, &QTimer::timeout, this, &SriDriver::onReconnect);
}

SriDriver::~SriDriver()
{
    stop();
}

void SriDriver::configure(const ForceSensorConfig &cfg)
{
    m_cfg = cfg;
}

bool SriDriver::isRunning() const { return m_running; }

WrenchFrame SriDriver::latest() const
{
    QMutexLocker lock(&m_mutex);
    return m_latest;
}

bool SriDriver::isConnected() const
{
    QMutexLocker lock(&m_mutex);
    return m_connected;
}

int SriDriver::staleCount() const
{
    QMutexLocker lock(&m_mutex);
    return m_staleCounter;
}

void SriDriver::start()
{
    if (m_running) return;
    m_running = true;
    m_reconnectDelayS = 0.5;
    connectToSensor();
}

void SriDriver::stop()
{
    m_running = false;
    m_reconnectTimer->stop();
    closeSocket();
}

void SriDriver::connectToSensor()
{
    if (!m_running) return;
    closeSocket();

    m_sock = new QTcpSocket(this);
    connect(m_sock, &QTcpSocket::connected, this, &SriDriver::onConnected);
    connect(m_sock, &QTcpSocket::disconnected, this, &SriDriver::onDisconnected);
    connect(m_sock, &QTcpSocket::readyRead, this, &SriDriver::onReadyRead);

    // 复位解析器、握手状态与累加器，防止上次会话的残留数据污染新会话
    m_parser.reset();
    m_hsState = HsInit;
    m_hsLen = 0;
    m_accForce[0] = m_accForce[1] = m_accForce[2] = 0.0;
    m_accTorque[0] = m_accTorque[1] = m_accTorque[2] = 0.0;
    m_accCount = 0;

    m_sock->connectToHost(m_cfg.host, m_cfg.port);
}

void SriDriver::closeSocket()
{
    if (m_sock) {
        m_sock->disconnect();
        m_sock->deleteLater();
        m_sock = nullptr;
    }
    QMutexLocker lock(&m_mutex);
    m_connected = false;
    m_latest.fresh = false;
}

void SriDriver::scheduleReconnect()
{
    m_reconnectDelayS = std::min(m_reconnectDelayS * 2.0, 5.0);
    m_reconnectTimer->start(int(m_reconnectDelayS * 1000));
}

void SriDriver::onConnected()
{
    m_hsState = HsSentFormat;
    m_sock->write(kQueryFormat);
}

void SriDriver::onDisconnected()
{
    {
        QMutexLocker lock(&m_mutex);
        m_connected = false;
        m_latest.fresh = false;
    }

    emit disconnected(QStringLiteral("SRI TCP disconnected"));

    if (m_running)
        scheduleReconnect();
}

void SriDriver::onReconnect()
{
    if (!m_running) return;
    connectToSensor();
}

void SriDriver::onReadyRead()
{
    // 固定大小栈缓冲：实时路径零堆分配
    uint8_t buf[4096];
    while (m_sock && m_sock->bytesAvailable() > 0) {
        const qint64 n = m_sock->read(reinterpret_cast<char *>(buf), sizeof(buf));
        if (n <= 0) break;

        switch (m_hsState) {
        case HsStreaming:
            processFrames(m_parser.feed(buf, size_t(n)));
            break;
        case HsSentFormat:
        case HsSentRate:
            handleAtResponse(buf, size_t(n));
            break;
        case HsInit:
            break;  // 未请求任何查询前不应有响应（防御性忽略）
        }
    }
}

bool SriDriver::handleAtResponse(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        const char c = char(data[i]);
        if (c == '\n' || m_hsLen >= int(sizeof(m_hsBuf)) - 1) {
            m_hsBuf[m_hsLen] = '\0';
            const bool ok = (m_hsState == HsSentFormat)
                ? (std::strstr(m_hsBuf, kFormatMarker) != nullptr)
                : (std::strstr(m_hsBuf, "ERROR") == nullptr);
            m_hsLen = 0;

            if (!ok) {
                emit fault(QStringLiteral("SRI handshake rejected: %1")
                               .arg(QString::fromLatin1(m_hsBuf)));
                closeSocket();
                if (m_running)
                    scheduleReconnect();
                return true;
            }

            if (m_hsState == HsSentFormat) {
                m_sock->write(kQueryRate);
                m_hsState = HsSentRate;
            } else {
                m_sock->write(kStartStream);
                m_hsState = HsStreaming;
                {
                    QMutexLocker lock(&m_mutex);
                    m_connected = true;
                }
                m_reconnectDelayS = 0.5;
                emit connected();
            }
            return true;
        }
        m_hsBuf[m_hsLen++] = c;
    }
    return false;
}

void SriDriver::processFrames(const std::vector<SriFrame> &frames)
{
    for (const auto &f : frames) {
        // Apply channel signs + torque scale
        double sv[6];
        for (int i = 0; i < 6; ++i) sv[i] = double(f.values[i]);
        for (int i = 0; i < 6; ++i) sv[i] *= double(m_cfg.channelSigns[i]);
        sv[3] *= m_cfg.torqueScale;
        sv[4] *= m_cfg.torqueScale;
        sv[5] *= m_cfg.torqueScale;

        // Accumulate for window mean (anti-aliasing decimation)
        m_accForce[0]  += sv[0]; m_accForce[1]  += sv[1]; m_accForce[2]  += sv[2];
        m_accTorque[0] += sv[3]; m_accTorque[1] += sv[4]; m_accTorque[2] += sv[5];
        ++m_accCount;
    }
}

void SriDriver::drainAccumulator(WrenchFrame &out)
{
    QMutexLocker lock(&m_mutex);
    if (m_accCount > 0 && m_connected) {
        const double inv = 1.0 / double(m_accCount);
        out.fx = m_accForce[0] * inv;
        out.fy = m_accForce[1] * inv;
        out.fz = m_accForce[2] * inv;
        out.mx = m_accTorque[0] * inv;
        out.my = m_accTorque[1] * inv;
        out.mz = m_accTorque[2] * inv;
        out.fresh = true;
        m_latest = out;
        m_staleCounter = 0;
    } else {
        out.fresh = false;
        ++m_staleCounter;
    }
    m_accForce[0] = m_accForce[1] = m_accForce[2] = 0.0;
    m_accTorque[0] = m_accTorque[1] = m_accTorque[2] = 0.0;
    m_accCount = 0;
}
