#include "net/RsiWorker.h"

#include <QNetworkDatagram>
#include <algorithm>
#include <cmath>
#include "core/RsiCodec.h"

RsiWorker::RsiWorker(const AppConfig &cfg, SharedState *state,
                     SampleRing *ring, QObject *parent)
    : QObject(parent), m_cfg(cfg), m_state(state), m_ring(ring)
{
    m_ctl.configure(cfg);
}

void RsiWorker::start()
{
    if (m_sock)
        return;

    m_sock = new QUdpSocket(this);
    const QHostAddress addr(m_cfg.listenIp);
    if (!m_sock->bind(addr, m_cfg.listenPort)) {
        const QString why = QStringLiteral("bind %1:%2 failed: %3")
                                .arg(m_cfg.listenIp)
                                .arg(m_cfg.listenPort)
                                .arg(m_sock->errorString());
        delete m_sock;
        m_sock = nullptr;
        emit bindFailed(why);
        return;
    }
    connect(m_sock, &QUdpSocket::readyRead,
            this, &RsiWorker::onDatagram);

    m_watchdog = new QTimer(this);
    // 看门狗周期取通信周期的 20 倍，最少 200ms
    m_watchdog->setInterval(
        std::max(200, int(m_cfg.cycleMs * 20.0)));
    connect(m_watchdog, &QTimer::timeout,
            this, &RsiWorker::onWatchdog);
    m_watchdog->start();

    m_sessionTimer.start();
    m_haveFirstFrame  = false;
    m_frameCount      = 0;
    m_missed          = 0;
    m_maxReplyUs      = 0.0;
    m_cycleTimerValid = false;

    emit listening();
}

void RsiWorker::stop()
{
    if (m_watchdog) {
        m_watchdog->stop();
        m_watchdog->deleteLater();
        m_watchdog = nullptr;
    }
    if (m_sock) {
        m_sock->close();
        m_sock->deleteLater();
        m_sock = nullptr;
    }
    m_haveFirstFrame = false;
    StatusSnapshot s;
    s.connected = false;
    m_state->publish(s);
}

void RsiWorker::applyTarget(Pose t)      { m_ctl.setTarget(t); }
void RsiWorker::setTracking(bool on)     { m_ctl.setTracking(on); }

void RsiWorker::applyConfig(AppConfig cfg)
{
    m_cfg = cfg;
    m_ctl.configure(cfg);
}

void RsiWorker::resetToActual()
{
    m_ctl.resetToActual(m_state->snapshot().actual);
}

void RsiWorker::onWatchdog()
{
    // 长时间无包：视为 RSI 已停止，退回未连接
    if (!m_haveFirstFrame)
        return;
    m_haveFirstFrame = false;
    m_ring->clear();
    StatusSnapshot s = m_state->snapshot();
    s.connected = false;
    m_state->publish(s);
}

void RsiWorker::onDatagram()
{
    while (m_sock && m_sock->hasPendingDatagrams()) {
        QElapsedTimer replyTimer;
        replyTimer.start();

        const QNetworkDatagram dg = m_sock->receiveDatagram();
        m_peerAddr = dg.senderAddress();
        m_peerPort = quint16(dg.senderPort());

        const RobFrame f = RsiCodec::parseRob(dg.data());

        // ── 无论解析成败，都必须回包 ──
        quint64 echoIpoc = f.valid ? f.ipoc : m_lastIpoc;
        Pose    delta;   // 默认零增量

        if (f.valid) {
            if (m_haveFirstFrame) {
                // IPOC 应单调递增；否则计一次丢包
                if (f.ipoc <= m_lastIpoc)
                    ++m_missed;
            } else {
                // 首帧 = RSI 会话开始：目标置为实际且累积清零，机器人原地不动。
                // 必须用 beginSession 而非 resetToActual——后者刻意保留累积量，
                // 因为 RELATIVE 模式下 KRC 侧已施加的修正不会因主机归零而消失。
                m_ctl.beginSession(f.rist);
                m_haveFirstFrame = true;
                emit firstFrameReceived();
            }

            if (m_cycleTimerValid) {
                m_measuredCycleMs = m_cycleTimer.nsecsElapsed() / 1.0e6;
            }
            m_cycleTimer.start();
            m_cycleTimerValid = true;

            m_lastIpoc = f.ipoc;
            ++m_frameCount;

            delta = m_ctl.step(f.rist);
        } else {
            ++m_missed;
        }

        const QByteArray sen =
            RsiCodec::buildSen(delta, echoIpoc, m_cfg.senType);
        m_sock->writeDatagram(sen, m_peerAddr, m_peerPort);

        const double replyUs = replyTimer.nsecsElapsed() / 1000.0;
        m_maxReplyUs = std::max(m_maxReplyUs, replyUs);

        if (m_missed >= m_cfg.watchdogMissLimit &&
            m_ctl.state() == TrackState::Tracking) {
            m_ctl.setTracking(false);
        }

        if (f.valid) {
            const Pose err = poseSub(m_ctl.target(), f.rist);
            publishSnapshot(f.rist, err, f.ipoc, true);

            ChartSample cs;
            cs.tSec = m_sessionTimer.nsecsElapsed() / 1.0e9;
            cs.posErrNorm = std::sqrt(err.x * err.x + err.y * err.y +
                                      err.z * err.z);
            cs.rotErrNorm = std::max({std::fabs(err.a), std::fabs(err.b),
                                      std::fabs(err.c)});
            m_ring->push(cs);
        }

        m_watchdog->start();   // 收到包就重置看门狗
    }
}

void RsiWorker::publishSnapshot(const Pose &actual, const Pose &err,
                                quint64 ipoc, bool connected)
{
    StatusSnapshot s;
    s.actual          = actual;
    s.target          = m_ctl.target();
    s.error           = err;
    s.accum           = m_ctl.accumulated();
    s.ipoc            = ipoc;
    s.state           = m_ctl.state();
    s.faultReason     = m_ctl.faultReason();
    s.missedCount     = m_missed;
    s.measuredCycleMs = m_measuredCycleMs;
    s.maxReplyUs      = m_maxReplyUs;
    s.frameCount      = m_frameCount;
    s.connected       = connected;
    m_state->publish(s);
}
