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
    m_lastIpoc        = 0;
    m_measuredCycleMs = 0.0;
    // 注意：m_sinceLastFrame 在 start() 和 stop() 里都刻意不动——它必须跨越
    // 一次 teardown 存活，下个 start() 才能分辨"真正的会话重启"与"快速的
    // stop()→start()"。进程启动后的首个 start() 时它从未 start 过，isValid()
    // 为假，首帧走 beginSession()；此后的 start() 仍持有上一帧的时间戳，
    // elapsed() 很小，首帧走 resetToActual()，保住 KRC 侧已施加的修正。
    // 在这里 invalidate() 会让 isValid() 恒假，判据形同虚设。

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
    // 用通信线程自己缓存的最近一帧实际位姿，避免为了取 6 个 double 而拷贝
    // 整个 StatusSnapshot（内含 QString），也避免在尚无发布时退化成原点。
    m_ctl.resetToActual(m_lastActual);
}

void RsiWorker::onWatchdog()
{
    if (!m_haveFirstFrame)
        return;
    if (!m_watchdog)
        return;
    // 用流逝时间判定静默，而不是每帧重启定时器：QTimer::start() 在已激活的
    // 定时器上会 delete/new 一个 WinTimerInfo 并做一对 KillTimer/SetTimer
    // 系统调用，那是每周期一次的堆分配，违反"实时路径无动态内存分配"。
    if (m_sinceLastFrame.isValid()
        && m_sinceLastFrame.elapsed() < m_watchdog->interval())
        return;

    m_haveFirstFrame  = false;
    m_cycleTimerValid = false;   // 否则下个会话的首帧会把整段静默当作周期发布
    m_missed          = 0;       // 丢包计数是突发保护，不是终身计数
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
        // codec 独立解析 IPOC，并在其不可信时留在默认值 0（见 RsiCodec 里的
        // hasError 守卫），所以 0 是可靠哨兵。只要 IPOC 本身解析成功就必须
        // 原样回显——哪怕 RIst 损坏导致整帧 invalid。回一个陈旧 IPOC 等同
        // 丢包，而那正是"任何分支都必须回包"这条约束要避免的后果。
        const quint64 echoIpoc = f.ipoc ? f.ipoc : m_lastIpoc;
        Pose    delta;   // 默认零增量
        bool    wasFirstFrame = false;

        if (f.valid) {
            bool ipocOk = true;
            if (m_haveFirstFrame) {
                // IPOC 应单调递增；否则计一次丢包
                if (f.ipoc <= m_lastIpoc) {
                    ++m_missed;
                    ipocOk = false;
                }
            } else {
                // 只有确实静默过至少一个会话间隔，才算真正的 RSI 会话重启，
                // 才可以清零累积量。快速的 stop()→start() 不算：KRC 侧已施加
                // 的修正仍然存在，清零等于凭空多发一份预算，反复几次就能把
                // 总修正推过 POSCORR 的 ~50mm 硬限，而界面上第 2 层始终显示
                // 一个很小的累积值。
                // 用独立的会话间隔阈值，而不是看门狗间隔。看门狗只负责
                // "连接丢失"的显示，阈值必须小；会话判定则必须大于 KRC 的
                // Timeout，否则会在 KRC 仍认为会话连续时移动安全锚点。
                const bool genuineSessionStart =
                    !m_sinceLastFrame.isValid()
                    || m_sinceLastFrame.elapsed() >= qint64(m_cfg.sessionGapMs);
                if (genuineSessionStart)
                    m_ctl.beginSession(f.rist);
                else
                    m_ctl.resetToActual(f.rist);
                m_haveFirstFrame = true;
                wasFirstFrame    = true;
            }

            if (m_cycleTimerValid) {
                m_measuredCycleMs = m_cycleTimer.nsecsElapsed() / 1.0e6;
            }
            m_cycleTimer.start();
            m_cycleTimerValid = true;

            m_lastIpoc = f.ipoc;
            m_sinceLastFrame.restart();
            m_lastActual = f.rist;
            ++m_frameCount;

            if (ipocOk)
                m_missed = 0;       // 连续丢包计数：只有正常帧才清零

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

            // 必须在 publishSnapshot 之后才发这个信号：GUI 的处理器会读
            // snapshot() 来同步目标位姿，若先发信号它读到的还是本帧之前的
            // 快照（首帧时即全零）。今天只是显示错，但一旦「使能跟踪」
            // 依赖这个目标值，就会变成带着错误目标启动跟踪。
            if (wasFirstFrame)
                emit firstFrameReceived();
        }
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
