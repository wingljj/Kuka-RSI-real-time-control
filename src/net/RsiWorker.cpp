#include "net/RsiWorker.h"

#include <QNetworkDatagram>
#include <QVariant>
#include <algorithm>
#include <array>
#include <cmath>
#include "core/PoseOps.h"
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
    // 突发积压时内核缓冲兜底。Windows 上 SO_RCVBUF 是软上限，由 OS 决定实际值。
    m_sock->setSocketOption(
        QAbstractSocket::ReceiveBufferSizeSocketOption, m_cfg.rxBufferBytes);

    m_watchdog = new QTimer(this);
    // 看门狗周期取通信周期的 20 倍，最少 200ms
    m_watchdog->setInterval(
        std::max(200, int(m_cfg.cycleMs * 20.0)));
    connect(m_watchdog, &QTimer::timeout,
            this, &RsiWorker::onWatchdog);
    m_watchdog->start();

    m_sessionTimer.start();
    m_ipocTracker.reset();
    m_frameCount      = 0;
    m_missed          = 0;
    m_maxReplyUs      = 0.0;
    m_cycleTimerValid = false;
    m_measuredCycleMs = 0.0;
    m_sinceLastStep.invalidate();   // 新会话：没有"上一次 step"，预算基准不可信
    m_lifetimeLost = 0;
    m_lastDelta    = Pose{};
    m_cycleHead    = 0;
    m_cycleCount   = 0;
    m_havePrevPose = false;   // 会话重启：下一帧无可比帧，不检查跳变
    m_staleCount   = 0;
    // 注意：m_sinceLastFrame 在 start() 和 stop() 里都刻意不动——它必须跨越
    // 一次 teardown 存活，下个 start() 才能分辨"真正的会话重启"与"快速的
    // stop()→start()"。进程启动后的首个 start() 时它从未 start 过，isValid()
    // 为假，首帧走 beginSession()；此后的 start() 仍持有上一帧的时间戳，
    // elapsed() 很小，首帧不换锚点、不清账本，保住 KRC 侧已施加的修正。
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
    m_ipocTracker.reset();
    m_peerLocked   = false;
    m_peerRejected = 0;
    m_sendFails    = 0;
    m_lastDelay    = 0;
    m_delayRising  = 0;
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
    if (!m_ipocTracker.haveFirst())
        return;
    if (!m_watchdog)
        return;
    // 用流逝时间判定静默，而不是每帧重启定时器：QTimer::start() 在已激活的
    // 定时器上会 delete/new 一个 WinTimerInfo 并做一对 KillTimer/SetTimer
    // 系统调用，那是每周期一次的堆分配，违反"实时路径无动态内存分配"。
    if (m_sinceLastFrame.isValid()
        && m_sinceLastFrame.elapsed() < m_watchdog->interval())
        return;

    m_ipocTracker.reset();
    m_cycleTimerValid = false;   // 否则下个会话的首帧会把整段静默当作周期发布
    // 步长预算的时间基准同样作废。跨越静默的那个间隔不是"这一帧欠了多少
    // 修正"——静默期间主机根本没在控制，把它当预算发出去等于凭空补发一段
    // 从未发生的跟踪。基准不可信时 step() 只能拿到 0 预算（见 onDatagram）。
    m_sinceLastStep.invalidate();
    m_missed          = 0;       // 丢包计数是突发保护，不是终身计数
    m_ring->clear();
    StatusSnapshot s = m_state->snapshot();
    s.connected = false;
    m_state->publish(s);
}

void RsiWorker::onDatagram()
{
    int processed = 0;
    while (m_sock && m_sock->hasPendingDatagrams() && processed < kMaxBurst) {
        ++processed;
        QElapsedTimer replyTimer;
        replyTimer.start();

        const QNetworkDatagram dg = m_sock->receiveDatagram();
        if (!m_peerLocked) {
            // 首帧锁定对端。此后只认同一 (IP, port) 的帧。
            m_peerAddr   = dg.senderAddress();
            m_peerPort   = quint16(dg.senderPort());
            m_peerLocked = true;
        } else if (dg.senderAddress() != m_peerAddr
                   || quint16(dg.senderPort()) != m_peerPort) {
            // 异源帧：假 KRC（残留模拟器之类）。丢弃且不回包——回包会让它
            // 误以为掌控链路；真实 KRC 的源固定，不受影响。
            ++m_peerRejected;
            continue;
        }

        const RobFrame f = RsiCodec::parseRob(dg.data());

        // ── 无论解析成败，都必须回包 ──
        // codec 独立解析 IPOC，并在其不可信时留在默认值 0（见 RsiCodec 里的
        // hasError 守卫），所以 0 是可靠哨兵。只要 IPOC 本身解析成功就必须
        // 原样回显——哪怕 RIst 损坏导致整帧 invalid。回一个陈旧 IPOC 等同
        // 丢包，而那正是"任何分支都必须回包"这条约束要避免的后果。
        const quint64 echoIpoc = f.ipoc ? f.ipoc : m_ipocTracker.lastGood();
        Pose    delta;   // 默认零增量
        bool    wasFirstFrame   = false;   // 本帧是 IPOC 序列的首帧（含间隙恢复）
        bool    wasSessionStart = false;   // 且是真正的会话重启（新账本、新锚点）

        if (f.valid) {
            const IpocEvent ev = m_ipocTracker.classify(f.ipoc);

            if (ev.kind == IpocEvent::First) {
                // 只有确实静默过至少一个会话间隔，才算真正的 RSI 会话重启，
                // 才可以清零累积量。快速的 stop()→start() 不算：KRC 侧已施加
                // 的修正仍然存在，清零等于凭空多发一份预算，反复几次就能把
                // 总修正推过 POSCORR 的 ~50mm 硬限。
                // 用独立的会话间隔阈值，而不是看门狗间隔。看门狗只负责
                // "连接丢失"的显示，阈值必须小；会话判定则必须大于 KRC 的
                // Timeout，否则会在 KRC 仍认为会话连续时移动安全锚点。
                const bool genuineSessionStart =
                    !m_sinceLastFrame.isValid()
                    || m_sinceLastFrame.elapsed() >= qint64(m_cfg.sessionGapMs);
                if (genuineSessionStart) {
                    m_ctl.beginSession(f.rist);
                    // 真正的会话重启：首帧不与重启前（看门狗间隙之前）的旧帧
                    // 比较跳变。start() 里那句"会话重启：下一帧无可比帧"的注释
                    // 对跨间隙的首帧同样适用——不在这里清掉，第一帧就会拿
                    // 间隙前的位姿当锚点，物理跳变判定可能误杀会话恢复帧。
                    m_havePrevPose = false;
                }
                // ── else：看门狗间隙恢复。刻意什么都不做。────────────────
                // 这里曾经调用 m_ctl.resetToActual(f.rist)，那是一个真实缺陷：
                // resetToActual 的设计用途是"界面上手动归零"，它会把目标改写
                // 成当前实际、把状态打回 Idle、并重置目标轨迹。于是一次
                // 240ms（= 看门狗间隔）的网络抖动就足以让一段正在运行的跟踪
                // 无声停止、实时误差归零，而操作员设定的目标被丢弃——现场
                // 报告的"跟到一半突然停了"正是这三件事。
                //
                // 恢复通信不是操作员的意图表达，不该改变操作员的意图。间隙
                // 恢复需要做的只有"重建 IPOC 基准"，而这一件事 classify() 自己
                // 已经做完了（它在 First 分支里锁存了新的 lastGood）。所以这
                // 一支正确的实现就是空的。
                //
                // 为什么保留跟踪状态是安全的：step() 的第 1 层限值
                // （vmax × cycle，默认 0.6mm/周期）是绝对的单周期上限，控制律
                // 里没有积分项、不存在饱和积分，所以间隙期间误差涨到多大，
                // 恢复后第一帧发出的增量仍然只有 0.6mm——与操作员在界面上
                // 直接改一个远目标完全同量级，不是新引入的风险。现在这个上限
                // 还要再乘以"实测帧间隔 / 配置周期"（∈[0,1]），只会更小。
                //
                // 为什么这里刻意 *不* 清 m_havePrevPose（与上面的会话重启相反）：
                // KRC 没有重启，间隙前后是同一段连续运动，两帧之间可比。留着
                // 它，下面的 exceedsPhysicalJump 就会拿间隙前的位姿做一次跳变
                // 检查——它用的是一个周期的预算（12ms → 6mm），对一段几百
                // 毫秒的间隙而言过严，但过严的方向是安全的：至多把恢复首帧
                // 判为 stale、本周期回零增量，下一帧即自愈（m_staleCount 会被
                // 清零，够不到 staleFrameLimit）。这等于白得一次"间隙期间机器
                // 人是否被挪动过"的检查。
                //
                // 为什么真正的会话重启仍必须走 beginSession 清账本：那时 KRL
                // 程序重跑过，KRC 侧的 POSCORR 累积量确实已经回零，主机的安全
                // 锚点必须跟着移到新的 RIst₀，否则主机账本与 KRC 层 4/5 的限值
                // 不再共享原点。而间隙恢复时 KRC 从未重启、它的累积量原封不动，
                // 此时移锚点等于在已施加的修正之上再发一份全新预算。
                wasFirstFrame   = true;
                wasSessionStart = genuineSessionStart;

                // 步长预算的时间基准重开在这一帧：本帧不 step（First 不在
                // Normal/Gap 之列），预算从"恢复这一刻"起算。
                //
                // 【这一句到不了积压排空，别把它当那道防线】主机侧停顿期间
                // KRC 的 IPOC 流是连续的，IpocTracker 从未被 reset，所以恢复后
                // 被排空的那几十帧一律分类为 Normal，根本进不了 First 分支。
                // 变异实测（删掉本行）：排空总增量 0.7103mm，与基线 0.7255~
                // 0.7548mm 统计上不可分辨，14/14 用例全绿。排空真正的防线是
                // 下面 Normal 帧那段按墙钟发放 + 封顶的预算（见 step()）。
                //
                // 【它真正的作用范围】IpocEvent::First 只可能出现在
                // IpocTracker::reset() 之后，而生产里只有 onWatchdog() 与
                // start() 会 reset——也就是说本分支仅在"看门狗抢在 readyRead
                // 之前赢得竞争"时才走到，典型情形是 KRC 侧停发（无积压）。
                //
                // 【方向也要说清，别看反】那两个 reset 点都已经先
                // invalidate() 过基准了，所以本行并不是在"挡住一段跨间隙的
                // 陈旧间隔"——那件事已经由 onWatchdog() 做完，没有它也漏不
                // 出去。本行实际做的是反向的一件事：把恢复后第一个 Normal 帧
                // 从"基准 invalid → 0 预算 → 静默不发一帧"救回成"领到它应得的
                // 一个周期预算"。变异实测这一帧由 0.6000mm 变 0.0000mm
                // （watchdogGapRecovery_restartsStepBudgetBasis）。这是正确的：
                // 那一帧确实是恢复后整整一个周期，本就该值一个周期的运动，
                // 而且它照样受同一个封顶约束，不可能超过一帧的额度。
                //
                // 保留它还有纵深防御的意义：若日后有人新增一条 reset
                // IpocTracker 却忘了 invalidate 基准的路径，本行是那条路径上
                // 唯一还会把跨间隙间隔排除掉的地方。
                m_sinceLastStep.start();
            }

            if (m_cycleTimerValid) {
                m_measuredCycleMs = m_cycleTimer.nsecsElapsed() / 1.0e6;
                m_cycleHist[m_cycleHead] = m_measuredCycleMs;
                m_cycleHead = (m_cycleHead + 1) % kCycleHist;
                if (m_cycleCount < kCycleHist)
                    ++m_cycleCount;
            }
            m_cycleTimer.start();
            m_cycleTimerValid = true;

            m_sinceLastFrame.restart();
            m_lastActual = f.rist;
            ++m_frameCount;

            // 丢包计数：仅 Normal 清零；Gap 加缺口；Dup/Back 各 +1。
            // Gap 帧带全新 RIst（非旧位姿重放），正常 step；Dup/Back 是旧数据
            // 重放，只回零增量且不推进 lastGood（IpocTracker 已保证），否则会
            // 用一帧旧位姿再产生一次修正。
            switch (ev.kind) {
            case IpocEvent::Normal:
                m_missed = 0;
                break;
            case IpocEvent::Gap:
                m_missed += int(ev.gapCount);
                m_lifetimeLost += quint64(ev.gapCount);
                break;
            case IpocEvent::Duplicate:
            case IpocEvent::Backward:
                ++m_missed;
                ++m_lifetimeLost;
                break;
            case IpocEvent::First:
                break;
            }

            // 反馈异常剔除：单帧跳变超物理极限 → 本周期回零增量 + stale 计数；
            // 连续超限 → Fault。首帧/会话重启首帧无可比帧，不检查。
            bool stale = false;
            if (m_havePrevPose
                && poseops::exceedsPhysicalJump(
                    m_prevValidPose, f.rist, m_cfg.cycleMs / 1000.0,
                    m_cfg.physVmaxPosMmS, m_cfg.physVmaxRotDegS)) {
                stale = true;
                if (++m_staleCount >= m_cfg.staleFrameLimit
                    && m_ctl.state() == TrackState::Tracking) {
                    m_ctl.forceFault(QStringLiteral(
                        "feedback stale frames (jump beyond physical limit)"));
                }
            } else {
                m_staleCount = 0;
            }
            m_prevValidPose = f.rist;
            m_havePrevPose  = true;

            // KRC Delay 运行中保护：SENTYPE 错配、回复迟到/被丢弃都会让 KRC
            // 自己的 Delay 计数增长，而主机侧的丢包计数看不见这些。连续 3 帧
            // 递增（持平不算）即转 Fault。
            if (f.delay > m_lastDelay) {
                ++m_delayRising;
                if (m_delayRising >= 3 && m_ctl.state() == TrackState::Tracking)
                    m_ctl.forceFault(QStringLiteral(
                        "KRC reports rising delay %1 (lost/late replies)")
                            .arg(f.delay));
            } else {
                m_delayRising = 0;
            }
            m_lastDelay = f.delay;

            if (!stale
                && (ev.kind == IpocEvent::Normal || ev.kind == IpocEvent::Gap)) {
                // 步长预算按"距上一次 step 的实测墙钟"发放，而不是按配置周期。
                // 基准不可信（看门狗刚清过，且本帧居然不是 First）时只能给 0：
                // 一段无从核实的静默不该换来一份满预算。
                const double sinceStepMs =
                    m_sinceLastStep.isValid()
                        ? m_sinceLastStep.nsecsElapsed() / 1.0e6
                        : 0.0;
                delta = m_ctl.step(f.rist, sinceStepMs);
                m_sinceLastStep.start();
            }
        } else {
            ++m_missed;
            ++m_lifetimeLost;
        }

        m_lastDelta = delta;

        const QByteArray sen =
            RsiCodec::buildSen(delta, echoIpoc, m_cfg.senType);
        // 发送失败必须计数：KRC 收不到修正时继续跟踪是危险的。连续 5 次失败
        // 即 Fault；一次成功清零连续计数。
        const qint64 sent = m_sock->writeDatagram(sen, m_peerAddr, m_peerPort);
        if (sent < 0) {
            ++m_sendFails;
            if (m_sendFails >= 5 && m_ctl.state() == TrackState::Tracking)
                m_ctl.forceFault(
                    QStringLiteral("send failed %1 times").arg(m_sendFails));
        } else {
            m_sendFails = 0;
        }

        const double replyUs = replyTimer.nsecsElapsed() / 1000.0;
        m_maxReplyUs = std::max(m_maxReplyUs, replyUs);

        if (m_missed >= m_cfg.watchdogMissLimit &&
            m_ctl.state() == TrackState::Tracking) {
            m_ctl.setTracking(false);
        }

        if (f.valid) {
            // 姿态误差 = SO(3) 旋转向量（世界坐标，度）——与控制器一致，奇异/边界不跳变
            const Pose err = poseops::errorPoseDeg(m_ctl.target(), f.rist);
            publishSnapshot(f.rist, err, f.ipoc, true, wasFirstFrame);

            ChartSample cs;
            cs.tSec = m_sessionTimer.nsecsElapsed() / 1.0e9;
            cs.posErrNorm = std::sqrt(err.x * err.x + err.y * err.y +
                                      err.z * err.z);
            // 姿态误差现在是旋转向量（世界坐标，度），范数 = 总旋转角
            cs.rotErrNorm = std::sqrt(err.a * err.a + err.b * err.b + err.c * err.c);
            m_ring->push(cs);

            // 必须在 publishSnapshot 之后才发这个信号：GUI 的处理器会读
            // snapshot() 来同步目标位姿，若先发信号它读到的还是本帧之前的
            // 快照（首帧时即全零）。今天只是显示错，但一旦「使能跟踪」
            // 依赖这个目标值，就会变成带着错误目标启动跟踪。
            //
            // 只在真正的会话重启时发，看门狗间隙恢复时不发。这个信号的语义
            // 是"控制器的目标刚被重置成实际位姿了，界面请跟着同步"——只有
            // beginSession 那一支真的做了这件事。间隙恢复不改目标（见上面
            // 那段），此时若照发，GUI 会把目标输入框改写成当前实际位姿并
            // saveTargetSnapshot()，于是界面显示的目标与控制器持有的目标从此
            // 不一致，而「应用目标」按钮还是灰的——操作员看不到自己的目标
            // 已经在界面上被悄悄换掉。控制器保住了目标而界面丢了，比两边
            // 一起丢更难排查。
            if (wasSessionStart)
                emit firstFrameReceived();
        }
    }
}

void RsiWorker::publishSnapshot(const Pose &actual, const Pose &err,
                                quint64 ipoc, bool connected,
                                bool wasFirstFrame)
{
    // 7 态会话/控制状态，优先级：Fault > StaleFrame > Syncing(首帧瞬间) >
    // Tracking > Ready > WaitingFirstFrame > Disconnected。
    // Syncing 只出现在首帧（会话/恢复的同步瞬间），下一帧立即转 Ready/Tracking。
    ControlState cs;
    if (m_ctl.state() == TrackState::Fault) {
        cs = ControlState::Fault;
    } else if (m_staleCount > 0) {
        cs = ControlState::StaleFrame;
    } else if (wasFirstFrame) {
        cs = ControlState::Syncing;
    } else if (m_ctl.state() == TrackState::Tracking) {
        cs = ControlState::Tracking;
    } else if (m_ipocTracker.haveFirst()) {
        cs = ControlState::Ready;
    } else if (m_peerLocked) {
        cs = ControlState::WaitingFirstFrame;
    } else {
        cs = ControlState::Disconnected;
    }

    StatusSnapshot s;
    s.actual          = actual;
    s.target          = m_ctl.target();
    s.error           = err;
    s.accum           = m_ctl.accumulated();
    s.ipoc            = ipoc;
    s.state           = cs;
    s.faultReason     = m_ctl.faultReason();
    s.missedCount     = m_missed;
    s.measuredCycleMs = m_measuredCycleMs;
    s.maxReplyUs      = m_maxReplyUs;
    s.krcDelay        = m_lastDelay;
    s.peerRejected    = m_peerRejected;
    s.sendFails       = m_sendFails;
    s.frameCount      = m_frameCount;
    s.connected       = connected;
    s.peerIp4      = m_peerLocked ? m_peerAddr.toIPv4Address() : 0;
    s.peerPort     = m_peerLocked ? m_peerPort : 0;
    s.lifetimeLost = m_lifetimeLost;
    s.lastDelta    = m_lastDelta;

    // ── 跟踪质量：误差与累积修正相对限值的比例 ──
    if (cs == ControlState::Tracking) {
        const double errPos = std::sqrt(err.x*err.x + err.y*err.y + err.z*err.z);
        const double errRot = std::sqrt(err.a*err.a + err.b*err.b + err.c*err.c);
        const double accPos = std::max({std::fabs(s.accum.x),
                                        std::fabs(s.accum.y),
                                        std::fabs(s.accum.z)});
        const double accRot = std::max({std::fabs(s.accum.a),
                                        std::fabs(s.accum.b),
                                        std::fabs(s.accum.c)});
        s.accumPosPct = (m_cfg.accumLimitPosMm > 0.0)
                            ? accPos / m_cfg.accumLimitPosMm : 0.0;
        s.accumRotPct = (m_cfg.accumLimitRotDeg > 0.0)
                            ? accRot / m_cfg.accumLimitRotDeg : 0.0;
        s.errorPosPct = (m_cfg.accumLimitPosMm > 0.0)
                            ? errPos / m_cfg.accumLimitPosMm : 0.0;
        s.errorRotPct = (m_cfg.accumLimitRotDeg > 0.0)
                            ? errRot / m_cfg.accumLimitRotDeg : 0.0;
        s.accumOverLimit = (s.accumPosPct >= 1.0) || (s.accumRotPct >= 1.0);

        const double worstPct = std::max({s.accumPosPct, s.accumRotPct,
                                          s.errorPosPct, s.errorRotPct});
        if (worstPct >= 1.0)
            s.trackingQuality = TrackingQuality::OverLimit;
        else if (worstPct >= m_cfg.trackingQualityCriticalPct)
            s.trackingQuality = TrackingQuality::NearLimit;
        else if (worstPct >= m_cfg.trackingQualityWarnPct)
            s.trackingQuality = TrackingQuality::LargeError;
        else
            s.trackingQuality = TrackingQuality::Normal;
    } else {
        s.trackingQuality = TrackingQuality::Inactive;
    }

    if (m_cycleCount > 0) {
        // 拷贝到定容临时数组：nth_element 原地改，不能碰历史。无堆分配。
        const int n = m_cycleCount;
        std::array<double, kCycleHist> h{};
        double sum = 0.0, mx = 0.0;
        for (int i = 0; i < n; ++i) {
            const double v =
                m_cycleHist[(m_cycleHead - n + i + kCycleHist) % kCycleHist];
            h[i] = v;
            sum += v;
            mx = std::max(mx, v);
        }
        s.cycleMeanMs = sum / n;
        s.cycleMaxMs  = mx;
        const int p   = std::max(0, (n * 99) / 100 - 1);
        std::nth_element(h.begin(), h.begin() + p, h.begin() + n);
        s.cycleP99Ms  = h[p];
    }
    m_state->publish(s);
}
