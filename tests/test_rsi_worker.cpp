#include <QtTest>
#include <QUdpSocket>
#include <cmath>
#include "core/AppConfig.h"
#include "net/RsiWorker.h"
#include "net/SharedState.h"

// RsiWorker 的「IPOC 首帧该怎么处理」路由测试。
//
// 为什么这组测试必须走真实 UDP、而不是直接单测 PoseController：被测的不是
// PoseController 的某个方法，而是 RsiWorker 在三种静默时长下选了哪条路。那个
// 选择依赖 QElapsedTimer 的真实流逝与看门狗定时器的真实触发，抽成纯函数就把
// 要测的东西一起抽掉了。端口用 127.0.0.1 上的高位端口，每个用例各占一个，
// 避免同一 fixture 内 socket 尚未释放导致的 bind 冲突。

namespace {

constexpr double kActualX = 1000.0;   // 机器人「当前」X（mm）
constexpr double kActualZ = 1500.0;
constexpr double kTargetOffsetX = 5.0;   // 操作员设定的目标偏移（mm）

// 看门狗间隔 = max(200, cycleMs × 20)；12ms 周期下 = 240ms。
// 会话间隔刻意压到 1200ms，让「真会话重启」用例不必等满 2 秒。
AppConfig testCfg(quint16 port)
{
    AppConfig c = AppConfig::defaults();
    c.listenIp           = QStringLiteral("127.0.0.1");
    c.listenPort         = port;
    c.cycleMs            = 12.0;
    c.sessionGapMs       = 1200.0;
    c.targetTrajectoryMs = 0.0;   // 轨迹直通：目标一经设定即生效，断言无需等轨迹跑完
    return c;
}

Pose poseAt(double x)
{
    Pose p;
    p.x = x;
    p.z = kActualZ;
    return p;
}

// 从一份 SEN 回包里取出 RKorr 的 X（mm）。回包是主机真正发上线的东西，
// 所以它是"这次事件里机器人被要求走多远"的唯一可信度量——比读控制器的
// 内部账本更接近 KRC 看到的现实（含 4 位小数量化）。
double senRkorrX(const QByteArray &sen)
{
    const int k = sen.indexOf("<RKorr X=\"");
    if (k < 0)
        return 0.0;
    const int b = k + int(sizeof("<RKorr X=\"")) - 1;
    const int e = sen.indexOf('"', b);
    return e < 0 ? 0.0 : sen.mid(b, e - b).toDouble();
}

QByteArray robFrame(const Pose &p, quint64 ipoc)
{
    QByteArray s = "<Rob Type=\"KUKA\">\n<RIst";
    const char *k[6] = {" X=\"", " Y=\"", " Z=\"", " A=\"", " B=\"", " C=\""};
    const double v[6] = {p.x, p.y, p.z, p.a, p.b, p.c};
    for (int i = 0; i < 6; ++i) {
        s += k[i];
        s += QByteArray::number(v[i], 'f', 4);
        s += '"';
    }
    s += "/>\n<Delay D=\"0\"/>\n<IPOC>";
    s += QByteArray::number(ipoc);
    s += "</IPOC>\n</Rob>";
    return s;
}

} // namespace

class TestRsiWorker : public QObject
{
    Q_OBJECT

private:
    // 每个用例自己起 worker/sender，析构时归还端口。
    struct Rig
    {
        SharedState state;
        SampleRing  ring;
        AppConfig   cfg;
        RsiWorker  *worker = nullptr;
        QUdpSocket  sender;
        quint16     port = 0;

        explicit Rig(quint16 p) : cfg(testCfg(p)), port(p)
        {
            worker = new RsiWorker(cfg, &state, &ring);
            worker->start();
        }
        ~Rig()
        {
            worker->stop();
            delete worker;
        }

        // 发 n 帧，IPOC 从 startIpoc 连续递增。
        //
        // 每帧都等到 worker 真的处理完才发下一帧，而不是固定 qWait(N)。固定
        // 等待在本机偶尔够、在负载下就不够，于是「目标有没有被保住」这类断言
        // 会退化成「帧到了没有」的抛硬币——测试红了却与被测行为无关，比没有
        // 测试更糟。frameCount 只在有效帧上自增，是可靠的处理完成信号。
        void feed(const Pose &p, quint64 startIpoc, int n)
        {
            for (int i = 0; i < n; ++i) {
                const quint64 before = snap().frameCount;
                sender.writeDatagram(robFrame(p, startIpoc + quint64(i)),
                                     QHostAddress::LocalHost, port);
                QTRY_VERIFY(snap().frameCount > before);
            }
        }
        // 不跑事件循环地灌 n 帧：模拟"主机停顿期间 KRC 继续发包"，帧全部
        // 积压在内核接收缓冲里，等主机恢复后一次排空。用 feed() 造不出来——
        // 它每帧都等 worker 处理完，正是要绕开的那个节奏。
        void blast(const Pose &p, quint64 startIpoc, int n)
        {
            for (int i = 0; i < n; ++i)
                sender.writeDatagram(robFrame(p, startIpoc + quint64(i)),
                                     QHostAddress::LocalHost, port);
        }

        // 读走 sender 上积压的 SEN 回包，返回 (帧数, ΣRKorr.X)。
        QPair<int, double> drainReplies()
        {
            int n = 0;
            double sumX = 0.0;
            while (sender.hasPendingDatagrams()) {
                QByteArray buf(int(sender.pendingDatagramSize()), '\0');
                sender.readDatagram(buf.data(), buf.size());
                sumX += senRkorrX(buf);
                ++n;
            }
            return {n, sumX};
        }

        StatusSnapshot snap() const { return state.snapshot(); }
    };

private slots:
    // ── 路径 1：进程刚启动的首帧 ──
    // 判据 !m_sinceLastFrame.isValid()。应走 beginSession：目标同步为实际、
    // 状态 Idle、并锁存安全锚点 RIst₀。
    void processStartFirstFrame_beginsSession()
    {
        Rig r(59231);

        // 首帧之前先塞一个目标：beginSession 必须把它覆盖掉。生产里此刻还没有
        // 操作员目标，但正是「首帧是否重置目标」把本路径与看门狗恢复区分开。
        r.worker->applyTarget(poseAt(kActualX + 999.0));
        r.feed(poseAt(kActualX), 1000, 3);

        const StatusSnapshot s = r.snap();
        QVERIFY2(s.frameCount >= 3, "worker 没收到帧，端口或环境有问题");
        QCOMPARE(s.target.x, kActualX);              // 目标被同步为实际
        QVERIFY(s.state != ControlState::Tracking);  // 首帧不自行使能跟踪

        // 锚点确实锁存在首帧位姿上：使能跟踪后喂一帧偏 3mm 的实际位姿，
        // displacement = actual − anchor 应为 3。若 beginSession 没跑过，
        // m_haveAnchor 为假、displacement 恒为 0，本断言会失败。
        // 3mm 刻意小于物理跳变预算（500mm/s × 12ms = 6mm），不会被判 stale。
        r.worker->applyTarget(poseAt(kActualX + kTargetOffsetX));
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX + 3.0), 1003, 3);
        QVERIFY(std::fabs(r.snap().accum.x - 3.0) < 1e-6);
    }

    // ── 路径 2：看门狗间隙恢复（本次修复的缺陷）──
    // 静默 ≥ 看门狗间隔（240ms）但 < sessionGapMs（1200ms）。恢复首帧只该重建
    // IPOC 基准：操作员设定的目标必须原样保留，跟踪不得停。
    // ── 路径 2：Tracking 中链路静默超看门狗 → 可见 Fault(2026-08-06 P0-2)──
    // 旧语义是"间隙恢复无声继续跟踪"。那使升级策略非单调:100–200ms 断流
    // (恢复帧带大 gap)会 Fault,而更严重的 >200ms 断流反而无声恢复,机器人
    // 在无人确认下继续运动。现在:静默超看门狗且在跟踪 → forceFault,由看门狗
    // 立即发布(不等恢复帧);恢复帧不得悄悄回到 Tracking;操作员归零 + 重新
    // 使能后恢复正常。
    void watchdogSilence_whileTracking_faultsVisibly()
    {
        Rig r(59232);
        r.feed(poseAt(kActualX), 1000, 3);

        const Pose target = poseAt(kActualX + kTargetOffsetX);
        r.worker->applyTarget(target);
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1003, 3);
        QCOMPARE(r.snap().state, ControlState::Tracking);

        // 静默 600ms：看门狗（240ms）必然触发；远不到 1200ms 的会话间隔。
        QTest::qWait(600);
        const StatusSnapshot s = r.snap();
        QVERIFY2(!s.connected, "看门狗没触发，本用例没测到要测的路径");
        // Fault 必须在静默期间就可见,而不是等下一个有效帧才浮现
        QCOMPARE(s.state, ControlState::Fault);
        QVERIFY2(s.faultReason.contains(QStringLiteral("silent")),
                 qPrintable(s.faultReason));
        // 目标不被 Fault 抹掉(归零才会),便于操作员确认后继续
        QCOMPARE(s.target.x, target.x);

        // 恢复帧绝不悄悄恢复跟踪
        r.feed(poseAt(kActualX), 2000, 2);
        QCOMPARE(r.snap().state, ControlState::Fault);

        // 操作员路径:归零 → 重设目标 → 重新使能 → 恢复正常
        r.worker->resetToActual();
        r.worker->applyTarget(target);
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 2002, 2);
        QCOMPARE(r.snap().state, ControlState::Tracking);
    }

    // ── 路径 3：真正的会话重启 ──
    // 静默 ≥ sessionGapMs。KRL 重跑过、KRC 侧累积量已回零，主机必须换锚点、
    // 清账本、并把目标同步为新的实际位姿。这一支的行为刻意保持原样。
    void genuineSessionRestart_movesAnchorAndResetsTarget()
    {
        Rig r(59233);
        r.feed(poseAt(kActualX), 1000, 3);

        r.worker->applyTarget(poseAt(kActualX + kTargetOffsetX));
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1003, 3);
        QCOMPARE(r.snap().state, ControlState::Tracking);

        // 静默 1500ms > sessionGapMs(1200ms)
        QTest::qWait(1500);

        // 重启后机器人停在别处（KRL 重新 BCO）：新锚点必须是这个位姿。
        const double restartX = kActualX + 100.0;
        r.feed(poseAt(restartX), 1, 3);

        const StatusSnapshot s = r.snap();
        QCOMPARE(s.target.x, restartX);              // 目标同步为新实际
        QVERIFY(s.state != ControlState::Tracking);  // 重启后必须由操作员重新使能

        // 锚点已移到 restartX：使能跟踪后喂一帧偏 3mm 的位姿，displacement 应为 3。
        // 若锚点仍停在旧的 kActualX，这里会是 103——那正是「凭空多发一份预算」。
        r.worker->applyTarget(poseAt(restartX + kTargetOffsetX));
        r.worker->setTracking(true);
        r.feed(poseAt(restartX + 3.0), 4, 3);
        QVERIFY(std::fabs(r.snap().accum.x - 3.0) < 1e-6);
    }

    // ── 物理跳变剔除(不依赖看门狗场景)──
    // 跟踪中单帧位姿跳变超物理预算(physVmaxPosMmS 500 × 12ms = 6mm)必须被
    // 判为 stale:本周期回零增量、状态显示 StaleFrame。看门狗静默场景下跟踪
    // 已转 Fault(见上),本守卫的剩余价值在正常运行中的坏帧与 Ready 态。
    void physicalJump_rejectsFrameAndZerosDelta()
    {
        Rig r(59234);
        r.feed(poseAt(kActualX), 1000, 3);
        r.worker->applyTarget(poseAt(kActualX + kTargetOffsetX));
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1003, 3);
        QCOMPARE(r.snap().state, ControlState::Tracking);

        // 下一帧(IPOC 连续)位姿突跳 50mm ≫ 6mm 物理预算
        r.feed(poseAt(kActualX + 50.0), 1006, 1);
        QCOMPARE(r.snap().state, ControlState::StaleFrame);
        // 且本周期一定回零增量：反馈可疑时不许发修正。
        QCOMPARE(r.snap().lastDelta.x, 0.0);
    }

    // ── 无效帧路径上的 Fault 必须立即发布(2026-08-06 审查 P1-5)──
    // publishSnapshot 只在 f.valid 时调用;报文被网络设备持续截断成 parse-fail
    // 时,丢包计数照涨、forceFault 照锁,但快照永远不更新——GUI 冻结显示
    // "跟踪中"。持续洪流下内核缓冲总有积压,看门狗(hasPendingDatagrams 让路)
    // 也永远不发布。Fault 转移瞬间必须补发一次快照。
    void invalidFrameFlood_faultsVisiblyAtOnce()
    {
        Rig r(59239);
        r.feed(poseAt(kActualX), 1000, 3);
        r.worker->applyTarget(poseAt(kActualX + kTargetOffsetX));
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1003, 1);
        QCOMPARE(r.snap().state, ControlState::Tracking);
        r.drainReplies();          // 清掉此前 feed 的回包,只数洪流的

        // 30 帧垃圾报文(> miss limit 25),全部 parse-fail
        constexpr int kFlood = 30;
        for (int i = 0; i < kFlood; ++i)
            r.sender.writeDatagram(QByteArray("garbage-not-xml"),
                                   QHostAddress::LocalHost, r.port);
        // 等 worker 处理完(每帧必回包,以回包计数为同步信号),不给看门狗
        // (240ms)兜底的机会——断言的是"Fault 转移瞬间就发布"。
        int replies = 0;
        QElapsedTimer t;
        t.start();
        while (replies < kFlood && t.elapsed() < 3000) {
            QCoreApplication::processEvents(QEventLoop::AllEvents);
            replies += r.drainReplies().first;
        }
        QCOMPARE(replies, kFlood);
        const StatusSnapshot s = r.snap();
        QCOMPARE(s.state, ControlState::Fault);
        QVERIFY2(s.faultReason.contains(QStringLiteral("lost")),
                 qPrintable(s.faultReason));
    }

    // ── 突发排空：主机侧停顿，KRC 继续发 ──
    // 这是 --restart-gap-ms 造不出来的那一半（那个是 KRC 停发，主机侧无积压）。
    // 主机停顿 500ms 期间约 41 个数据报堆在接收缓冲里，且每一帧都带着间隙前
    // 那个陈旧位姿（KRC 没收到修正，机器人没动），所以误差始终顶格。若每帧
    // 都按"一个配置周期"发放预算，一次恢复就能吐出 41 × 0.6 ≈ 25mm——正好是
    // KRC 侧 POSCORR 硬限的量级。预算按实测帧间隔计算后，整批积压只值它真正
    // 占用的那几毫秒墙钟。
    void hostStallBacklog_drainsWithinOneCycleOfBudget()
    {
        Rig r(59235);
        r.feed(poseAt(kActualX), 1000, 3);

        // 远目标：误差 100mm × kp 0.3 = 30mm ≫ 0.6mm 限值，每帧都顶格。
        r.worker->applyTarget(poseAt(kActualX + 100.0));
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1003, 3);
        QCOMPARE(r.snap().state, ControlState::Tracking);
        r.drainReplies();          // 只统计排空事件本身发出的增量

        // qSleep 而不是 qWait：必须不跑事件循环，否则就不是"主机停顿"了。
        QTest::qSleep(500);
        constexpr int kBacklog = 41;      // 500ms / 12ms
        r.blast(poseAt(kActualX), 1006, kBacklog);

        // 排空占用的墙钟：断言的判据是"这批增量值不值这么多时间"，而不是一个
        // 写死的毫米数——排空快慢取决于构建类型与机器负载（Debug 下每帧约
        // 0.8ms，Release 下约 0.07ms），写死数字只会让用例在别的机器上变成
        // 抛硬币。
        QElapsedTimer drain;
        drain.start();
        // 手写自旋而不是 QTRY_COMPARE：QTRY 内部按 qWait(step) 分段等待，测出
        // 来的"排空耗时"会是它自己的轮询节奏（实测把 41 帧拖到 100ms），而生产
        // 里主机恢复后事件循环是空的，readyRead 会连着重入——那正是危险所在。
        // 这里让事件循环全速转，复现的才是真正的突发排空。
        while (r.snap().frameCount < quint64(6 + kBacklog) && drain.elapsed() < 5000)
            QCoreApplication::processEvents(QEventLoop::AllEvents);
        const double drainMs = drain.nsecsElapsed() / 1.0e6;
        QCOMPARE(r.snap().frameCount, quint64(6 + kBacklog));
        const auto [replies, sumX] = r.drainReplies();
        qInfo("PROBE replies=%d sumX=%.4f mm drain=%.2f ms", replies, sumX, drainMs);

        // 积压必须真的被处理过，否则"少发增量"可能只是因为帧被丢了——
        // 那不是修复，是另一个缺陷。
        QVERIFY2(replies >= kBacklog, "积压帧没被处理完，本用例没测到排空");
        // 排空本身若慢到与停顿同量级，判据的区分力就没了（vmax × 500ms 正好
        // 是 25mm，缺陷值本身）。先钉住这个前提。
        QVERIFY2(drainMs < 200.0, "排空耗时与停顿同量级，本用例失去区分力");

        // 安全属性：一次排空发出的总增量 ≤ 一个周期的满预算（跨越停顿的那
        // 一帧，被封顶）+ vmax × 排空真正占用的墙钟。即"机器人的平均速度不
        // 超过 vmax"，与积压多深无关。修复前这里是 41 × 0.6 = 24.6mm。
        const double bound = 0.6 + 50.0 * drainMs / 1000.0;
        QVERIFY2(sumX <= bound + 1e-9,
                 qPrintable(QStringLiteral("排空吐出 %1 mm，上界 %2 mm")
                                .arg(sumX).arg(bound)));
        // 排空不得顺带把跟踪弄停：预算收紧的代价必须只落在幅值上。
        QTRY_COMPARE(r.snap().state, ControlState::Tracking);
    }

    // ── First 分支重开预算基准（onDatagram 里那句 m_sinceLastStep.start()）──
    //
    // 这条路径与上面的突发排空是两件不同的事，很容易被混为一谈：主机侧停顿
    // 时 KRC 的 IPOC 流是连续的，IpocTracker 从未被 reset，所以排空帧一律
    // 分类为 Normal，根本进不了 First 分支。IpocEvent::First 只可能出现在
    // IpocTracker::reset() 之后，而生产里只有 onWatchdog()/start() 会 reset
    // ——也就是说这一支只在"看门狗抢在 readyRead 之前赢得竞争"时才会走到。
    //
    // 用"KRC 停发"而不是"主机停顿"来构造它：停发期间事件循环照转、且没有任何
    // 数据报与看门狗抢事件，看门狗必然先跑，这是那场竞争的确定性一侧。想在
    // 测试里制造真正的竞争（既有积压又让定时器抢先）是抛硬币，不该写进用例。
    //
    // 钉住的性质：恢复后第一次 step 的预算从"恢复这一刻"（First 帧）起算。
    // 删掉那句 start() 时，基准在看门狗里已被 invalidate()，操作员确认恢复后
    // 的第一个满预算帧只能领到 0——静默不动一次，而其它断言全部照常通过。
    // 看门狗静默现已转 Fault(P0-2),故恢复流程含操作员归零/重使能一步。
    void watchdogGapRecovery_restartsStepBudgetBasis()
    {
        Rig r(59237);
        r.feed(poseAt(kActualX), 1000, 3);

        // 远目标：误差 100mm × kp 0.3 = 30mm ≫ 0.6mm 限值，每帧都顶格，
        // 于是"这一帧领到多少预算"能被回包里的 RKorr.X 直接读出来。
        r.worker->applyTarget(poseAt(kActualX + 100.0));
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1003, 3);
        QCOMPARE(r.snap().state, ControlState::Tracking);

        // KRC 停发 600ms：> 看门狗 240ms（必触发 reset + invalidate + Fault），
        // < sessionGapMs 1200ms（不算真会话重启）。
        QTest::qWait(600);
        QVERIFY2(!r.snap().connected, "看门狗没触发，本用例没测到要测的路径");
        QCOMPARE(r.snap().state, ControlState::Fault);
        r.drainReplies();          // 只统计恢复事件本身发出的增量

        // 恢复首帧：走 First 分支(重开预算基准)。控制器在 Fault,必回零增量。
        r.feed(poseAt(kActualX), 2000, 1);
        const auto [n1, firstFrameX] = r.drainReplies();
        QCOMPARE(n1, 1);
        QCOMPARE(firstFrameX, 0.0);

        // 操作员确认:归零 → 重设目标 → 重新使能。
        r.worker->resetToActual();
        r.worker->applyTarget(poseAt(kActualX + 100.0));
        r.worker->setTracking(true);

        // 距 First 帧一个整周期后来帧。基准若在 First 帧重开过，它领到一个
        // 完整周期预算（12ms → 0.6mm 封顶）；若没重开，基准 invalid → 0 预算。
        QTest::qWait(12);
        r.feed(poseAt(kActualX), 2001, 1);
        const auto [n2, afterX] = r.drainReplies();
        qInfo("GAP-RECOVERY first step after recovery = %.4f mm", afterX);
        QCOMPARE(n2, 1);
        QVERIFY2(afterX > 0.55,
                 qPrintable(QStringLiteral(
                     "恢复后第一帧只发了 %1 mm：预算基准没在 First 帧重开")
                                .arg(afterX)));
        // 封顶仍然生效：跨越 600ms 静默的那一段绝不能被当成预算发出去。
        QVERIFY(afterX <= 0.6 + 1e-9);
    }

    // ── 连续丢包达到 watchdog_miss_limit：转可见 Fault，不许静默停跟踪 ──
    // 设计文档（2026-07-30 spec）写的就是"超过即转 Fault"，实现走样成了
    // setTracking(false)——状态无声退回 Ready，界面上没有任何征兆。
    // 2026-08-04 现场排查难点正在于此：机械臂纹丝不动、无报错、位姿照常刷新。
    void missLimitReached_faultsVisibly()
    {
        Rig r(59238);
        r.feed(poseAt(kActualX), 1000, 2);           // 建立节拍（步长 1）
        r.worker->applyTarget(poseAt(kActualX + kTargetOffsetX));
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1002, 1);
        QCOMPARE(r.snap().state, ControlState::Tracking);

        // 单帧前向跳号，缺口超过阈值
        const quint64 jump = 1003 + quint64(r.cfg.watchdogMissLimit) + 2;
        r.feed(poseAt(kActualX), jump, 1);

        const StatusSnapshot s = r.snap();
        QCOMPARE(s.state, ControlState::Fault);
        QVERIFY2(s.faultReason.contains(QStringLiteral("lost")),
                 qPrintable(QStringLiteral("faultReason 应说明丢包原因，实际: %1")
                                .arg(s.faultReason)));
    }

    // 正常配速下预算必须与"按配置周期算"一致：这是"单调安全"论证的另一半，
    // 否则一个把预算无脑调小的实现也能通过上面那个用例。
    void normalPacing_stillEmitsFullPerCycleBudget()
    {
        Rig r(59236);
        r.feed(poseAt(kActualX), 1000, 3);
        r.worker->applyTarget(poseAt(kActualX + 100.0));
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1003, 1);
        QCOMPARE(r.snap().state, ControlState::Tracking);
        r.drainReplies();

        // 按 12ms 边界配速发 10 帧。实测间隔总在 12ms 附近抖动，预算被封在
        // 一个配置周期，故每帧 ≤0.6 且应非常接近 0.6。
        constexpr int kN = 10;
        for (int i = 0; i < kN; ++i) {
            QTest::qWait(12);
            r.feed(poseAt(kActualX), 1004 + quint64(i), 1);
        }
        const auto [replies, sumX] = r.drainReplies();
        qInfo("PACED replies=%d sumX=%.4f mm", replies, sumX);
        QCOMPARE(replies, kN);
        // 上界：每帧不得超过一个周期的预算（封顶生效）
        QVERIFY(sumX <= kN * 0.6 + 1e-9);
        // 下界：正常配速不得被误伤成"排空"。qWait 只保证 ≥12ms，调度抖动
        // 只会让间隔更长（被封顶），不会更短，故 95% 是很宽的余量。
        QVERIFY2(sumX >= kN * 0.6 * 0.95,
                 qPrintable(QStringLiteral("正常配速只发了 %1 mm，预算被误伤")
                                .arg(sumX)));
    }
};

QTEST_MAIN(TestRsiWorker)
#include "test_rsi_worker.moc"
