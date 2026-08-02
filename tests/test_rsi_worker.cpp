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
    void watchdogGapRecovery_keepsTargetAndTracking()
    {
        Rig r(59232);
        r.feed(poseAt(kActualX), 1000, 3);

        const Pose target = poseAt(kActualX + kTargetOffsetX);
        r.worker->applyTarget(target);
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1003, 3);
        QCOMPARE(r.snap().state, ControlState::Tracking);
        QCOMPARE(r.snap().target.x, target.x);

        // 静默 600ms：看门狗（240ms）必然触发并 reset 了 IpocTracker，
        // 但远不到 1200ms 的会话间隔。
        QTest::qWait(600);
        QVERIFY2(!r.snap().connected, "看门狗没触发，本用例没测到要测的路径");

        // 恢复：KRC 没重启，IPOC 从中断处继续（这里刻意换一段号，因为
        // IpocTracker 已被 reset，任何号都会被判为 First——这正是缺陷的入口）。
        r.feed(poseAt(kActualX), 2000, 1);
        // 恢复首帧本身是同步瞬间（Syncing），目标必须已经保住。
        QCOMPARE(r.snap().target.x, target.x);

        // 下一帧应立刻回到 Tracking——修复前这里是 Ready：resetToActual 把
        // 控制器打回了 Idle，跟踪无声停止。
        r.feed(poseAt(kActualX), 2001, 3);
        const StatusSnapshot s = r.snap();
        QCOMPARE(s.target.x, target.x);
        QCOMPARE(s.state, ControlState::Tracking);
        // 实时误差也必须还在：修复前 target 被改写成 actual，误差被清零，
        // 现场看到的就是这一幕。
        QVERIFY(std::fabs(s.error.x - kTargetOffsetX) < 1e-6);
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

    // ── 保留跟踪状态的安全前提 ──
    // 「间隙后继续跟踪」之所以可以接受，靠的是间隙期间机器人若真被挪动过，
    // 恢复首帧会被物理跳变检测抓住。这条依赖 m_havePrevPose 在看门狗路径上
    // 保持为真（与会话重启相反，那里刻意清掉）。若有人日后为了「省事」在
    // 间隙恢复分支里也清掉它，跳变检测就会静默失效而所有其它断言照常通过
    // ——本用例就是钉住这一点。
    void watchdogGapRecovery_stillDetectsPhysicalJump()
    {
        Rig r(59234);
        r.feed(poseAt(kActualX), 1000, 3);
        r.worker->applyTarget(poseAt(kActualX + kTargetOffsetX));
        r.worker->setTracking(true);
        r.feed(poseAt(kActualX), 1003, 3);

        QTest::qWait(600);   // 落进「> 看门狗 240ms，< 会话 1200ms」

        // 恢复首帧位姿偏离 50mm，远超一个周期的物理预算
        //（physVmaxPosMmS 500 × 12ms = 6mm）。
        r.feed(poseAt(kActualX + 50.0), 2000, 1);
        QCOMPARE(r.snap().state, ControlState::StaleFrame);
        // 且本周期一定回零增量：反馈可疑时不许发修正。
        QCOMPARE(r.snap().lastDelta.x, 0.0);
    }
};

QTEST_MAIN(TestRsiWorker)
#include "test_rsi_worker.moc"
