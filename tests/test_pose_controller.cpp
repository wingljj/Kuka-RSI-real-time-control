#include <QtTest>
#include <cmath>
#include <limits>
#include "core/PoseController.h"

namespace {

AppConfig testCfg()
{
    AppConfig c = AppConfig::defaults();
    c.cycleMs            = 12.0;
    c.kpPos              = 1.0;    // 便于算术验证
    c.kpRot              = 1.0;
    c.vmaxPosMmS         = 50.0;   // 12ms → 步长上限 0.6mm
    c.vmaxRotDegS        = 10.0;   // 12ms → 步长上限 0.12°
    c.accumLimitPosMm    = 30.0;
    c.accumLimitRotDeg   = 15.0;
    c.targetSmoothingMs  = 0.0;    // 保持增量 = kp×误差 的精确算术断言
    return c;
}

} // namespace

class TestPoseController : public QObject
{
    Q_OBJECT
private slots:
    void notTracking_returnsZeroDelta()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});   // 巨大误差
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);                        // 未使能 → 不动
        QCOMPARE(pc.state(), TrackState::Idle);
    }

    void zeroError_producesZeroDelta()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{10, 20, 30, 1, 2, 3});
        pc.setTracking(true);
        const Pose d = pc.step(Pose{10, 20, 30, 1, 2, 3});
        QCOMPARE(d.x, 0.0);
        QCOMPARE(d.a, 0.0);
    }

    void largeError_isClampedToStepLimit()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 50mm/s * 0.012s = 0.6mm
        QVERIFY(qAbs(d.x - 0.6) < 1e-9);
    }

    void rotationClampUsesSeparateLimit()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 90, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 10deg/s * 0.012s = 0.12deg，且不受位置限值影响
        QVERIFY(qAbs(d.a - 0.12) < 1e-9);
        QCOMPARE(d.x, 0.0);
    }

    void smallError_notClamped()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = 0.5;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0.4, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 0.5 * 0.4 = 0.2 < 0.6 → 不限幅
        QVERIFY(qAbs(d.x - 0.2) < 1e-9);
    }

    void rotationTakesShortestPath()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, -179, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 179, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, -179, 0, 0});
        // 误差 wrap 成 -2° → 向负方向走，而非 +358°
        QVERIFY(d.a < 0.0);
        QVERIFY(qAbs(d.a + 0.12) < 1e-9);
    }

    void accumulation_tracksSumOfDeltas()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;               // 模拟机器人跟随
        }
        // 命令和仍是三步之和
        QVERIFY(qAbs(pc.commandedSum().x - 1.8) < 1e-9);  // 3 * 0.6
        // 锚点位移按每周期开始时的 actual 度量，故天然滞后一个周期：
        // 第 3 步开始时机器人只走了 2 * 0.6。这个滞后是有意的——只有
        // 控制器真正回传了新的 RIst，第 2 层才认这笔修正。
        QVERIFY(qAbs(pc.accumulated().x - 1.2) < 1e-9);   // 2 * 0.6
    }

    void accumOverLimit_entersFaultAndStopsMoving()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;     // 两步就越限
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});

        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;
        }
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(!pc.faultReason().isEmpty());
        // Fault 后必须返回零增量
        QCOMPARE(pc.step(actual).x, 0.0);
        // 锚点位移滞后一个周期，故越限时最多超出一个单周期步长（0.6mm）才被拦下
        QVERIFY(qAbs(pc.accumulated().x) <= 1.0 + 0.6 + 1e-9);
    }

    void rotationAccumHasOwnLimit()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitRotDeg = 0.2;    // 姿态先越限
        c.accumLimitPosMm  = 1000.0; // 位置不越限
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 90, 0, 0});

        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i) {
            const Pose d = pc.step(actual);
            actual.a += d.a;
        }
        QCOMPARE(pc.state(), TrackState::Fault);
    }

    void resetToActual_clearsFaultAndTargetButKeepsAccum()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i)
            actual.x += pc.step(actual).x;
        QCOMPARE(pc.state(), TrackState::Fault);

        // 归零前的累积量必须非零，否则本用例无从证明「保留」
        const double accumBefore = pc.accumulated().x;
        QVERIFY(qAbs(accumBefore) > 1e-9);

        pc.resetToActual(Pose{7, 8, 9, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
        QVERIFY(pc.faultReason().isEmpty());
        QCOMPARE(pc.target().x, 7.0);      // 目标 = 实际，误差归零
        // 【关键】KRC 侧已施加的修正不会消失，累积量必须原样保留
        QCOMPARE(pc.accumulated().x, accumBefore);

        // 【关键之二】会话锚点也必须原样保留：再走一个周期，位移仍以
        // 原锚点（0）度量，而不是以 resetToActual 传入的 {7,8,9} 度量。
        pc.setTracking(true);
        pc.step(actual);
        QCOMPARE(pc.accumulated().x, accumBefore);
        QVERIFY(qAbs(pc.accumulated().x - actual.x) < 1e-9);
    }

    void beginSession_clearsAccum()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i)
            actual.x += pc.step(actual).x;
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(qAbs(pc.accumulated().x) > 1e-9);

        // 仅 RSI 会话重启才可清零累积量
        pc.beginSession(Pose{7, 8, 9, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
        QVERIFY(pc.faultReason().isEmpty());
        QCOMPARE(pc.target().x, 7.0);
        QCOMPARE(pc.accumulated().x, 0.0);
    }

    void faultCannotBeReEnabledWithoutReset()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i)
            actual.x += pc.step(actual).x;
        QCOMPARE(pc.state(), TrackState::Fault);

        // 锁存的核心保证：不经 resetToActual 不得重新使能
        pc.setTracking(true);
        QCOMPARE(pc.state(), TrackState::Fault);
        QCOMPARE(pc.step(actual).x, 0.0);
    }

    void nonFinitePoseEntersFault()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        Pose bad{0, 0, 0, 0, 0, 0};
        bad.x = std::numeric_limits<double>::quiet_NaN();
        const Pose d = pc.step(bad);
        QCOMPARE(d.x, 0.0);
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(!pc.faultReason().isEmpty());
        // 累积量绝不能被 NaN 污染，否则第 2 层永久失效
        QVERIFY(std::isfinite(pc.accumulated().x));
    }

    void diagonalAccumUsesEuclideanNorm()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;      // 范数上限 1mm
        c.vmaxPosMmS      = 1000.0;   // 放开单周期限幅，便于快速累积
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        // 三轴各 0.7mm：逐轴判限会放过（0.7 < 1），欧氏范数 1.21 必须拦下。
        // 位移以控制器回传的 actual 度量，所以必须把运动喂回 actual。
        pc.setTarget(Pose{0.7, 0.7, 0.7, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 10 && pc.state() == TrackState::Tracking; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;
            actual.y += d.y;
            actual.z += d.z;
        }
        QCOMPARE(pc.state(), TrackState::Fault);
        // 被拦下的那一刻，逐轴都还在限内，只有欧氏范数越限——这正是本用例
        // 要钉住的性质（逐轴判限会放过它，合成位移就能顶穿 POSCORR 的硬限）。
        QVERIFY(qAbs(pc.displacement().x) < 1.0);
        QVERIFY(qAbs(pc.displacement().y) < 1.0);
        QVERIFY(qAbs(pc.displacement().z) < 1.0);
        QVERIFY(std::hypot(pc.displacement().x,
                           pc.displacement().y,
                           pc.displacement().z) > 1.0);
        QVERIFY(pc.faultReason().contains("displacement from session anchor"));
    }

    void negativeVmaxDoesNotInvertDirection()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.vmaxPosMmS = -50.0;         // 恶意/误填的负限速
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 必须朝目标走，绝不能因 clamp(lo>hi) 反向
        QVERIFY(d.x > 0.0);
        QVERIFY(qAbs(d.x - 0.6) < 1e-9);
    }

    void allSixComponentsClampIndependently()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        // 六个分量都给足够大的误差，全部应被各自的步长上限限住
        pc.setTarget(Pose{100, 100, 100, 90, 90, 90});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QVERIFY(qAbs(d.x - 0.6) < 1e-9);
        QVERIFY(qAbs(d.y - 0.6) < 1e-9);
        QVERIFY(qAbs(d.z - 0.6) < 1e-9);
        QVERIFY(qAbs(d.a - 0.12) < 1e-9);
        QVERIFY(qAbs(d.b - 0.12) < 1e-9);
        QVERIFY(qAbs(d.c - 0.12) < 1e-9);
    }

    void invalidCycleMsSurvivesReset()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.cycleMs = 0.0;
        pc.configure(c);
        // 生产调用顺序：configure → beginSession(首帧)。粘滞标志必须活过它
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        QVERIFY(pc.state() != TrackState::Tracking);   // 不得被使能
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(pc.faultReason().contains("cycleMs"));
    }

    void validConfigureClearsInvalidFlag()
    {
        PoseController pc;
        AppConfig bad = testCfg();
        bad.cycleMs = -1.0;
        pc.configure(bad);
        pc.configure(testCfg());       // 一次有效配置应解除粘滞
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        QCOMPARE(pc.state(), TrackState::Tracking);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        QVERIFY(pc.step(Pose{0, 0, 0, 0, 0, 0}).x > 0.0);
    }

    void nonFiniteGainEntersFault()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = std::numeric_limits<double>::quiet_NaN();
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{10, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);
        QCOMPARE(pc.state(), TrackState::Fault);
        // 累积量绝不能被污染，否则第 2 层永久失效
        QVERIFY(std::isfinite(pc.accumulated().x));
    }

    void negativeAccumLimitDoesNotFaultAtZeroError()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = -30.0;     // 误填负号
        pc.configure(c);
        pc.beginSession(Pose{5, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        // 误差为 0、累积为 0，不该因负限值立刻故障
        const Pose d = pc.step(Pose{5, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);
        QCOMPARE(pc.state(), TrackState::Tracking);
    }

    void subQuantumIncrementDoesNotAccumulate()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = 1e-6;          // 极小增益，使每周期增量远小于线上量化步长
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{1.0, 0, 0, 0, 0, 0});
        for (int i = 0; i < 1000; ++i)
            pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 每周期 1e-6 会被 buildSen 量化成 0.0000，机器人不动，
        // 所以账本也不该增长——否则收敛后会持续漂移。
        // actual 固定不动，位移当然是 0；真正钉住死区的是命令和。
        QCOMPARE(pc.accumulated().x, 0.0);
        QCOMPARE(pc.commandedSum().x, 0.0);
    }

    void inSessionResetDoesNotMoveTheAnchor()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 10; ++i)
            actual.x += pc.step(actual).x;
        // 再走一个周期但不喂回，使 before 就是"以当前 actual 度量的位移"，
        // 这样才能和归零后同一 actual 上的度量直接比较（位移天然滞后一拍）。
        pc.step(actual);
        const double before = pc.accumulated().x;
        QVERIFY(before > 0.0);

        pc.resetToActual(actual);      // 会话内归零：原点不得移动
        pc.setTracking(true);
        pc.step(actual);
        QVERIFY(qAbs(pc.accumulated().x - before) < 1e-9);
    }

    void beginSessionMovesTheAnchor()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 10; ++i)
            actual.x += pc.step(actual).x;
        QVERIFY(pc.accumulated().x > 0.0);

        pc.beginSession(actual);       // 真正的会话重启：原点移到此处
        QCOMPARE(pc.accumulated().x, 0.0);
        pc.setTracking(true);
        pc.step(actual);
        QCOMPARE(pc.accumulated().x, 0.0);
    }

    void forceFault_latchesUntilReset()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.forceFault(QStringLiteral("network write failed"));
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(pc.faultReason().contains("network write failed"));
        QCOMPARE(pc.step(Pose{0, 0, 0, 0, 0, 0}).x, 0.0);
        pc.setTracking(true);   // 不得直接重新使能
        QCOMPARE(pc.state(), TrackState::Fault);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
    }

    void rotatedOverLimit_firesViaCommandedSumEvenIfRistWraps()
    {
        // RIst 姿态角折返/不跟随：若机器人未跟随命令（丢包或卡住），RIst
        // 位移停在锚点附近，主机从 RIst 看不出已累计的修正量。
        // commandedSum（不折返）反映"主机以为发出去了多少修正"，必须兜底：
        // 0.6°/cycle，200° 需 ~334 周期。
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitRotDeg = 200.0;   // 高于单圈 180°，让 RIst 折返不触发
        c.vmaxRotDegS      = 50.0;    // 12ms → 0.6°/cycle，200° 需 ~334 周期
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 220, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 500 && pc.state() == TrackState::Tracking; ++i)
            pc.step(actual);              // RIst 不跟随：位移恒 0，命令持续累计
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(pc.faultReason().contains("rotation"));
    }

    void smoothing_progressivelyApproachesStepTarget()
    {
        // 放开限幅让增量 = kp×误差；无平滑时目标阶跃 100 第一周期误差=100
        // → 增量=50。平滑后平滑目标第一步 = 100×α，α=12/(12+50)=0.1935
        // → 误差≈19.35 → 增量≈9.68，显著削平。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        c.kpPos             = 0.5;
        c.vmaxPosMmS        = 1000000.0;   // 放开限幅
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QVERIFY(d1.x < 20.0);
        QVERIFY(qAbs(d1.x - 9.68) < 0.5);
    }

    void smoothing_tauZero_isPassthrough()
    {
        PoseController pc;
        AppConfig c = testCfg();            // targetSmoothingMs=0
        c.kpPos      = 0.5;
        c.vmaxPosMmS = 1000000.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QVERIFY(qAbs(d1.x - 50.0) < 1e-9);   // α=1 直通
    }

    void resetToActual_syncsSmoothTarget()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        c.kpPos             = 0.5;
        c.vmaxPosMmS        = 1000000.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        pc.step(Pose{0, 0, 0, 0, 0, 0});     // 平滑目标开始逼近（≈19.35）
        pc.resetToActual(Pose{3, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        // 平滑目标必须同步为 actual(3)，否则从 19.35 向 3 逼近 → 假误差 → 非零增量
        const Pose d = pc.step(Pose{3, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);
    }

    void smoothing_doesNotChangeSteadyState()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{5, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5000; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;
        }
        QVERIFY(qAbs(pc.target().x - actual.x) < 1e-3);
        QVERIFY(qAbs(pc.accumulated().x - 5.0) < 1e-3);
    }

    void smoothing_angularJumpTakesShortestPath()
    {
        // 目标 -179→+179（最短差 2°）。线性平滑会让 smooth 经 0 走 358° 长路；
        // wrap180 delta 应走经 180 的短路（第一周期增量方向为正）。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        c.kpRot             = 0.5;
        c.vmaxRotDegS       = 1000000.0;   // 放开限幅
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 179, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, -179, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 179, 0, 0});
        QVERIFY(d1.a > 0.0);   // 经 180 侧（短路径），而非经 0 侧
    }
};

QTEST_MAIN(TestPoseController)
#include "test_pose_controller.moc"
