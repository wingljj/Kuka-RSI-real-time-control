#include <QtTest>
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
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});   // 巨大误差
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);                        // 未使能 → 不动
        QCOMPARE(pc.state(), TrackState::Idle);
    }

    void zeroError_producesZeroDelta()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.resetToActual(Pose{10, 20, 30, 1, 2, 3});
        pc.setTracking(true);
        const Pose d = pc.step(Pose{10, 20, 30, 1, 2, 3});
        QCOMPARE(d.x, 0.0);
        QCOMPARE(d.a, 0.0);
    }

    void largeError_isClampedToStepLimit()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
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
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
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
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
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
        pc.resetToActual(Pose{0, 0, 0, -179, 0, 0});
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
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;               // 模拟机器人跟随
        }
        QVERIFY(qAbs(pc.accumulated().x - 1.8) < 1e-9);  // 3 * 0.6
    }

    void accumOverLimit_entersFaultAndStopsMoving()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;     // 两步就越限
        pc.configure(c);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
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
        QVERIFY(qAbs(pc.accumulated().x) <= 1.0 + 1e-9);
    }

    void rotationAccumHasOwnLimit()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitRotDeg = 0.2;    // 姿态先越限
        c.accumLimitPosMm  = 1000.0; // 位置不越限
        pc.configure(c);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 90, 0, 0});

        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i) {
            const Pose d = pc.step(actual);
            actual.a += d.a;
        }
        QCOMPARE(pc.state(), TrackState::Fault);
    }

    void resetToActual_clearsFaultAndAccum()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;
        pc.configure(c);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i)
            actual.x += pc.step(actual).x;
        QCOMPARE(pc.state(), TrackState::Fault);

        pc.resetToActual(Pose{7, 8, 9, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
        QCOMPARE(pc.accumulated().x, 0.0);
        QCOMPARE(pc.target().x, 7.0);      // 目标 = 实际，误差归零
    }
};

QTEST_MAIN(TestPoseController)
#include "test_pose_controller.moc"
