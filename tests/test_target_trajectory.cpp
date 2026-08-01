#include <QtTest>
#include <cmath>
#include "core/TargetTrajectory.h"
#include "core/PoseOps.h"

class TestTrajectory : public QObject
{
    Q_OBJECT
private slots:
    void endpointsExact()
    {
        TargetTrajectory tr;
        tr.setGoal(Pose{0,0,0,0,0,0}, Pose{100,50,0,30,-20,10}, 1000);
        QCOMPARE(tr.sample().x, 0.0);
        for (int i = 0; i < 100; ++i) tr.advance(0.01);
        const Pose p = tr.sample();
        QVERIFY(qAbs(p.x - 100) < 1e-6);
        QVERIFY(qAbs(p.y - 50) < 1e-6);
        QVERIFY(tr.isFinished());
    }

    void quinticZeroVelAtEnds()
    {
        // 数值差分：u 接近 0/1 时速度 ≈ 0
        TargetTrajectory tr;
        tr.setGoal(Pose{0,0,0,0,0,0}, Pose{100,0,0,0,0,0}, 1000);
        tr.advance(0.001);
        const double vEarly = tr.sample().x / 0.001;
        // 推进到接近结束
        for (int i = 0; i < 998; ++i) tr.advance(0.001);
        tr.advance(0.001);
        const double vLate = (100.0 - tr.sample().x) / 0.001;
        QVERIFY(qAbs(vEarly) < 0.1);
        QVERIFY(qAbs(vLate) < 0.1);
    }

    void slerpMidpoint()
    {
        // 90° 旋转：u=0.5 时中点 45°
        TargetTrajectory tr;
        tr.setGoal(Pose{0,0,0,0,0,0}, Pose{0,0,0,0,0,90}, 1000);
        tr.advance(0.5);
        const Pose p = tr.sample();
        QVERIFY(qAbs(p.c - 45.0) < 1e-6);
    }

    void zeroDurationImmediate()
    {
        TargetTrajectory tr;
        tr.setGoal(Pose{1,2,3,0,0,0}, Pose{9,8,7,0,0,0}, 0);
        QVERIFY(tr.isFinished());
        QCOMPARE(tr.sample().x, 9.0);
    }
};
QTEST_MAIN(TestTrajectory)
#include "test_target_trajectory.moc"
