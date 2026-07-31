#include <QtTest>
#include "core/Pose.h"

class TestPose : public QObject
{
    Q_OBJECT
private slots:
    void wrap180_withinRange()
    {
        QCOMPARE(wrap180(0.0), 0.0);
        QCOMPARE(wrap180(90.0), 90.0);
        QCOMPARE(wrap180(-90.0), -90.0);
        QCOMPARE(wrap180(180.0), 180.0);
        QCOMPARE(wrap180(-180.0), 180.0);
    }

    void wrap180_crossesBoundary()
    {
        // 关键用例：目标 179，实际 -179，差值 358 应折成 -2
        QCOMPARE(wrap180(358.0), -2.0);
        QCOMPARE(wrap180(-358.0), 2.0);
        QCOMPARE(wrap180(181.0), -179.0);
        QCOMPARE(wrap180(-181.0), 179.0);
    }

    void wrap180_multipleTurns()
    {
        QCOMPARE(wrap180(720.0), 0.0);
        QCOMPARE(wrap180(725.0), 5.0);
        QCOMPARE(wrap180(-725.0), -5.0);
        QCOMPARE(wrap180(540.0), 180.0);
    }

    void poseSub_wrapsRotationOnly()
    {
        Pose target{10.0, 20.0, 30.0, 179.0, 5.0, -170.0};
        Pose actual{1.0, 2.0, 3.0, -179.0, -5.0, 170.0};
        const Pose d = poseSub(target, actual);
        // 位置分量：直接相减，不做角度归一化
        QCOMPARE(d.x, 9.0);
        QCOMPARE(d.y, 18.0);
        QCOMPARE(d.z, 27.0);
        // 姿态分量：取最短角路径
        QCOMPARE(d.a, -2.0);    // 179 - (-179) = 358 -> -2
        QCOMPARE(d.b, 10.0);    // 5 - (-5) = 10, 无需归一化
        QCOMPARE(d.c, 20.0);    // -170 - 170 = -340 -> 20
    }
};

QTEST_MAIN(TestPose)
#include "test_pose.moc"
