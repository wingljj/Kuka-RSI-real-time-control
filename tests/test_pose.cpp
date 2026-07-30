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
    }

    void poseSub_wrapsRotationOnly()
    {
        Pose target{10.0, 0.0, 0.0, 179.0, 0.0, 0.0};
        Pose actual{0.0, 0.0, 0.0, -179.0, 0.0, 0.0};
        const Pose d = poseSub(target, actual);
        QCOMPARE(d.x, 10.0);        // 位置直接相减，不 wrap
        QCOMPARE(d.a, -2.0);        // 姿态走近路
    }
};

QTEST_MAIN(TestPose)
#include "test_pose.moc"
