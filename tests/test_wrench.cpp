#include <QtTest>
#include "core/Wrench.h"

class TestWrench : public QObject
{
    Q_OBJECT
private slots:
    void defaultIsZero()
    {
        WrenchFrame w;
        QCOMPARE(w.fx, 0.0);
        QCOMPARE(w.fy, 0.0);
        QCOMPARE(w.fz, 0.0);
        QCOMPARE(w.mx, 0.0);
        QCOMPARE(w.my, 0.0);
        QCOMPARE(w.mz, 0.0);
        QCOMPARE(w.fresh, false);
    }

    void metatypeRegistered()
    {
        const int id = QMetaType::fromType<WrenchFrame>().id();
        QVERIFY(id != QMetaType::UnknownType);
    }

    void valuesSurviveRoundtrip()
    {
        WrenchFrame w;
        w.fx = 12.5; w.fy = -3.25; w.fz = 100.0;
        w.mx = 0.1;  w.my = 2.5;   w.mz = -0.75;
        w.fresh = true;
        QCOMPARE(w.fx, 12.5);
        QCOMPARE(w.fy, -3.25);
        QCOMPARE(w.fz, 100.0);
        QCOMPARE(w.mx, 0.1);
        QCOMPARE(w.my, 2.5);
        QCOMPARE(w.mz, -0.75);
        QCOMPARE(w.fresh, true);
    }
};

QTEST_MAIN(TestWrench)
#include "test_wrench.moc"
