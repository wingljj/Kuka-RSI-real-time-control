#include <QtTest>
#include <cmath>
#include "core/ForceFilter.h"

class TestForceFilter : public QObject
{
    Q_OBJECT
private slots:
    void passThroughAtDc()
    {
        Butterworth2 f;
        f.configure(10.0, 250.0);
        // After many steps with constant input, output should converge to input
        for (int i = 0; i < 500; ++i)
            f.step(1.0);
        const double out = f.step(1.0);
        QVERIFY(std::abs(out - 1.0) < 0.01);
    }

    void attenuatesAboveCutoff()
    {
        Butterworth2 f;
        f.configure(10.0, 250.0);
        // Feed a 100 Hz sine (0.8·Nyquist) — should be heavily attenuated.
        // 注意不能取 125 Hz（=Nyquist）：sin(2π·125·i/250)=sin(π·i)≡0，
        // 采样后输入恒为 0，测不出任何衰减。
        const double pi = 3.141592653589793;
        double maxOut = 0.0;
        for (int i = 0; i < 1000; ++i) {
            const double t = double(i) / 250.0;
            const double in = std::sin(2.0 * pi * 100.0 * t);
            const double out = f.step(in);
            maxOut = std::max(maxOut, std::abs(out));
        }
        // At 100 Hz (fc=10 Hz), Butterworth 2nd order gives ~34 dB attenuation
        QVERIFY(maxOut < 0.2);
    }

    void resetPresetsState()
    {
        Butterworth2 f;
        f.configure(10.0, 250.0);
        // Drive to non-zero state
        for (int i = 0; i < 100; ++i)
            f.step(5.0);
        // Reset with initial value
        f.reset(5.0);
        // First step after reset should be near 5.0
        const double out = f.step(5.0);
        QVERIFY(std::abs(out - 5.0) < 0.1);
    }

    void defaultIsUnity()
    {
        Butterworth2 f;  // unconfigured — pass-through
        const double out = f.step(42.0);
        QCOMPARE(out, 42.0);
    }

    void invalidConfigIsPassThrough()
    {
        // fc ≥ fs/2 would make the bilinear poles leave the unit circle;
        // configure() must fall back to pass-through instead of instability.
        Butterworth2 f;
        f.configure(250.0, 250.0);  // fc == fs/2
        QCOMPARE(f.step(3.0), 3.0);
        f.configure(300.0, 250.0);  // fc > fs/2
        QCOMPARE(f.step(3.0), 3.0);
        f.configure(10.0, 0.0);     // invalid fs
        QCOMPARE(f.step(3.0), 3.0);
    }

    void stepResponseRisesToTarget()
    {
        Butterworth2 f;
        f.configure(10.0, 250.0);
        f.reset(0.0);
        // Butterworth 的 Q=1/√2 固有约 4.4% 超调（模拟原型的解析超调），
        // 所以不能断言单调；断言「收敛到 1.0」+「超调有界」。
        double maxOut = 0.0;
        double prev = 0.0;
        for (int i = 0; i < 200; ++i) {
            const double out = f.step(1.0);
            maxOut = std::max(maxOut, out);
            prev = out;
        }
        QVERIFY(std::abs(prev - 1.0) < 0.01);  // settled at target
        QVERIFY(maxOut > 0.9);                 // it actually rose
        QVERIFY(maxOut < 1.1);                 // overshoot bounded (~4.4%)
    }
};
QTEST_MAIN(TestForceFilter)
#include "test_force_filter.moc"
