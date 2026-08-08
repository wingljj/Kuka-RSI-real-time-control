#include <QtTest>
#include <cmath>
#include "core/ForceController.h"
#include "core/Wrench.h"

static ForceControlConfig testCfg()
{
    ForceControlConfig c;
    c.params.cutoffHz = 10.0;
    c.params.deadzoneForceN = 5.0;
    c.params.deadzoneTorqueNm = 1.0;
    c.params.gainForce = 0.05;
    c.params.gainTorque = 0.5;
    c.params.vmaxPosMmS = 5.0;
    c.params.vmaxRotDegS = 1.0;
    c.axes.enZ = true;
    c.mounting.flangeTSensor[2] = 0.0;  // z=0 for simplicity
    c.mounting.flangeTTool[2] = 0.0;
    return c;
}

class TestForceController : public QObject
{
    Q_OBJECT
private slots:
    void inactiveReturnsZero()
    {
        ForceController fc;
        fc.configure(testCfg());
        WrenchFrame w; w.fresh = true;
        const Pose out = fc.step(w, Pose{}, 0.004);
        QCOMPARE(out.x, 0.0); QCOMPARE(out.y, 0.0); QCOMPARE(out.z, 0.0);
    }

    void sigmoidDeadzoneSuppressesSmallForce()
    {
        const double v = ForceController::sigmoid(3.0, 5.0, 0.05, 5.0);
        QCOMPARE(v, 0.0);  // 3.0 < 5.0 deadzone
    }

    void sigmoidSaturatesAtLargeForce()
    {
        const double v = ForceController::sigmoid(200.0, 5.0, 0.05, 5.0);
        QVERIFY(std::abs(v - 5.0) < 0.01);  // should saturate at vmax
    }

    void sigmoidIsMonotonic()
    {
        double prev = 0.0;
        for (double f = 0.0; f <= 500.0; f += 1.0) {
            const double v = ForceController::sigmoid(f, 5.0, 0.05, 5.0);
            QVERIFY(v >= prev - 1e-12);
            prev = v;
        }
    }

    void enableRecordsBias()
    {
        ForceController fc;
        fc.configure(testCfg());
        WrenchFrame bias; bias.fx = 10.0; bias.fy = 2.0; bias.fz = -30.0;
        fc.enable(Pose{}, bias);
        QVERIFY(fc.isActive());
        QCOMPARE(fc.bias().fx, 10.0);
        QCOMPARE(fc.bias().fz, -30.0);
    }

    void zeroForceOutputsZeroDelta()
    {
        ForceController fc;
        fc.configure(testCfg());
        WrenchFrame bias;  // all zero
        fc.enable(Pose{}, bias);
        // Feed exactly the bias (net force = 0)
        WrenchFrame w; w.fresh = true;  // all zeros = bias
        // After filters settle, output should be zero
        for (int i = 0; i < 500; ++i)
            fc.step(w, Pose{}, 0.004);
        const Pose out = fc.step(w, Pose{}, 0.004);
        QVERIFY(std::abs(out.x) < 0.001);
        QVERIFY(std::abs(out.y) < 0.001);
        QVERIFY(std::abs(out.z) < 0.001);
    }

    void axisMaskSuppressesDisabledDirection()
    {
        ForceController fc;
        ForceControlConfig cfg = testCfg();
        cfg.axes.enZ = true;
        cfg.axes.enX = false;
        cfg.axes.enY = false;
        cfg.params.deadzoneForceN = 0.0;  // no deadzone
        fc.configure(cfg);
        WrenchFrame bias;
        fc.enable(Pose{}, bias);

        // Apply force in X direction
        WrenchFrame w; w.fresh = true;
        w.fx = 50.0; w.fy = 0.0; w.fz = 50.0;

        for (int i = 0; i < 500; ++i)
            fc.step(w, Pose{}, 0.004);
        const Pose out = fc.step(w, Pose{}, 0.004);
        // X component should be suppressed (en_x=false)
        QVERIFY(std::abs(out.x) < 0.01);
        // Z component should move
        QVERIFY(out.z > 0.001);
    }

    void transformToBaseRotatesForceWithOrientation()
    {
        // Sensor and tool axes coincident (identity mounting): base-frame wrench
        // at A=B=C=0 equals the sensor values; at A=90° (KUKA A = rotation about
        // base Z), sensor Fx=1 maps to base Fy=+1.
        ForceController fc;
        ForceControlConfig cfg = testCfg();
        fc.configure(cfg);
        const float sv[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        const WrenchFrame w0 = fc.transformToBase(sv, 0.0, 0.0, 0.0);
        QVERIFY(std::abs(w0.fx - 1.0) < 1e-9);
        QVERIFY(std::abs(w0.fy) < 1e-9);
        const WrenchFrame w90 = fc.transformToBase(sv, 90.0, 0.0, 0.0);
        QVERIFY(std::abs(w90.fx) < 1e-9);
        QVERIFY(std::abs(w90.fy - 1.0) < 1e-9);
        // Torque channel is rotated the same way.
        const float svM[6] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
        const WrenchFrame m90 = fc.transformToBase(svM, 90.0, 0.0, 0.0);
        QVERIFY(std::abs(m90.mx) < 1e-9);
        QVERIFY(std::abs(m90.my - 1.0) < 1e-9);
    }

    void commandedSumAccumulatesDeltas()
    {
        ForceController fc;
        ForceControlConfig cfg = testCfg();
        cfg.axes.enZ = true;
        cfg.axes.enX = false;
        cfg.axes.enY = false;
        cfg.params.deadzoneForceN = 0.0;
        fc.configure(cfg);
        WrenchFrame bias;
        fc.enable(Pose{}, bias);

        WrenchFrame w; w.fresh = true;
        w.fz = 50.0;
        double sum = 0.0;
        for (int i = 0; i < 100; ++i) {
            const Pose d = fc.step(w, Pose{}, 0.004);
            sum += d.z;
        }
        QVERIFY(fc.commandedSum().z > 0.5);
        QVERIFY(std::abs(fc.commandedSum().z - sum) < 1e-9);
    }
};
QTEST_MAIN(TestForceController)
#include "test_force_controller.moc"
