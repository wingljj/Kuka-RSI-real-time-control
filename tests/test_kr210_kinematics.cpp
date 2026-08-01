#include <QtTest>
#include <cmath>
#include "tools/krc_simulator/kinematics.h"

class TestKr210 : public QObject
{
    Q_OBJECT
private slots:
    void forward_zeroPose()
    {
        // q 全 0：位置 = (a1+a2+a3, 0, d1+d4+d6)，姿态 0
        const double q[6] = {0, 0, 0, 0, 0, 0};
        const Pose p = kr210::forward(q);
        QVERIFY(qAbs(p.x - (350 + 1150 + 41)) < 1e-6);
        QVERIFY(qAbs(p.y) < 1e-6);
        QVERIFY(qAbs(p.z - (675 + 1200 + 240)) < 1e-6);
        QVERIFY(qAbs(p.a) < 1e-6);
        QVERIFY(qAbs(p.b) < 1e-6);
        QVERIFY(qAbs(p.c) < 1e-6);
    }

    void forward_zRotationOnly()
    {
        // A1=90° 只绕基座 Z 转 90°：位置 x→-y 方向（y 变负），z 不变
        const double q[6] = {90.0 * M_PI / 180.0, 0, 0, 0, 0, 0};
        const Pose p = kr210::forward(q);
        // 位置应绕 Z 转 90°：原 (1541,0) → (0,1541)？验证一个旋转关系：
        // |p.x| ≈ 0（绕 Z 90° 后 x 方向分量消失）
        QVERIFY(qAbs(p.x) < 1e-3);
        QVERIFY(qAbs(p.y - 1541.0) < 1e-3);
        QVERIFY(qAbs(p.z - 2115.0) < 1e-3);
        QVERIFY(qAbs(p.a - 90.0) < 1e-6);
    }

    void jacobian_matchesNumerical()
    {
        const double q[6] = {0.3, -0.6, 0.8, 0.2, 0.5, -0.4};
        double J[6][6];
        kr210::jacobian(q, J);
        constexpr double h = 1e-7;
        for (int col = 0; col < 6; ++col) {
            double qp[6], qm[6];
            std::copy(q, q + 6, qp);
            std::copy(q, q + 6, qm);
            qp[col] += h; qm[col] -= h;
            const Pose fp = kr210::forward(qp);
            const Pose fm = kr210::forward(qm);
            const double dq[6] = {fp.x - fm.x, fp.y - fm.y, fp.z - fm.z,
                                  (fp.a - fm.a) * M_PI / 180.0,
                                  (fp.b - fm.b) * M_PI / 180.0,
                                  (fp.c - fm.c) * M_PI / 180.0};
            for (int row = 0; row < 6; ++row)
                QVERIFY2(qAbs(J[row][col] - dq[row] / (2 * h)) < 1e-5,
                         "jacobian mismatch with numerical");
        }
    }

    void solveDelta_roundTrip()
    {
        const double q[6] = {0.2, -0.5, 0.7, 0.1, 0.4, -0.3};
        const Pose dx{5.0, -3.0, 2.0, 1.0, -0.5, 0.8};   // mm, 度
        double dq[6];
        QVERIFY(kr210::solveDelta(q, dx, dq));
        // 用雅可比线性化验证：J·Δq ≈ Δx
        double J[6][6];
        kr210::jacobian(q, J);
        const double dxRot[3] = {dx.a * M_PI / 180.0, dx.b * M_PI / 180.0,
                                 dx.c * M_PI / 180.0};
        const double got[6] = {
            J[0][0]*dq[0]+J[0][1]*dq[1]+J[0][2]*dq[2]+J[0][3]*dq[3]+J[0][4]*dq[4]+J[0][5]*dq[5],
            J[1][0]*dq[0]+J[1][1]*dq[1]+J[1][2]*dq[2]+J[1][3]*dq[3]+J[1][4]*dq[4]+J[1][5]*dq[5],
            J[2][0]*dq[0]+J[2][1]*dq[1]+J[2][2]*dq[2]+J[2][3]*dq[3]+J[2][4]*dq[4]+J[2][5]*dq[5],
            J[3][0]*dq[0]+J[3][1]*dq[1]+J[3][2]*dq[2]+J[3][3]*dq[3]+J[3][4]*dq[4]+J[3][5]*dq[5],
            J[4][0]*dq[0]+J[4][1]*dq[1]+J[4][2]*dq[2]+J[4][3]*dq[3]+J[4][4]*dq[4]+J[4][5]*dq[5],
            J[5][0]*dq[0]+J[5][1]*dq[1]+J[5][2]*dq[2]+J[5][3]*dq[3]+J[5][4]*dq[4]+J[5][5]*dq[5],
        };
        QVERIFY(qAbs(got[0] - dx.x) < 1e-6);
        QVERIFY(qAbs(got[1] - dx.y) < 1e-6);
        QVERIFY(qAbs(got[2] - dx.z) < 1e-6);
        QVERIFY(qAbs(got[3] - dxRot[0]) < 1e-6);
        QVERIFY(qAbs(got[4] - dxRot[1]) < 1e-6);
        QVERIFY(qAbs(got[5] - dxRot[2]) < 1e-6);
    }

    void limitCartDelta_speedClamps()
    {
        Pose dx{100.0, 0, 0, 0, 0, 0};          // 大增量
        const Pose prev{};                       // 0
        kr210::limitCartDelta(&dx, prev, 50.0, 10.0, 0, 0, 0.012);
        QVERIFY(qAbs(dx.x - 0.6) < 1e-9);        // 50 mm/s × 0.012 s
        QCOMPARE(dx.y, 0.0);
    }

    void limitCartDelta_accelClamps()
    {
        Pose dx{0.6, 0, 0, 0, 0, 0};             // 已达速度限
        const Pose prev{0.6, 0, 0, 0, 0, 0};     // 上一周期也是 0.6（匀速）
        // 加速度限 1000 mm/s²：变化 ≤ 1000×0.012² = 0.144/周期
        kr210::limitCartDelta(&dx, prev, 1000.0, 1000.0, 1000.0, 1000.0, 0.012);
        QVERIFY(qAbs(dx.x - (0.6 - 0.144)) < 1e-9);  // 被压回 0.456
        QVERIFY(qAbs(dx.x - 0.456) < 1e-9);
    }

    void limitCartDelta_zeroLimits_passthrough()
    {
        Pose dx{1.234, -2.5, 0.1, 0, 0, 0};
        const Pose prev{};
        kr210::limitCartDelta(&dx, prev, 0, 0, 0, 0, 0.012);
        QCOMPARE(dx.x, 1.234);
        QCOMPARE(dx.y, -2.5);
    }

    void euler_singularityBranch()
    {
        // B = +90°：R = Rz(A)Ry(90)Rx(0)。验证 eulerFromR 走奇异分支不崩且合理。
        const double q[6] = {0.5, 0.5 * M_PI, 0, 0, 0, 0};
        const Pose p = kr210::forward(q);
        QVERIFY(qAbs(p.b - 90.0) < 1e-6);
        QVERIFY(std::isfinite(p.a) && std::isfinite(p.c));
    }
};
QTEST_MAIN(TestKr210)
#include "test_kr210_kinematics.moc"
