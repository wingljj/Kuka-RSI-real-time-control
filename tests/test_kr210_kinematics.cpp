#include <QtTest>
#include <algorithm>
#include <cmath>
#include "tools/krc_simulator/kinematics.h"

namespace {

// 由 KUKA A/B/C（度，ZYX: R = Rz(A)·Ry(B)·Rx(C)）重建旋转矩阵。
void rotFromPose(const Pose &p, double R[3][3])
{
    const double a = p.a * M_PI / 180.0;
    const double b = p.b * M_PI / 180.0;
    const double c = p.c * M_PI / 180.0;
    const double ca = std::cos(a), sa = std::sin(a);
    const double cb = std::cos(b), sb = std::sin(b);
    const double cc = std::cos(c), sc = std::sin(c);
    R[0][0] = ca * cb;          R[0][1] = ca * sb * sc - sa * cc;  R[0][2] = ca * sb * cc + sa * sc;
    R[1][0] = sa * cb;          R[1][1] = sa * sb * sc + ca * cc;  R[1][2] = sa * sb * cc - ca * sc;
    R[2][0] = -sb;              R[2][1] = cb * sc;                 R[2][2] = cb * cc;
}

void mul3x3(const double A[3][3], const double B[3][3], double out[3][3])
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            out[r][c] = 0.0;
            for (int k = 0; k < 3; ++k)
                out[r][c] += A[r][k] * B[k][c];
        }
}

void trans3x3(const double A[3][3], double out[3][3])
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out[r][c] = A[c][r];
}

// 两旋转矩阵之间差的旋转角（rad）。
double rotAngleBetween(const double Ra[3][3], const double Rb[3][3])
{
    double t = 0.0;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            t += Ra[r][c] * Rb[r][c];   // tr(Ra^T Rb)
    const double v = std::clamp((t - 1.0) / 2.0, -1.0, 1.0);
    return std::acos(v);
}

} // namespace

class TestKr210 : public QObject
{
    Q_OBJECT
private slots:
    void forward_zeroPose()
    {
        // q 全 0：位置 = (a1+a2+a3, 0, d1−d4+d6)。注意 DH 表下 d4 沿 frame-3 z，
        // 零位时指向 -z，故 z = 675 − 1200 + 240 = −285（非 2115）。
        const double q[6] = {0, 0, 0, 0, 0, 0};
        const Pose p = kr210::forward(q);
        QVERIFY(qAbs(p.x - (350 + 1150 + 41)) < 1e-6);
        QVERIFY(qAbs(p.y) < 1e-6);
        QVERIFY(qAbs(p.z - (675 - 1200 + 240)) < 1e-6);
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
        QVERIFY(qAbs(p.z - (-285.0)) < 1e-3);
        QVERIFY(qAbs(p.a - 90.0) < 1e-6);
    }

    void jacobian_matchesNumerical()
    {
        const double q[6] = {0.3, -0.6, 0.8, 0.2, 0.5, -0.4};
        double J[6][6];
        kr210::jacobian(q, J);
        constexpr double h = 1e-7;
        const Pose f0 = kr210::forward(q);
        double R0[3][3], R0t[3][3];
        rotFromPose(f0, R0);
        trans3x3(R0, R0t);
        for (int col = 0; col < 6; ++col) {
            double qp[6], qm[6];
            std::copy(q, q + 6, qp);
            std::copy(q, q + 6, qm);
            qp[col] += h; qm[col] -= h;
            const Pose fp = kr210::forward(qp);
            const Pose fm = kr210::forward(qm);
            // 位置行：x/y/z 有限差分。
            const double dp[3] = {fp.x - fm.x, fp.y - fm.y, fp.z - fm.z};
            for (int row = 0; row < 3; ++row)
                QVERIFY2(qAbs(J[row][col] - dp[row] / (2 * h)) < 1e-5,
                         "jacobian position mismatch with numerical");
            // 姿态行：旋转矩阵有限差分 → 基系角速度。
            // 几何雅可比角部 = 基系角速度；欧拉角差分仅在零位姿时才等于角速度，
            // 非零位姿下二者由非正交矩阵 E(A,B,C) 联系（本测试 q 的位姿远离零）。
            double Rp[3][3], Rm[3][3];
            rotFromPose(fp, Rp);
            rotFromPose(fm, Rm);
            double Ap[3][3], Am[3][3];
            mul3x3(Rp, R0t, Ap);
            mul3x3(Rm, R0t, Am);
            // [ω]× = (R(q+h)R(q)ᵀ − R(q−h)R(q)ᵀ) / 2h
            const double w[3] = {
                (Ap[2][1] - Am[2][1]) / (2 * h),
                (Ap[0][2] - Am[0][2]) / (2 * h),
                (Ap[1][0] - Am[1][0]) / (2 * h),
            };
            for (int row = 0; row < 3; ++row)
                QVERIFY2(qAbs(J[3 + row][col] - w[row]) < 1e-5,
                         "jacobian angular mismatch with numerical");
        }
    }

    void solveDelta_roundTrip()
    {
        // 非奇异位形（腕部 q5=90° 远离腕部奇异，σ_min≈0.87）。亦为默认位形。
        const double q[6] = {0, -60.0 * M_PI / 180.0, 30.0 * M_PI / 180.0,
                             0, 90.0 * M_PI / 180.0, 0};
        const Pose dx{2.0, -1.0, 0.5, 0.5, -0.3, 0.4};   // mm, 度
        double dq[6];
        QVERIFY(kr210::solveDelta(q, dx, dq));
        for (int i = 0; i < 6; ++i)
            QVERIFY(std::isfinite(dq[i]));
        // 函数式往返：forward(q+dq) 应复现 Δx（阻尼小 + 非奇异位形下精确）。
        const Pose p0 = kr210::forward(q);
        double q1[6];
        for (int i = 0; i < 6; ++i)
            q1[i] = q[i] + dq[i];
        const Pose p1 = kr210::forward(q1);
        QVERIFY(qAbs((p1.x - p0.x) - dx.x) < 0.01);
        QVERIFY(qAbs((p1.y - p0.y) - dx.y) < 0.01);
        QVERIFY(qAbs((p1.z - p0.z) - dx.z) < 0.01);
        // 姿态：比较旋转角误差（R_cmd 与 R_actual）。不能用 A/B/C 逐分量差：
        // 同一小旋转在非零位姿下按欧拉角表示会被耦合放大（本位姿 B=60°）。
        double R0[3][3], R1[3][3], R0t[3][3], Ract[3][3], Rcmd[3][3];
        rotFromPose(p0, R0);
        rotFromPose(p1, R1);
        trans3x3(R0, R0t);
        mul3x3(R1, R0t, Ract);
        const Pose dpose{0, 0, 0, dx.a, dx.b, dx.c};
        rotFromPose(dpose, Rcmd);
        QVERIFY(rotAngleBetween(Rcmd, Ract) < 0.01);
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
