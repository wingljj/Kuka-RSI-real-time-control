#include <QtTest>
#include <cmath>
#include "core/PoseOps.h"

using poseops::Quat;

class TestPoseOps : public QObject
{
    Q_OBJECT
private slots:
    void abcRoundTrip()
    {
        for (const auto &abc : {std::array<double,3>{30,20,-40},
                                std::array<double,3>{-170,80,150},
                                std::array<double,3>{90,-60,0}}) {
            const Quat q = poseops::quatFromABC(abc[0], abc[1], abc[2]);
            double a, b, c;
            poseops::abcFromQuat(q, &a, &b, &c);
            // 欧拉非唯一（奇异/边界），比较旋转矩阵而非角度
            const Quat q2 = poseops::quatFromABC(a, b, c);
            const double dot = q.w*q2.w + q.x*q2.x + q.y*q2.y + q.z*q2.z;
            QVERIFY(qAbs(std::fabs(dot) - 1.0) < 1e-9);
        }
        // 奇异分支：B=±90（C=0 gauge）round-trip 必须保持旋转
        for (const auto &abc : {std::array<double,3>{30,90,0},   // 回归：旧实现提取 A=-30（dot 0.866）
                                std::array<double,3>{0,90,0},
                                std::array<double,3>{40,-90,25},
                                std::array<double,3>{-170,90,10}}) {
            const Quat q = poseops::quatFromABC(abc[0], abc[1], abc[2]);
            double a, b, c;
            poseops::abcFromQuat(q, &a, &b, &c);
            const Quat q2 = poseops::quatFromABC(a, b, c);
            const double dot = q.w*q2.w + q.x*q2.x + q.y*q2.y + q.z*q2.z;
            QVERIFY(qAbs(std::fabs(dot) - 1.0) < 1e-9);
        }
    }

    void quatError_singularTarget_givesSaneRotation()
    {
        // 目标 B=180, A/C=±180（奇异 + 边界）：误差必须是有限旋转，非逐轴跳变
        const Quat qA = poseops::quatFromABC(0, 60, 0);       // 默认位形附近
        const Quat qT = poseops::quatFromABC(-180, 180, -180); // 用户遇到的奇异目标
        const Quat qE = poseops::quatError(qT, qA);
        double rot[3];
        poseops::rotVecFromQuat(qE, rot);
        QVERIFY(std::isfinite(rot[0]) && std::isfinite(rot[1]) && std::isfinite(rot[2]));
        QVERIFY(std::sqrt(rot[0]*rot[0]+rot[1]*rot[1]+rot[2]*rot[2]) > 0.01);
        // 旋转向量 → 四元数 → 再取回，应一致（round-trip）
        Quat qE2 = poseops::quatFromRotVec(rot);
        const double dot = qE.w*qE2.w + qE.x*qE2.x + qE.y*qE2.y + qE.z*qE2.z;
        QVERIFY(qAbs(std::fabs(dot) - 1.0) < 1e-6);
    }

    void quatError_identity_returnsIdentity()
    {
        const Quat q = poseops::quatFromABC(45, -30, 10);
        const Quat qE = poseops::quatError(q, q);
        double rot[3];
        poseops::rotVecFromQuat(qE, rot);
        QVERIFY(qAbs(rot[0]) < 1e-9 && qAbs(rot[1]) < 1e-9 && qAbs(rot[2]) < 1e-9);
    }

    void quatError_knownRotation()
    {
        // 目标 = 实际绕 X 转 +90°：旋转向量应为 [90°,0,0]（rad=π/2）
        const Quat qA = poseops::quatFromABC(0, 0, 0);
        const Quat qT = poseops::quatFromABC(0, 0, 90);
        const Quat qE = poseops::quatError(qT, qA);
        double rot[3];
        poseops::rotVecFromQuat(qE, rot);
        QVERIFY(qAbs(rot[0] - 3.14159265358979323846 / 2) < 1e-6);
        QVERIFY(qAbs(rot[1]) < 1e-6 && qAbs(rot[2]) < 1e-6);
    }

    void quatSlerp_endpointsExact()
    {
        const Quat a = poseops::quatFromABC(30, 20, -40);
        const Quat b = poseops::quatFromABC(-170, 80, 150);
        const Quat s0 = poseops::quatSlerp(a, b, 0.0);
        const Quat s1 = poseops::quatSlerp(a, b, 1.0);
        QCOMPARE(s0.w, a.w); QCOMPARE(s0.x, a.x); QCOMPARE(s0.y, a.y); QCOMPARE(s0.z, a.z);
        QCOMPARE(s1.w, b.w); QCOMPARE(s1.x, b.x); QCOMPARE(s1.y, b.y); QCOMPARE(s1.z, b.z);
    }

    void quatSlerp_midpointOf90deg()
    {
        // 绕 Z 90° 的 Slerp 中点 = 45°（角度随 t 线性）
        const Quat q0 = poseops::quatFromABC(0, 0, 0);
        const Quat q1 = poseops::quatFromABC(0, 0, 90);
        const Quat qm = poseops::quatSlerp(q0, q1, 0.5);
        double a, b, c;
        poseops::abcFromQuat(qm, &a, &b, &c);
        QVERIFY(qAbs(a) < 1e-9 && qAbs(b) < 1e-9);
        QVERIFY(qAbs(c - 45.0) < 1e-9);
    }

    void quatSlerp_shortestArc()
    {
        // 179° → -179°（最短差 2°）：中点走短弧 1°，而非长弧 179°
        const Quat q0 = poseops::quatFromABC(179, 0, 0);
        const Quat q1 = poseops::quatFromABC(-179, 0, 0);
        const Quat qm = poseops::quatSlerp(q0, q1, 0.5);
        double rot[3];
        poseops::rotVecFromQuat(poseops::quatError(qm, q0), rot);
        const double angleDeg = std::sqrt(rot[0]*rot[0]+rot[1]*rot[1]+rot[2]*rot[2])
                                * 180.0 / 3.14159265358979323846;
        QVERIFY(qAbs(angleDeg - 1.0) < 1e-6);
        // 对 q1 取反（同一旋转）结果旋转不变——最短弧不依赖符号
        const Quat q1n{-q1.w, -q1.x, -q1.y, -q1.z};
        const Quat qm2 = poseops::quatSlerp(q0, q1n, 0.5);
        double a1, b1, c1, a2, b2, c2;
        poseops::abcFromQuat(qm, &a1, &b1, &c1);
        poseops::abcFromQuat(qm2, &a2, &b2, &c2);
        QVERIFY(qAbs(a1 - a2) < 1e-6 && qAbs(b1 - b2) < 1e-6 && qAbs(c1 - c2) < 1e-6);
    }

    void quatSlerp_angleProportionalToT()
    {
        // 角速度恒定：t=0.25 处相对起点的转角 = 总转角 × 0.25
        const Quat q0 = poseops::quatFromABC(0, 0, 0);
        const Quat q1 = poseops::quatFromABC(30, -20, 10);
        double rot[3];
        poseops::rotVecFromQuat(poseops::quatError(q1, q0), rot);
        const double total = std::sqrt(rot[0]*rot[0]+rot[1]*rot[1]+rot[2]*rot[2]);
        const Quat qH = poseops::quatSlerp(q0, q1, 0.25);
        poseops::rotVecFromQuat(poseops::quatError(qH, q0), rot);
        const double partial = std::sqrt(rot[0]*rot[0]+rot[1]*rot[1]+rot[2]*rot[2]);
        QVERIFY(qAbs(partial - 0.25 * total) < 1e-6);
    }

    void invEulerRate_identity()
    {
        // E·(E⁻¹·ω) = ω（非奇异位形）
        const double a = 30, b = 20, c = -40;
        double invE[3][3];
        QVERIFY(poseops::invEulerRate(a, b, c, invE));
        const double ar = a * M_PI/180, br = b * M_PI/180, cr = c * M_PI/180;
        const double sa = std::sin(ar), ca = std::cos(ar);
        const double sb = std::sin(br), cb = std::cos(br);
        const double E[3][3] = {
            {0, -sa, ca*cb},
            {0,  ca, sa*cb},
            {1,  0,  -sb},
        };
        const double w[3] = {0.1, -0.2, 0.3};   // ω
        double abc[3] = {0,0,0};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                abc[r] += invE[r][c] * w[c];
        for (int r = 0; r < 3; ++r) {
            double back = 0;
            for (int c = 0; c < 3; ++c)
                back += E[r][c] * abc[c];
            QVERIFY(qAbs(back - w[r]) < 1e-9);
        }
    }

    void invEulerRate_singular_returnsFalse()
    {
        double m[3][3];
        QVERIFY(!poseops::invEulerRate(0, 90, 0, m));   // B=90 严格奇异
        // 近奇异（B=85，|cosB|≈0.087 < 0.1）：E⁻¹ 放大无界，必须拒绝以回退一阶近似
        QVERIFY(!poseops::invEulerRate(0, 85, 0, m));
        // 安全区边界外一档（B=80，|cosB|≈0.17 > 0.1）：仍可用
        QVERIFY(poseops::invEulerRate(0, 80, 0, m));
    }

    void errorPoseDeg_singularTarget_rotationVector()
    {
        // 目标 (-180,180,-180) vs 实际 (0,60,0)：SO(3) 最短旋转范数 ≈ 60°，而非
        // 逐轴欧拉差范数 ≈ 293°（修复前显示层）。
        const Pose actual{0, 0, 0, 0, 60, 0};
        const Pose target{0, 0, 0, -180, 180, -180};
        const Pose e = poseops::errorPoseDeg(target, actual);
        QVERIFY(std::isfinite(e.a) && std::isfinite(e.b) && std::isfinite(e.c));
        const double norm = std::sqrt(e.a*e.a + e.b*e.b + e.c*e.c);
        QVERIFY(qAbs(norm - 60.0) < 1.0);   // 真实 SO(3) 误差 60°（±容差）
    }
};
QTEST_MAIN(TestPoseOps)
#include "test_pose_ops.moc"
