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
        QVERIFY(!poseops::invEulerRate(0, 90, 0, m));   // B=90 奇异
    }
};
QTEST_MAIN(TestPoseOps)
#include "test_pose_ops.moc"
