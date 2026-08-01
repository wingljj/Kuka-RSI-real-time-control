#include <QtTest>
#include <cmath>
#include <string>

#include "core/PoseOps.h"
#include "tools/krc_simulator/rl_kinematics.h"

// 模型路径：默认 Comau Racer 7-1.4（RL-T1 验证过的模型），可用环境变量 RLMDL_PATH 覆盖。
namespace {

std::string modelPath()
{
    const QByteArray env = qgetenv("RLMDL_PATH");
    return env.isEmpty()
        ? "D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml"
        : env.toStdString();
}

// 四元数点积（≈1 表示两姿态几乎相同）。
double quatDot(const poseops::Quat &a, const poseops::Quat &b)
{
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

// 往返校验：inverse(forward(q0)) 解出的 q 再 forward，位姿必须 ≈ forward(q0)。
// 只断位姿（RL 腕部零空间自运动会让 q3/q5 漂移 ±0.19 rad，不断关节角）。
void checkRoundTrip(const double q0[6])
{
    const Pose target = rlk::forward(q0);
    double q[6] = {0, 0, 0, 0, 0, 0};
    QVERIFY(rlk::inverse(target, q));
    const Pose back = rlk::forward(q);

    QVERIFY2(std::abs(back.x - target.x) < 0.1,
             qPrintable(QStringLiteral("dx %1 mm").arg(back.x - target.x)));
    QVERIFY2(std::abs(back.y - target.y) < 0.1,
             qPrintable(QStringLiteral("dy %1 mm").arg(back.y - target.y)));
    QVERIFY2(std::abs(back.z - target.z) < 0.1,
             qPrintable(QStringLiteral("dz %1 mm").arg(back.z - target.z)));

    // 姿态：A/B/C → 四元数点积 ≈ 1（≈ 0.003° 余量）。
    const poseops::Quat qTarget = poseops::quatFromABC(target.a, target.b, target.c);
    const poseops::Quat qBack = poseops::quatFromABC(back.a, back.b, back.c);
    const double dot = quatDot(qTarget, qBack);
    QVERIFY2(dot > 1.0 - 1e-9, qPrintable(QStringLiteral("quat dot %1").arg(dot)));
}

} // namespace

class TestRlKin : public QObject
{
    Q_OBJECT
private slots:
    void loadModel_ok()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // 不存在的文件 → false；失败后重新加载成功（幂等、失败清状态）。
        QVERIFY(!rlk::loadModel("E:/no/such/model.rlmdl.xml"));
        QVERIFY(rlk::loadModel(modelPath()));
    }

    void forward_homePose()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // home q = [0, 0, -90°, 0, 0, 0]（RL-T1 验证值）：
        //   位置 (934, 0, 1150) mm；姿态四元数 (0.70711, 0, 0.70711, 0)
        //   = 绕 Y 轴 +90° → ZYX 欧拉 (a, b, c) = (0, 90, 0) 度。
        const double q[6] = {0, 0, -M_PI / 2.0, 0, 0, 0};
        const Pose p = rlk::forward(q);
        QVERIFY(std::abs(p.x - 934.0) < 1e-3);
        QVERIFY(std::abs(p.y - 0.0) < 1e-3);
        QVERIFY(std::abs(p.z - 1150.0) < 1e-3);
        QVERIFY(std::abs(p.a - 0.0) < 1e-3);
        QVERIFY(std::abs(p.b - 90.0) < 1e-3);
        QVERIFY(std::abs(p.c - 0.0) < 1e-3);
    }

    void forward_zeroPose()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // q 全 0（RL-T1 验证值）：位置 (20, 0, 1804) mm；姿态恒等 → (0, 0, 0) 度。
        const double q[6] = {0, 0, 0, 0, 0, 0};
        const Pose p = rlk::forward(q);
        QVERIFY(std::abs(p.x - 20.0) < 1e-3);
        QVERIFY(std::abs(p.y - 0.0) < 1e-3);
        QVERIFY(std::abs(p.z - 1804.0) < 1e-3);
        QVERIFY(std::abs(p.a - 0.0) < 1e-3);
        QVERIFY(std::abs(p.b - 0.0) < 1e-3);
        QVERIFY(std::abs(p.c - 0.0) < 1e-3);
    }

    void inverse_roundTrip()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // 位姿精确往返：home 位形 + 一个非平凡关节角位形。
        const double qHome[6] = {0, 0, -M_PI / 2.0, 0, 0, 0};
        checkRoundTrip(qHome);
        const double qGen[6] = {0.5, -0.4, -1.6, 0.3, -0.5, 0.7};
        checkRoundTrip(qGen);
    }

    void inverse_unreachable_returnsFalse()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // 7 m 外——远超工作空间（最大臂展 ≈ 1.8 m），必 false。
        const Pose far{5000.0, 0.0, 5000.0, 0.0, 0.0, 0.0};
        double q[6] = {0, 0, 0, 0, 0, 0};
        QVERIFY(!rlk::inverse(far, q));
    }

    void limits_sane()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        const kr210::JointLimits &lim = rlk::limits();
        for (int i = 0; i < 6; ++i) {
            QVERIFY2(lim.min[i] < lim.max[i],
                     qPrintable(QStringLiteral("joint %1 min %2 >= max %3")
                                    .arg(i).arg(lim.min[i]).arg(lim.max[i])));
        }
        // RL-T1 验证过的限位（度 → 弧度）抽查，防单位/解析回归。
        const double d2r = M_PI / 180.0;
        QVERIFY(std::abs(lim.min[0] + 165.0 * d2r) < 1e-9);
        QVERIFY(std::abs(lim.max[0] - 165.0 * d2r) < 1e-9);
        QVERIFY(std::abs(lim.min[2] + 170.0 * d2r) < 1e-9);   // j2 [-170, 0]°
        QVERIFY(std::abs(lim.max[2] - 0.0) < 1e-9);
        QVERIFY(std::abs(lim.min[5] + 2700.0 * d2r) < 1e-9);  // j5 ±2700°
        QVERIFY(std::abs(lim.max[5] - 2700.0 * d2r) < 1e-9);
    }

    void skeleton_size()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        rlk::forward((const double[6]){0, 0, 0, 0, 0, 0});  // 触发 forwardPosition()
        const rlk::Skeleton s = rlk::skeleton();
        QCOMPARE(int(s.bodies.size()), 7);  // Comau Racer 7-1.4: body0..body6
    }

    void skeleton_body0_isOrigin()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        rlk::forward((const double[6]){0, 0, 0, 0, 0, 0});
        const rlk::Skeleton s = rlk::skeleton();
        QVERIFY(std::abs(s.bodies[0].x) < 1e-6);
        QVERIFY(std::abs(s.bodies[0].y) < 1e-6);
        QVERIFY(std::abs(s.bodies[0].z) < 1e-6);  // fixed0 = identity
    }

    void skeleton_body1_atQzero()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        rlk::forward((const double[6]){0, 0, 0, 0, 0, 0});
        const rlk::Skeleton s = rlk::skeleton();
        // body1 = fixed1 (+0.43 z) + joint0(identity, q=0) + fixed2 (Rx(-90°), trans (150,0,0))
        // → world (150, 0, 430) mm
        QVERIFY2(std::abs(s.bodies[1].x - 150.0) < 2.0,
                 qPrintable(QStringLiteral("body1.x %1").arg(s.bodies[1].x)));
        QVERIFY2(std::abs(s.bodies[1].y - 0.0) < 2.0,
                 qPrintable(QStringLiteral("body1.y %1").arg(s.bodies[1].y)));
        QVERIFY2(std::abs(s.bodies[1].z - 430.0) < 2.0,
                 qPrintable(QStringLiteral("body1.z %1").arg(s.bodies[1].z)));
    }

    void skeleton_lastBody_eq_tcp()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        const double qHome[6] = {0, 0, -M_PI / 2.0, 0, 0, 0};
        rlk::forward(qHome);
        const rlk::Skeleton s = rlk::skeleton();
        // body6 (fixed9 = identity) = frame8 = operational frame 0 = TCP
        const rlk::BodyPose &b6 = s.bodies[6];
        QVERIFY(std::abs(b6.x - s.tcp.x) < 1e-3);
        QVERIFY(std::abs(b6.y - s.tcp.y) < 1e-3);
        QVERIFY(std::abs(b6.z - s.tcp.z) < 1e-3);
    }
};

QTEST_MAIN(TestRlKin)
#include "test_rl_kinematics.moc"
