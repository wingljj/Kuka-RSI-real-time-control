#include <QtTest>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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

// 两组关节角的最大逐关节差，角度按 2π 同余比较。
// 为什么要同余：RL 的 solve 成功前会 normalize(q)，把关节角折回 (-π, π]，
// 所以「同一个分支」的解可能与种子相差整整 2π —— 直接相减会把它误判成跳变。
double maxJointDiffMod2Pi(const double a[6], const double b[6])
{
    double worst = 0.0;
    for (int i = 0; i < 6; ++i) {
        double d = std::fmod(std::abs(a[i] - b[i]), 2.0 * M_PI);
        if (d > M_PI)
            d = 2.0 * M_PI - d;
        worst = std::max(worst, d);
    }
    return worst;
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

    // ---- seedRad 路径（模拟器实际走的那条）----

    void inverse_seed_selectsSeedBranch()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // 腕翻分支：q3+π / -q4 / q5+π 给出**同一个** TCP 位姿，是货真价实的
        // 另一个 IK 解。两个种子各自是自己分支上的精确解，因此本例与 RL 内部的
        // 随机重启无关，结果确定。
        const double qA[6] = {0.5, -0.4, -1.6, 0.3, -0.5, 0.7};
        const double qB[6] = {0.5, -0.4, -1.6, 0.3 + M_PI, 0.5, 0.7 + M_PI};
        const Pose pose = rlk::forward(qA);
        const Pose poseB = rlk::forward(qB);
        QVERIFY2(std::abs(poseB.x - pose.x) < 1e-6 && std::abs(poseB.y - pose.y) < 1e-6
                     && std::abs(poseB.z - pose.z) < 1e-6,
                 "腕翻分支前提不成立：两个位形的 TCP 位置不同");

        // 同一目标位姿，给不同种子 → 落回各自的分支。这是 inverse() 真正的行为
        // 主张：种子顺序 = 分支优先级，把当前 q 排第一就等于「不许翻分支」。
        // 模拟器靠这条性质保证 AIPos 逐帧连续；否则位姿连续而关节角整段跳 π，
        // --joint-limits 一夹住就变成 RIst 跳变（主机 exceedsPhysicalJump 判 stale）。
        const auto budget = rlk::solveBudgetForCycle(12.0);
        double outA[6], outB[6];
        QVERIFY(rlk::inverse(pose, outA, qA, budget));
        QVERIFY(rlk::inverse(pose, outB, qB, budget));

        QVERIFY2(maxJointDiffMod2Pi(outA, qA) < 1e-3,
                 qPrintable(QStringLiteral("seed A 分支未保持：%1 rad")
                                .arg(maxJointDiffMod2Pi(outA, qA))));
        QVERIFY2(maxJointDiffMod2Pi(outB, qB) < 1e-3,
                 qPrintable(QStringLiteral("seed B 分支未保持：%1 rad")
                                .arg(maxJointDiffMod2Pi(outB, qB))));
        // 两解确实是不同分支（差 ≈ π），否则上面两条断言可能只是碰巧同解。
        QVERIFY2(maxJointDiffMod2Pi(outA, outB) > 3.0,
                 qPrintable(QStringLiteral("两个分支没分开：%1 rad")
                                .arg(maxJointDiffMod2Pi(outA, outB))));
    }

    void inverse_seed_tracksIncrementsWithinCycleBudget()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // 模拟器的真实工况：每帧 0.6mm 增量，上一帧的 q 当种子，预算按
        // --cycle-ms 4（最紧的现实周期）推导。跑 200 帧看三件事：
        // 帧内解得出、关节连续、机器人真的走了 120mm（不是靠全部解失败装干净）。
        const auto budget = rlk::solveBudgetForCycle(4.0);
        double q[6] = {0.5, -0.4, -1.6, 0.3, -0.5, 0.7};
        Pose pose = rlk::forward(q);
        const double x0 = pose.x;
        int solved = 0;
        double maxJump = 0.0;
        for (int step = 0; step < 200; ++step) {
            Pose target = pose;
            target.x += 0.6;
            double qNew[6];
            if (!rlk::inverse(target, qNew, q, budget))
                continue;               // 与模拟器一致：解不出就保持旧 q
            ++solved;
            maxJump = std::max(maxJump, maxJointDiffMod2Pi(qNew, q));
            std::copy(qNew, qNew + 6, q);
            pose = rlk::forward(q);
        }
        // 不断 200/200：预算是硬上限，机器卡顿时放弃求解是设计行为而非回归
        // （实测 0 失败，p99 收敛耗时 82µs，只占热路径份额 1ms 的 8%）。
        QVERIFY2(solved >= 190, qPrintable(QStringLiteral("solved %1/200").arg(solved)));
        QVERIFY2(maxJump < 0.05,
                 qPrintable(QStringLiteral("关节跳变 %1 rad（分支翻转？）").arg(maxJump)));
        QVERIFY2(pose.x - x0 > 100.0,
                 qPrintable(QStringLiteral("只走了 %1 mm，机器人没动").arg(pose.x - x0)));
    }

    void inverse_seed_outOfLimits_isClamped()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // 种子越限必须被夹回限位内，而不是原样交给 solve：RL 的成功判定要求
        // isValid(q)，从越限起点出发很可能整段预算白烧。
        // 这里把 joint0 推到限位外 +10 rad，夹回来恰好等于目标自己的 joint0，
        // 于是「夹住了」的观测后果很锐利：一次迭代就还原出目标位形本身。
        const kr210::JointLimits &lim = rlk::limits();
        const double qEdge[6] = {lim.max[0], -0.4, -1.6, 0.3, -0.5, 0.7};
        const Pose pose = rlk::forward(qEdge);
        double seed[6];
        std::copy(qEdge, qEdge + 6, seed);
        seed[0] = qEdge[0] + 10.0;

        double out[6];
        QVERIFY(rlk::inverse(pose, out, seed, rlk::solveBudgetForCycle(12.0)));
        QVERIFY2(maxJointDiffMod2Pi(out, qEdge) < 1e-3,
                 qPrintable(QStringLiteral("越限种子没被夹到限位：偏差 %1 rad")
                                .arg(maxJointDiffMod2Pi(out, qEdge))));
        for (int i = 0; i < 6; ++i) {
            QVERIFY2(out[i] >= lim.min[i] - 1e-9 && out[i] <= lim.max[i] + 1e-9,
                     qPrintable(QStringLiteral("j%1 = %2 越限").arg(i).arg(out[i])));
        }
    }

    void inverse_seed_nonFinite_isIgnored()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // clampToLimits 用 std::max/min，NaN 会原样穿过（max(NaN, lo) == NaN），
        // 所以非有限种子必须在入口就丢掉，否则 NaN 会一路传到 qRad 再传到 RIst。
        // 生产里 ctx.q 恒在限位内，这条只是把「坏种子不污染输出」钉死。
        const double qRef[6] = {0.5, -0.4, -1.6, 0.3, -0.5, 0.7};
        const Pose pose = rlk::forward(qRef);
        const double bad[3][6] = {
            {0.5, -0.4, std::nan(""), 0.3, -0.5, 0.7},
            {std::numeric_limits<double>::infinity(), -0.4, -1.6, 0.3, -0.5, 0.7},
            {std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan(""), std::nan("")},
        };
        for (int k = 0; k < 3; ++k) {
            double out[6] = {0, 0, 0, 0, 0, 0};
            // 种子被丢弃后固定种子兜底：这条路径不在发帧回路里，用默认预算。
            QVERIFY2(rlk::inverse(pose, out, bad[k]), qPrintable(QStringLiteral("case %1 未解出").arg(k)));
            for (int i = 0; i < 6; ++i) {
                QVERIFY2(std::isfinite(out[i]),
                         qPrintable(QStringLiteral("case %1 j%2 非有限").arg(k).arg(i)));
            }
        }
    }

    // ---- 总预算：模拟器发帧节拍的硬约束 ----

    void solveBudget_staysBelowOneCycle()
    {
        // 「一次逆解撑不爆一个发帧周期」必须由代码保证。--cycle-ms 是用户可设的
        // （真机 IPO 常用 4ms），所以这里断的是函数关系而不是某个具体毫秒数：
        // 任何现实周期下预算都严格小于周期，且还给收包/正解/UDP/渲染留了一半。
        for (double cycleMs : {0.5, 1.0, 4.0, 8.0, 12.0, 100.0}) {
            const auto budget = rlk::solveBudgetForCycle(cycleMs);
            const double budgetMs = std::chrono::duration<double, std::milli>(budget).count();
            QVERIFY2(budgetMs < cycleMs,
                     qPrintable(QStringLiteral("cycle %1ms → budget %2ms").arg(cycleMs).arg(budgetMs)));
            QVERIFY2(budgetMs >= 0.19,
                     qPrintable(QStringLiteral("cycle %1ms → budget %2ms 过小").arg(cycleMs).arg(budgetMs)));
        }
    }

    void inverse_budget_isHardCeiling()
    {
        QVERIFY(rlk::loadModel(modelPath()));
        // 最坏情况：目标不可达 → 每个种子都烧完自己的份额。这是工作空间边界处
        // 的稳态而非偶发，所以它必须仍然被总预算钉住——预算是「一次调用」的上限，
        // 不是「每种子」的上限，加多少个种子都不会破界。
        const Pose far{5000.0, 0.0, 5000.0, 0.0, 0.0, 0.0};
        for (int ms : {2, 6, 20}) {
            double q[6] = {0, 0, 0, 0, 0, 0};
            const double seed[6] = {0.5, -0.4, -1.6, 0.3, -0.5, 0.7};
            QElapsedTimer t;
            t.start();
            QVERIFY(!rlk::inverse(far, q, seed, std::chrono::milliseconds(ms)));
            const double elapsedMs = t.nsecsElapsed() / 1.0e6;
            // 余量给 OS 调度和 RL 的「迭代末尾才查时钟」尾巴（实测超出 ~15µs）。
            QVERIFY2(elapsedMs < ms + 2.0,
                     qPrintable(QStringLiteral("budget %1ms 实耗 %2ms").arg(ms).arg(elapsedMs)));
        }
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
