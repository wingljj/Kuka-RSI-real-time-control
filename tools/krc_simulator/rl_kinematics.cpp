#include "tools/krc_simulator/rl_kinematics.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/Joint.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>

#include "core/PoseOps.h"

namespace {

const char* kDefaultModelPath = "D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml";

// 静态单例：RL 对象生命周期与进程一致。模拟器单线程调用，无需加锁。
std::shared_ptr<rl::mdl::Model> g_model;
std::shared_ptr<rl::mdl::Kinematic> g_kinematic;
kr210::JointLimits g_limits{};
bool g_loaded = false;

// qRad[6]（rad）→ RL 位置向量。
rl::math::Vector toRlQ(const double qRad[6])
{
    rl::math::Vector q(6);
    for (int i = 0; i < 6; ++i)
        q(i) = qRad[i];
    return q;
}

// RL 位姿（米 + 四元数）→ Pose（mm；A/B/C 度，KUKA ZYX）。
Pose toPoseMm(const rl::math::Transform &t)
{
    const Eigen::Vector3d p = t.translation();
    const Eigen::Quaterniond q(t.rotation());
    Pose pose;
    pose.x = p.x() * 1000.0;
    pose.y = p.y() * 1000.0;
    pose.z = p.z() * 1000.0;
    const poseops::Quat quat{q.w(), q.x(), q.y(), q.z()};
    poseops::abcFromQuat(quat, &pose.a, &pose.b, &pose.c);
    return pose;
}

// Pose（mm；A/B/C 度）→ RL Transform（米 + 旋转矩阵）。
rl::math::Transform toRlTransform(const Pose &target)
{
    const poseops::Quat quat = poseops::quatFromABC(target.a, target.b, target.c);
    rl::math::Transform t;
    t.translation() = Eigen::Vector3d(target.x * 1e-3, target.y * 1e-3, target.z * 1e-3);
    t.linear() = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
    return t;
}

// 热路径种子（调用方的当前关节角）分到的预算份额：总预算的 1/kHotSeedShare。
// 为什么单独给它一份而不是所有种子均分：它是唯一被指望能成功的种子（增量目标
// 从当前位形出发一两次迭代就收敛），后面 8 个固定种子在增量工况下本来就是长尾。
// 均分会让它在小周期下被挤到几百微秒——--cycle-ms 4 时是 2ms/9 ≈ 0.22ms，只有
// 实测收敛耗时的 2 倍余量；给它一半则有 10 倍余量，剩下的固定种子再平分余额。
//
// 把预算从原来的「每种子 120ms」压到这个量级**是有代价的**，代价不在增量路径上：
//   · 增量目标（当前 q 作种子，0.6mm/帧）：p50 43µs、p99 82µs，250 个随机位形里
//     249 个收敛——即使 --cycle-ms 4（热路径份额 1ms）也有 12 倍余量，可达范围
//     与压预算之前一致。
//   · 绝对目标（无种子，只能靠固定种子兜底）：可达率随总预算掉得很明显。用 FK 从
//     250 个随机合法 q 生成的、可达性有证明的位姿实测：总预算 50ms → 250/250
//     (100%)、9ms → 213/250 (85%)、6ms（--cycle-ms 12）→ 184/250 (74%)、
//     2ms（--cycle-ms 4）→ 171/250 (68%)。
// 之所以能接受：模拟器两处调用都传当前 q（走增量路径），绝对目标只有单测在用，
// 而单测走的是 kDefaultSolveBudget（50ms，100% 可达）。发帧回路里兜底种子解不出
// 的后果是这一帧保持旧 q（RIst 停一帧），而不是发帧延迟——这个取舍是刻意的：
// 节拍崩掉会被上位机看门狗误判成断流，少动一帧不会。
//
// 注：fb435d9 的报告称压预算「没有损失可达范围」，证据是修复前后机器人都停在
// X=326.600。那个实验测的是增量路径，当前 q 作种子几十微秒就收敛、预算根本不
// 生效，所以它在原理上就检测不到上面这项损失，不能当作无代价的证据。
constexpr std::size_t kHotSeedShare = 2;

// 种子限制到关节限位内：solve 的成功判定要求 isValid(q)（越限解会被拒绝）。
rl::math::Vector clampToLimits(const rl::math::Vector &q)
{
    rl::math::Vector out = q;
    const std::size_t n = std::min<std::size_t>(6, g_kinematic->getJoints());
    for (std::size_t i = 0; i < n; ++i) {
        const rl::mdl::Joint *joint = g_kinematic->getJoint(i);
        out(i) = std::max(out(i), joint->getMinimum()(0));
        out(i) = std::min(out(i), joint->getMaximum()(0));
    }
    return out;
}

} // namespace

namespace rlk {

bool loadModel(const std::string &rlmdlPath)
{
    const std::string path = rlmdlPath.empty() ? kDefaultModelPath : rlmdlPath;
    try {
        rl::mdl::XmlFactory factory;
        std::shared_ptr<rl::mdl::Model> model = factory.create(path);
        if (!model || model->getJoints() == 0) {
            g_loaded = false;
            return false;
        }
        std::shared_ptr<rl::mdl::Kinematic> kinematic =
            std::dynamic_pointer_cast<rl::mdl::Kinematic>(model);
        if (!kinematic) {
            g_loaded = false;
            return false;
        }

        g_model = model;
        g_kinematic = kinematic;

        // 关节限位（模型内部弧度；XML 的度由 XmlFactory 自动转换）。
        kr210::JointLimits limits{};
        const std::size_t n = std::min<std::size_t>(6, kinematic->getJoints());
        for (std::size_t i = 0; i < n; ++i) {
            const rl::mdl::Joint *joint = kinematic->getJoint(i);
            limits.min[i] = joint->getMinimum()(0);
            limits.max[i] = joint->getMaximum()(0);
        }
        g_limits = limits;
        g_loaded = true;
        return true;
    } catch (const std::exception &) {
        // 文件不存在/解析失败（libxml2 异常）。
        g_model.reset();
        g_kinematic.reset();
        g_loaded = false;
        return false;
    }
}

Pose forward(const double qRad[6])
{
    if (!g_loaded || !g_kinematic)
        return Pose{};
    g_kinematic->setPosition(toRlQ(qRad));
    g_kinematic->forwardPosition();
    return toPoseMm(g_kinematic->getOperationalPosition(0));
}

std::chrono::nanoseconds solveBudgetForCycle(double cycleMs)
{
    const double halfCycleUs = cycleMs * 1000.0 / 2.0;
    return std::chrono::microseconds(
        static_cast<long long>(std::max(200.0, halfCycleUs)));
}

bool inverse(const Pose &target, double qRad[6], const double *seedRad,
             std::chrono::nanoseconds budget)
{
    if (!g_loaded || !g_kinematic)
        return false;

    // 总截止时间在进入种子循环前就定死。约束的真实形态是「一次逆解不许超过多久」
    // ——它与种子数量无关，所以这里表达成一个截止时刻，而不是「每种子 N 毫秒 ×
    // 种子数」那种要靠注释里做算术才成立的界。种子数以后怎么增删都不会破界。
    const std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::now() + budget;

    const rl::math::Transform goal = toRlTransform(target);

    // 种子：当前关节角（若给）→ home → q=0 → home 逐关节微扰
    //（确定性；RL 内部还有随机重启兜底）。
    const rl::math::Vector home = clampToLimits(g_kinematic->getHomePosition());
    const rl::math::Vector zero = rl::math::Vector::Zero(g_kinematic->getDofPosition());

    std::vector<rl::math::Vector> seeds;
    seeds.reserve(9);
    // 当前关节角必须排第一：它是唯一与目标距离不随运动增长的种子，因此几乎总在
    // 首个种子上就收敛——固定种子那 8 次昂贵的重试根本不会被执行到；顺序同时决定
    // 选中哪个 IK 分支，排第一就等于锁在当前分支上不翻（详见头文件）。
    //
    // 非有限种子（NaN/Inf）直接丢掉：clampToLimits 用 std::max/min，NaN 会原样穿过
    // （max(NaN, lo) 返回 NaN），带进 solve 就是一次纯浪费预算的迭代，更坏的情况是
    // 解出 NaN 的 q 再正解成 NaN 的 RIst 发给上位机。生产里 ctx.q 恒被夹在关节限位
    // 内，这里只是不让一个坏种子有机会污染输出。
    if (seedRad && std::isfinite(seedRad[0]) && std::isfinite(seedRad[1])
        && std::isfinite(seedRad[2]) && std::isfinite(seedRad[3])
        && std::isfinite(seedRad[4]) && std::isfinite(seedRad[5]))
        seeds.push_back(clampToLimits(toRlQ(seedRad)));
    const bool hotSeed = !seeds.empty();
    seeds.push_back(home);
    seeds.push_back(zero);
    for (std::size_t i = 0; i < 6 && i < g_kinematic->getDofPosition(); ++i) {
        rl::math::Vector s = home;
        s(i) += 0.1;
        seeds.push_back(clampToLimits(s));
    }

    rl::mdl::JacobianInverseKinematics ik(g_kinematic.get());
    ik.setEpsilon(1e-9);
    ik.addGoal(goal, 0);

    for (std::size_t i = 0; i < seeds.size(); ++i) {
        // 每个种子的 duration 从「到截止时刻还剩多少」现算：前一个种子超支多少，
        // 后面的份额自动少多少，总耗时因此被 deadline 钉住（RL 的 solve 在每次
        // 雅可比迭代末尾才查 duration，所以最后一个种子可能多跑一次迭代——数十
        // 微秒的固定尾巴，与种子数无关）。
        const std::chrono::nanoseconds remain = deadline - std::chrono::steady_clock::now();
        if (remain <= std::chrono::nanoseconds::zero())
            break;
        const std::size_t share = (i == 0 && hotSeed)
                                      ? kHotSeedShare
                                      : (seeds.size() - i);
        ik.setDuration(remain / static_cast<std::chrono::nanoseconds::rep>(share));

        g_kinematic->setPosition(seeds[i]);
        if (ik.solve()) {
            // solve 成功时已 normalize 并 isValid 检查（解在限位内）。
            const rl::math::Vector q = g_kinematic->getPosition();
            for (std::size_t k = 0; k < 6; ++k)
                qRad[k] = q(k);
            return true;
        }
    }
    return false;
}

const kr210::JointLimits &limits()
{
    return g_limits;
}

Skeleton skeleton()
{
    Skeleton s;
    if (!g_loaded || !g_model)
        return s;

    for (std::size_t i = 0; i < g_model->getBodies(); ++i) {
        const rl::math::Transform &t = g_model->getBodyFrame(i);
        BodyPose p;
        p.x = t.translation().x() * 1000.0;
        p.y = t.translation().y() * 1000.0;
        p.z = t.translation().z() * 1000.0;
        const Eigen::Quaterniond q(t.rotation());
        p.qw = q.w(); p.qx = q.x(); p.qy = q.y(); p.qz = q.z();
        s.bodies.push_back(p);
    }

    // TCP = operational frame 0（与 forward() 同源）
    const rl::math::Transform &tcp = g_kinematic->getOperationalPosition(0);
    BodyPose &tp = s.tcp;
    tp.x = tcp.translation().x() * 1000.0;
    tp.y = tcp.translation().y() * 1000.0;
    tp.z = tcp.translation().z() * 1000.0;
    const Eigen::Quaterniond qt(tcp.rotation());
    tp.qw = qt.w(); tp.qx = qt.x(); tp.qy = qt.y(); tp.qz = qt.z();
    return s;
}

} // namespace rlk
