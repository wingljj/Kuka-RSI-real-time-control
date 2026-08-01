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

bool inverse(const Pose &target, double qRad[6])
{
    if (!g_loaded || !g_kinematic)
        return false;

    const rl::math::Transform goal = toRlTransform(target);

    // 种子：home → q=0 → home 逐关节微扰（确定性；RL 内部还有随机重启兜底）。
    const rl::math::Vector home = clampToLimits(g_kinematic->getHomePosition());
    const rl::math::Vector zero = rl::math::Vector::Zero(g_kinematic->getDofPosition());

    std::vector<rl::math::Vector> seeds;
    seeds.reserve(8);
    seeds.push_back(home);
    seeds.push_back(zero);
    for (std::size_t i = 0; i < 6 && i < g_kinematic->getDofPosition(); ++i) {
        rl::math::Vector s = home;
        s(i) += 0.1;
        seeds.push_back(clampToLimits(s));
    }

    rl::mdl::JacobianInverseKinematics ik(g_kinematic.get());
    ik.setDuration(std::chrono::milliseconds(120));
    ik.setEpsilon(1e-9);
    ik.addGoal(goal, 0);

    for (const rl::math::Vector &seed : seeds) {
        g_kinematic->setPosition(seed);
        if (ik.solve()) {
            // solve 成功时已 normalize 并 isValid 检查（解在限位内）。
            const rl::math::Vector q = g_kinematic->getPosition();
            for (std::size_t i = 0; i < 6; ++i)
                qRad[i] = q(i);
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
