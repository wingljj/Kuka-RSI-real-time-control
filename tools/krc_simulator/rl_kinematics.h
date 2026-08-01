#pragma once
#include <string>
#include "core/Pose.h"
#include "tools/krc_simulator/kinematics.h"

// rl_kinematics — Robotics Library (RL) 0.7.0 封装。
// 加载 Comau Racer 7-1.4 的 rlmdl 模型（rl::mdl::XmlFactory），提供与 kr210 一致的
// 接口：forward q(rad)→Pose(mm, A/B/C 度)、inverse Pose→q(rad)、limits(rad)。
//
// 已知约束（见 docs/rl-build-notes.md）：
// - RL 0.7.0 核心库的 AnalyticalInverseKinematics 是抽象基类（无具体实现），
//   逆解用 JacobianInverseKinematics（迭代，位姿精确；腕部零空间自运动会使
//   q3/q5 相对初始位形漂移 ±0.19 rad —— 调用方只能依赖位姿结果，不能期望 q 还原）。
// - RL 位置单位是米、四元数；本接口换算为 mm + KUKA A/B/C（ZYX，poseops）。
// - 内部对象为静态单例（进程生命周期），仅限模拟器单线程使用。
namespace rlk {

// 加载 rlmdl 模型；成功 true。路径为空用默认模型
// （D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml）。
// 失败时清空已加载状态（后续 forward/inverse 返回零值/false）。
bool loadModel(const std::string &rlmdlPath);

// 正解：q（rad）→ 笛卡尔位姿（mm；A/B/C 度，KUKA ZYX）。模型未加载返回全零 Pose。
Pose forward(const double qRad[6]);

// 逆解：目标位姿 → q（rad）。种子依次为 home、q=0、home 逐关节微扰，
// 返回首个收敛解（保证在关节限位内）。奇异/不可达返回 false（调用方回零）。
bool inverse(const Pose &target, double qRad[6]);

// 关节限位（rad），loadModel 成功时从模型 joint 读取并缓存。
const kr210::JointLimits &limits();

} // namespace rlk
