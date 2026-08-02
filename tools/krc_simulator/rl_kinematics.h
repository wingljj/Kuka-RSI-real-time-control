#pragma once
#include <string>
#include <vector>
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

// 单个 body / 帧的世界位姿（mm + 四元数，正解后有效）。
struct BodyPose { double x = 0, y = 0, z = 0; double qw = 1, qx = 0, qy = 0, qz = 0; };

// 机器人骨架（body0..bodyN 世界系位姿 + TCP）。
struct Skeleton { std::vector<BodyPose> bodies; BodyPose tcp; };

// 加载 rlmdl 模型；成功 true。路径为空用默认模型
// （D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml）。
// 失败时清空已加载状态（后续 forward/inverse 返回零值/false）。
bool loadModel(const std::string &rlmdlPath);

// 正解：q（rad）→ 笛卡尔位姿（mm；A/B/C 度，KUKA ZYX）。模型未加载返回全零 Pose。
Pose forward(const double qRad[6]);

// 逆解：目标位姿 → q（rad）。返回首个收敛解（保证在关节限位内）。
// 奇异/不可达返回 false（调用方保持旧 q）。
//
// seedRad（可选，rad[6]）= 迭代起点，调用方应传机器人**当前**关节角。
// 为什么必须是当前关节角：RSI 每周期的笛卡尔增量不到 1mm，从当前位形出发
// 一两次雅可比迭代就收敛；而 home/零位这类固定种子与当前位形的距离随机器人
// 运动单调累积——机器人不动时目标≈当前位姿，home 恰好也很近（所以静止工况
// 看不出问题），一旦开始运动就越走越远、越解越慢，最终每个种子都烧完预算才
// 失败。种子必须跟着机器人走，才能让求解耗时与运动距离解耦。
//
// 固定种子（home、q=0、home 逐关节微扰）排在 seedRad 之后保留为兜底：目标
// 突变（新位姿指令）或从当前位形陷入局部极小时，仍要有一次全局搜索的机会。
// 传 nullptr（默认）时退化为纯固定种子，供单测等无状态调用方使用。
bool inverse(const Pose &target, double qRad[6], const double *seedRad = nullptr);

// 关节限位（rad），loadModel 成功时从模型 joint 读取并缓存。
const kr210::JointLimits &limits();

// 机器人骨架（body0..bodyN 世界系位姿 + TCP，mm + 四元数）。
// 前置：forward() 已调用（其内部 forwardPosition() 已计算所有 body 帧）。
// 模型未加载返回空 Skeleton（bodies 为空）。
Skeleton skeleton();

} // namespace rlk
