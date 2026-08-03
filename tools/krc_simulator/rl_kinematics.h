#pragma once
#include <chrono>
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

// 一次 inverse() 调用允许消耗的总时长，由发帧周期推导。
//
// 为什么预算必须跟着周期算、不能写死：模拟器是单线程的「收帧 → 逆解 → 正解 →
// 发下一帧」，逆解花掉的时间一比一变成发帧延迟；而 --cycle-ms 是用户可设的
// CLI 选项（默认 12，真机 IPO 常用 4），任何按 12ms 挑出来的常数换到 4ms 周期
// 就重新崩节拍——实测把预算固定成 9ms（fb435d9 的 9 种子 × 1ms 上界）再跑
// --cycle-ms 4，帧间隔 p50 从 4.00ms 变成 9.17ms，1500 帧要 13.8s 而不是 6s。
// 后果不只是慢：上位机看门狗是 max(200ms, cycleMs×20)，一旦被拖过阈值 connected
// 就置 false、下一帧又置回 true —— 用户看到的「已连接/监听中」抖动就是这么来的。
//
// 取半个周期：另一半留给收包等待 + 正解 + UDP 发送 +（--viz 时）渲染。下限 0.2ms
// 只为挡住 --cycle-ms 0 这类退化输入（0.4ms 以下的周期比任何真实 IPO 快一个数量
// 级，不是要支持的工况）。tests 里 solveBudget_staysBelowOneCycle 断的就是
// 「预算 < 周期」这条函数关系本身，不是某个具体毫秒数。
std::chrono::nanoseconds solveBudgetForCycle(double cycleMs);

// 不在发帧回路里的调用方（目前只有单测）的默认总预算。
// 慷慨一点是为了可达率：绝对目标（无种子）在 50ms 总预算下实测 250/250 收敛，
// 而 6ms 只有 74%（数字见 rl_kinematics.cpp 里 kHotSeedShare 的注释）。
// 但仍然有界，免得「忘了传预算」退化回原来最坏近 1 秒的求解。
// 模拟器**不能**用这个默认值——它必须用 solveBudgetForCycle(cycleMs)。
constexpr std::chrono::nanoseconds kDefaultSolveBudget{std::chrono::milliseconds{50}};

// 逆解：目标位姿 → q（rad）。返回首个收敛解（保证在关节限位内）。
// 奇异/不可达返回 false（调用方保持旧 q）。
//
// seedRad（可选，rad[6]）= 迭代起点，调用方应传机器人**当前**关节角。
// 为什么必须是当前关节角，有两条独立的理由：
//
// 1) 耗时与运动距离解耦。RSI 每周期的笛卡尔增量不到 1mm，从当前位形出发一两次
//    雅可比迭代就收敛；而 home/零位这类固定种子与当前位形的距离随机器人运动单调
//    累积——机器人不动时目标≈当前位姿，home 恰好也很近（所以静止工况看不出
//    问题），一旦开始运动就越走越远、越解越慢，最终每个种子都烧完预算才失败。
// 2) 选中**当前的 IK 分支**。inverse() 返回首个收敛的种子解，所以种子顺序就是
//    分支优先级：把当前 q 排第一，等于要求「不许翻分支」。home 优先时没有这个
//    保证，模拟机器人可能在运动中途跳到另一个分支（同一个 TCP 位姿、关节角差
//    整段 π）。后果不是 RIst 直接跳——分支解的位姿本来就相同，RIst 仍连续——
//    而是 AIPos 逐帧不连续，且一旦跳到的分支被模拟器本地的 --joint-limits 夹住，
//    正解就不再等于目标位姿，这才变成 RIst 跳变、被上位机 exceedsPhysicalJump
//    判成 stale 帧。属于消掉一个潜在的假 stale 源（200 帧 / 120mm 的增量扫掠里
//    没能实测到 home 优先真的翻分支，所以这是隐患解除，不是已观测缺陷修复）。
//    tests/test_rl_kinematics.cpp 的 inverse_seed_selectsSeedBranch 锁住种子
//    与分支的对应关系。
//
// 固定种子（home、q=0、home 逐关节微扰）排在 seedRad 之后保留为兜底：目标
// 突变（新位姿指令）或从当前位形陷入局部极小时，仍要有一次全局搜索的机会。
// 传 nullptr（默认）时退化为纯固定种子，供单测等无状态调用方使用。
//
// budget = 本次调用的总时长上限（所有种子共享，见 solveBudgetForCycle）。
// 超时即放弃并返回 false —— 由调用方的「保持旧 q」兜住。
bool inverse(const Pose &target, double qRad[6], const double *seedRad = nullptr,
             std::chrono::nanoseconds budget = kDefaultSolveBudget);

// 关节限位（rad），loadModel 成功时从模型 joint 读取并缓存。
const kr210::JointLimits &limits();

// 机器人骨架（body0..bodyN 世界系位姿 + TCP，mm + 四元数）。
// 前置：forward() 已调用（其内部 forwardPosition() 已计算所有 body 帧）。
// 模型未加载返回空 Skeleton（bodies 为空）。
Skeleton skeleton();

} // namespace rlk
