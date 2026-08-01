#pragma once
#include <array>
#include "core/Pose.h"

// KUKA KR210 R3100 简化运动学：DH 正解 + 几何雅可比阻尼伪逆。
// 纯数学，无 Qt 运行时依赖（Pose.h 仅 struct）。内部一律 rad，CLI 用度。
namespace kr210 {

struct DHRow { double alpha; double a; double d; };
struct JointLimits { double min[6]; double max[6]; };

// KR210 R3100 ultra 近似 DH（standard DH）。
extern const std::array<DHRow, 6> kDh;

// 关节限位（rad）。默认 KR210 各轴范围。
const JointLimits &limits();

// 正解：q（rad）→ 笛卡尔位姿（mm；A/B/C 为度，KUKA ZYX 约定）。
Pose forward(const double qRad[6]);

// 6×6 几何雅可比（位置+姿态），q（rad）。
void jacobian(const double qRad[6], double J[6][6]);

// 笛卡尔增量（dxDeg：mm/度）→ 关节增量 dqRad（rad），阻尼伪逆。
// 近奇异时仍返回近似解（阻尼），返回 false 仅当 q 解析失败。
bool solveDelta(const double qRad[6], const Pose &dxDeg, double dqRad[6]);

// 对笛卡尔增量施加速度/加速度限制（逐分量）。限制 0 = 无。
// 位置分量用 maxVelPos/maxAccelPos（mm/s, mm/s²）；姿态用 maxVelRot/maxAccelRot
// （°/s, °/s²）。顺序：先速度 clamp 再加速度 clamp（基于 prevDx）。
void limitCartDelta(Pose *dx, const Pose &prevDx,
                    double maxVelPos, double maxVelRot,
                    double maxAccelPos, double maxAccelRot,
                    double cycleS);

} // namespace kr210
