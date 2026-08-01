#pragma once
#include <array>
#include "core/Pose.h"

// 姿态运算纯函数：KUKA A/B/C = ZYX 欧拉角 ↔ 四元数、SO(3) 最短旋转（旋转向量）、
// ZYX 欧拉速率矩阵逆。无 Qt 运行时依赖。内部一律 rad，接口度。
namespace poseops {

struct Quat { double w, x, y, z; };

// 四元数乘法（Hamilton，右手系）。Task 2 平滑器插值依赖，故公开。
Quat quatMul(const Quat &a, const Quat &b);

// 四元数 Slerp 最短弧插值：dot<0 取反 q1（走短弧）；t=0/1 端点精确返回原值；
// 近共线（dot≈1）退化为线性插值 + 归一化，避免 acos(≈1) 数值病态。
Quat quatSlerp(const Quat &a, const Quat &b, double t);

// KUKA A/B/C = ZYX 欧拉（度）→ 四元数：q = qz(A) ⊗ qy(B) ⊗ qx(C)。
Quat quatFromABC(double aDeg, double bDeg, double cDeg);

// 四元数 → ZYX 欧拉（度）。B=±90° 奇异取 C=0 分支（与 KUKA 行为一致）。
void abcFromQuat(const Quat &q, double *aDeg, double *bDeg, double *cDeg);

// SO(3) 最短旋转四元数：qT ⊗ qA⁻¹（归一化，取最短弧 w≥0）。
Quat quatError(const Quat &target, const Quat &actual);

// 旋转四元数 → 旋转向量（世界坐标，rad）。axis × angle。
void rotVecFromQuat(const Quat &q, double rotVec[3]);

// 旋转向量（rad，世界坐标）→ 四元数（单位）。|v|≈0 返回恒等。
Quat quatFromRotVec(const double rotVec[3]);

// 便捷：目标 vs 实际 → 误差 Pose（x/y/z 逐轴差；a/b/c = SO(3) 旋转向量分量，世界坐标，度）
Pose errorPoseDeg(const Pose &target, const Pose &actual);

// ZYX 欧拉角速率矩阵逆 E⁻¹(A,B,C)（3×3）：[Ȧ,Ḃ,Ċ] = E⁻¹·ω。
// E = [[0,-sA,cA·cB],[0,cA,sA·cB],[1,0,-sB]]，det=-cosB。
// |cosB| < 0.1（B 超出 ~±84°，E⁻¹ 含 1/cosB 放大无界）返回 false；
// 调用方应回退到有界的一阶近似，避免近奇异下 RKorr 欧拉增量发散。
bool invEulerRate(double aDeg, double bDeg, double cDeg, double out3x3[3][3]);

} // namespace poseops
