# 四元数姿态误差设计（Quaternion Attitude Error）

日期：2026-08-01
状态：设计已获用户逐节批准

## 背景

用户在奇异姿态目标（如 A=-180 / B=180 / C=-180）下观察到位姿误差高频跳动、AC 发散。
根因是逐轴欧拉角差（`poseSub` 的 `wrap180`）在欧拉奇异（B=±90/±180）与边界（±180）
处不表示 SO(3) 最短旋转：误差跳变、方向耦合。用户决定**引入四元数**（上一轮 P1-1
曾建议，当时选缓解，现决定引入）。

## 范围

**做**（用户确认"仅主机误差计算"）：
- 主机姿态误差改用四元数 → SO(3) 最短旋转（旋转向量）
- 控制增量转回 KUKA 兼容的欧拉 RKorr（`E⁻¹` 转换；KRC 文件只读、协议不变）
- 平滑器姿态用旋转向量插值（与误差一致）
- 误差显示/快照姿态用旋转向量分量
- 新增 `core/PoseOps`（纯函数）挂 `rsi_core`

**不做**：
- 模拟器不改（它接收欧拉 RKorr，`solveDelta` 的 E 转换已修复）
- 目标/误差全程四元数（仅误差计算；输出必须欧拉兼容）
- 位置改四元数（位置无奇异问题，完全不变）

## 关键机制

### `core/PoseOps`（纯函数，无 Qt 运行时依赖）

```cpp
namespace poseops {
struct Quat { double w, x, y, z; };

// KUKA A/B/C = ZYX 欧拉（度）→ 四元数：q = qz(A) ⊗ qy(B) ⊗ qx(C)
Quat quatFromABC(double aDeg, double bDeg, double cDeg);

// 四元数 → ZYX 欧拉（度）。B=±90° 奇异取 C=0 分支（与 KUKA 行为一致）。
void abcFromQuat(const Quat &q, double *aDeg, double *bDeg, double *cDeg);

// SO(3) 最短旋转：qT ⊗ qA⁻¹ → 旋转向量（世界坐标，rad）。axis × angle。
// angle = 2·atan2(|v|, w)，axis = v/|v|（|v|≈0 时旋转向量 = 0）。
Quat quatError(const Quat &target, const Quat &actual);   // 返回旋转四元数
// 旋转四元数 → 旋转向量（rad，世界坐标）
void rotVecFromQuat(const Quat &q, double rotVec[3]);

// ZYX 欧拉角速率矩阵 E(A,B,C) 及其逆（3×3）。
// ω = E·[Ȧ,Ḃ,Ċ]；[Ȧ,Ḃ,Ċ] = E⁻¹·ω。
// E = [ 0, -sA, cA·cB; 0, cA, sA·cB; 1, 0, -sB ]
// E⁻¹ 在 B=±90/±180 处退化（欧拉表示固有极限）；行列式过小返回 false（调用方钳制）。
bool invEulerRate(double aDeg, double bDeg, double cDeg, double out3x3[3][3]);
}
```

### `PoseController` 接入

- **姿态误差**：`step()` 中 `actual`/`target` 各转四元数，`rotErr = quatError(qT, qA)` → 旋转向量（世界坐标 rad）。位置误差保持 `poseSub`（减法）。
- **增量**：`d_rot = kp_rot × rotErr`，**按范数限幅**（`|d_rot| ≤ stepLimitRot`，超限缩放而非逐分量 clamp——旋转向量是单一旋转，逐分量 clamp 语义错误）。位置 `d_pos` 保持逐分量 clamp。
- **RKorr 输出**：`Δ欧拉 = invE(actual A,B,C) · d_rot`（rad→度）。`invEulerRate` 返回 false（奇异）时退化为逐轴近似（`ΔA≈d_rot.x, ΔB≈d_rot.y, ΔC≈d_rot.z`，一阶）并限幅——保证不发散、方向大致正确。
- **平滑器姿态**（RF-T3）：`m_smoothTarget` 姿态向 `m_target` 逼近用旋转向量插值——每周期 `smoothQuat = quat(alpha × rotErr_s) ⊗ smoothQuat`（`rotErr_s = quatError(qT, qSmooth)`），归一化。替换现有 `wrap180` 逐轴插值。
- `commandedSum`/`m_accum` 姿态累计保持（RKorr 欧拉增量累计，显示用）。

### 显示与快照

- `StatusSnapshot.error` 姿态部分 = **旋转向量分量**（`rotErr × 180/π`，世界坐标度）——奇异/边界下不跳变。
- UI 读数「误差 A/B/C」标签保留（值语义为旋转向量分量）；图表姿态误差曲线 = 旋转向量范数。
- `poseSub` 保留（位置差 + 欧拉差）用于其它显示/测试，控制路径不再用它算姿态误差。
- 状态卡/读数布局不变。

## 奇异处理（诚实边界）

- 四元数保证**误差方向和大小**在奇异目标下正确（SO(3) 最短旋转，不再逐轴跳变）。
- 但 RKorr 输出经 `E⁻¹` 转欧拉时，B=±90/±180 处 `E⁻¹` 退化——这是 KUKA 欧拉
  RKorr 语义的硬限制（KRC 文件只读、协议不变）。退化时退化为一阶近似 + 限幅：
  不发散、方向正确，但奇异目标收敛慢。真机操作员同样应避免奇异姿态修正。

## 测试

- 新增 `tests/test_pose_ops.cpp`：
  - `quatFromABC`/`abcFromQuat` 往返（含 B=±90 奇异分支）
  - `quatError` 在奇异目标（B=180, A/C=±180）下返回正确旋转向量（连续、非跳变）
  - 旋转向量 round-trip（`quat(rotVec)` 再取回）
  - `invEulerRate` 恒等式（`E·(E⁻¹·ω) = ω`，非奇异位形）
- 调整 `tests/test_pose_controller.cpp`：
  - 奇异目标误差正确（旋转向量连续、无 ±179 跳变）
  - RKorr 输出仍欧拉兼容（`E·Δ欧拉 ≈ d_rot`）
  - 平滑器姿态旋转向量插值（奇异跳变不再）
  - 现有位置/限幅/平滑（τ=0 直通等）用例保持
- 模拟器测试不变。

## 文件变更

- Create: `src/core/PoseOps.h` / `src/core/PoseOps.cpp`（挂 `rsi_core`）
- Create: `tests/test_pose_ops.cpp`
- Modify: `src/core/PoseController.h/.cpp`（误差/增量/平滑器）
- Modify: `src/net/SharedState.h`（error 姿态字段注释：旋转向量语义）
- Modify: `src/ui/MainWindow.cpp`（读数误差显示旋转向量；图表姿态范数）
- Modify: `tests/CMakeLists.txt` / `CMakeLists.txt`（注册）

## 分阶段

1. `PoseOps` 库 + `test_pose_ops`
2. `PoseController` 接入（误差/增量/平滑器）+ `test_pose_controller` 调整
3. 显示/快照调整 + 全量回归（含 verify_kinematics / verify_robustness）

每阶段独立构建验证 + 提交。
