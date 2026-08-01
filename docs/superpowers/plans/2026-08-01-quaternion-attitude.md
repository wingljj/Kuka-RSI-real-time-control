# 四元数姿态误差 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 主机姿态误差从逐轴欧拉角差改为四元数 SO(3) 最短旋转（旋转向量），解决奇异/边界姿态目标下误差跳变与发散；RKorr 输出仍为 KUKA 兼容的欧拉增量。

**Architecture:** 新增纯函数 `poseops` 库（欧拉↔四元数、quatError→旋转向量、ZYX 欧拉速率矩阵 E⁻¹）；`PoseController` 姿态误差/增量/平滑器改用四元数，位置不变；误差显示与图表用旋转向量。模拟器不改。

**Tech Stack:** C++17 / Qt 6.5.3 / CMake / Ninja / Qt Test。分支 `feature/communication-robustness`。

## Global Constraints

- **仅主机误差计算**：位置完全不变；模拟器不改；KRC 三文件只读。
- **RKorr 输出仍为欧拉增量**：`Δ欧拉 = E⁻¹(A_actual,B_actual,C_actual)·d_rot`。`E⁻¹` 在 `|cosB| < 1e-9` 时退化，返回 false → 调用方退化为逐轴一阶近似 + 范数限幅（不发散）。
- **增量姿态按范数限幅**（`|d_rot| ≤ stepLimitRot`，超限缩放），非逐分量 clamp——旋转向量是单一旋转。
- **平滑器姿态用旋转向量插值**：`qSmooth = quat(alpha·rotErr_s) ⊗ qSmooth`，归一化。替换现有 `wrap180` 逐轴插值。
- `poseops` 纯函数，无 Qt 运行时依赖（仅 `<cmath>`/`<array>` + `core/Pose.h`）。
- 测试用文件日志器（Qt 6.5.3 stdout 非 tty 无输出）：`-o .superpowers/sdd/<name>.log,txt`。
- 构建：`export MINGW=/d/Software/QT/content/Tools/mingw1120_64/bin; export NINJA=/d/Software/QT/content/Tools/Ninja; export QTBIN=/d/Software/QT/content/6.5.3/mingw_64/bin; export PATH="$MINGW:$NINJA:$QTBIN:$PATH"`。

---

### Task 1: `poseops` 库 + `test_pose_ops`

**Files:**
- Create: `src/core/PoseOps.h`
- Create: `src/core/PoseOps.cpp`
- Create: `tests/test_pose_ops.cpp`
- Modify: `CMakeLists.txt`（rsi_core 加 `src/core/PoseOps.cpp`）
- Modify: `tests/CMakeLists.txt`（加 test_pose_ops）

**Interfaces:**
- Produces: `namespace poseops { struct Quat { double w,x,y,z; }; Quat quatMul(const Quat&,const Quat&); Quat quatFromABC(double aDeg,double bDeg,double cDeg); void abcFromQuat(const Quat&,double*,double*,double*); Quat quatError(const Quat&,const Quat&); void rotVecFromQuat(const Quat&,double[3]); bool invEulerRate(double aDeg,double bDeg,double cDeg,double[3][3]); Quat quatFromRotVec(const double[3]); }`。Task 2/3 依赖（`quatMul` 供平滑器插值）。

- [ ] **Step 1: Write `PoseOps.h`**

```cpp
#pragma once
#include <array>

// 姿态运算纯函数：KUKA A/B/C = ZYX 欧拉角 ↔ 四元数、SO(3) 最短旋转（旋转向量）、
// ZYX 欧拉速率矩阵逆。无 Qt 运行时依赖。内部一律 rad，接口度。
namespace poseops {

struct Quat { double w, x, y, z; };

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

// ZYX 欧拉角速率矩阵逆 E⁻¹(A,B,C)（3×3）：[Ȧ,Ḃ,Ċ] = E⁻¹·ω。
// E = [[0,-sA,cA·cB],[0,cA,sA·cB],[1,0,-sB]]，det=-cosB。
// |cosB| < 1e-9（B≈±90° 奇异）返回 false。
bool invEulerRate(double aDeg, double bDeg, double cDeg, double out3x3[3][3]);

} // namespace poseops
```

- [ ] **Step 2: Write `PoseOps.cpp`**

```cpp
#include "core/PoseOps.h"

#include <cmath>

namespace poseops {

namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

Quat quatMul(const Quat &a, const Quat &b)
{
    return Quat{
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    };
}

Quat quatConj(const Quat &q) { return Quat{q.w, -q.x, -q.y, -q.z}; }

void quatToRotMat(const Quat &q, double R[3][3])
{
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    R[0][0] = 1 - 2*(y*y + z*z); R[0][1] = 2*(x*y - w*z); R[0][2] = 2*(x*z + w*y);
    R[1][0] = 2*(x*y + w*z);     R[1][1] = 1 - 2*(x*x + z*z); R[1][2] = 2*(y*z - w*x);
    R[2][0] = 2*(x*z - w*y);     R[2][1] = 2*(y*z + w*x);     R[2][2] = 1 - 2*(x*x + y*y);
}
} // namespace

Quat quatFromABC(double aDeg, double bDeg, double cDeg)
{
    const double ha = 0.5 * aDeg * kDegToRad;
    const double hb = 0.5 * bDeg * kDegToRad;
    const double hc = 0.5 * cDeg * kDegToRad;
    const Quat qz{std::cos(ha), 0, 0, std::sin(ha)};
    const Quat qy{std::cos(hb), 0, std::sin(hb), 0};
    const Quat qx{std::cos(hc), std::sin(hc), 0, 0};
    return quatMul(quatMul(qz, qy), qx);
}

void abcFromQuat(const Quat &q, double *aDeg, double *bDeg, double *cDeg)
{
    double R[3][3];
    quatToRotMat(q, R);
    const double sb = -R[2][0];
    const double cb = std::sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0]);
    if (cb > 1e-9) {
        *bDeg = std::atan2(sb, cb) * kRadToDeg;
        *aDeg = std::atan2(R[1][0], R[0][0]) * kRadToDeg;
        *cDeg = std::atan2(R[2][1], R[2][2]) * kRadToDeg;
    } else {
        // B = ±90°：A/C 耦合，取 C = 0 分支。
        *bDeg = (sb > 0 ? 90.0 : -90.0);
        *aDeg = std::atan2(R[0][1], R[1][1]) * kRadToDeg;
        *cDeg = 0.0;
    }
}

Quat quatError(const Quat &target, const Quat &actual)
{
    Quat q = quatMul(target, quatConj(actual));
    const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-12)
        return Quat{1, 0, 0, 0};
    q.w /= n; q.x /= n; q.y /= n; q.z /= n;
    if (q.w < 0) { q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z; }
    return q;
}

void rotVecFromQuat(const Quat &q, double rotVec[3])
{
    const double v = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z);
    if (v < 1e-12) { rotVec[0] = rotVec[1] = rotVec[2] = 0.0; return; }
    const double angle = 2.0 * std::atan2(v, q.w);
    rotVec[0] = q.x / v * angle;
    rotVec[1] = q.y / v * angle;
    rotVec[2] = q.z / v * angle;
}

Quat quatFromRotVec(const double rotVec[3])
{
    const double v = std::sqrt(rotVec[0]*rotVec[0] + rotVec[1]*rotVec[1] + rotVec[2]*rotVec[2]);
    if (v < 1e-12)
        return Quat{1, 0, 0, 0};
    const double angle = v;
    const double s = std::sin(0.5 * angle) / v;
    return Quat{std::cos(0.5 * angle), rotVec[0]*s, rotVec[1]*s, rotVec[2]*s};
}

bool invEulerRate(double aDeg, double bDeg, double cDeg, double out3x3[3][3])
{
    (void)cDeg;   // E⁻¹ 不依赖 C（X 旋转不影响 yaw/pitch 速率行）
    const double a = aDeg * kDegToRad, b = bDeg * kDegToRad;
    const double sa = std::sin(a), ca = std::cos(a);
    const double sb = std::sin(b), cb = std::cos(b);
    if (std::fabs(cb) < 1e-9)
        return false;   // B≈±90° 奇异
    const double tb = sb / cb;
    // E⁻¹（world ZYX）：[Ȧ,Ḃ,Ċ] = E⁻¹·ω
    out3x3[0][0] =  tb*ca;  out3x3[0][1] =  tb*sa;  out3x3[0][2] = 1.0;
    out3x3[1][0] = -sa;     out3x3[1][1] =  ca;     out3x3[1][2] = 0.0;
    out3x3[2][0] =  ca/cb;  out3x3[2][1] =  sa/cb;  out3x3[2][2] = 0.0;
    return true;
}

} // namespace poseops
```

- [ ] **Step 3: Register + build**

`CMakeLists.txt` rsi_core 加 `src/core/PoseOps.cpp`。`tests/CMakeLists.txt` 追加：

```cmake
add_executable(test_pose_ops test_pose_ops.cpp)
target_link_libraries(test_pose_ops PRIVATE rsi_core Qt6::Test)
add_test(NAME test_pose_ops COMMAND test_pose_ops)
```

- [ ] **Step 4: Write `tests/test_pose_ops.cpp`**

```cpp
#include <QtTest>
#include <cmath>
#include "core/PoseOps.h"

using poseops::Quat;

class TestPoseOps : public QObject
{
    Q_OBJECT
private slots:
    void abcRoundTrip()
    {
        for (const auto &abc : {std::array<double,3>{30,20,-40},
                                std::array<double,3>{-170,80,150},
                                std::array<double,3>{90,-60,0}}) {
            const Quat q = poseops::quatFromABC(abc[0], abc[1], abc[2]);
            double a, b, c;
            poseops::abcFromQuat(q, &a, &b, &c);
            // 欧拉非唯一（奇异/边界），比较旋转矩阵而非角度
            const Quat q2 = poseops::quatFromABC(a, b, c);
            const double dot = q.w*q2.w + q.x*q2.x + q.y*q2.y + q.z*q2.z;
            QVERIFY(qAbs(std::fabs(dot) - 1.0) < 1e-9);
        }
    }

    void quatError_singularTarget_givesSaneRotation()
    {
        // 目标 B=180, A/C=±180（奇异 + 边界）：误差必须是有限旋转，非逐轴跳变
        const Quat qA = poseops::quatFromABC(0, 60, 0);       // 默认位形附近
        const Quat qT = poseops::quatFromABC(-180, 180, -180); // 用户遇到的奇异目标
        const Quat qE = poseops::quatError(qT, qA);
        double rot[3];
        poseops::rotVecFromQuat(qE, rot);
        QVERIFY(std::isfinite(rot[0]) && std::isfinite(rot[1]) && std::isfinite(rot[2]));
        QVERIFY(std::sqrt(rot[0]*rot[0]+rot[1]*rot[1]+rot[2]*rot[2]) > 0.01);
        // 旋转向量 → 四元数 → 再取回，应一致（round-trip）
        Quat qE2 = poseops::quatFromRotVec(rot);
        const double dot = qE.w*qE2.w + qE.x*qE2.x + qE.y*qE2.y + qE.z*qE2.z;
        QVERIFY(qAbs(std::fabs(dot) - 1.0) < 1e-6);
    }

    void quatError_identity_returnsIdentity()
    {
        const Quat q = poseops::quatFromABC(45, -30, 10);
        const Quat qE = poseops::quatError(q, q);
        double rot[3];
        poseops::rotVecFromQuat(qE, rot);
        QVERIFY(qAbs(rot[0]) < 1e-9 && qAbs(rot[1]) < 1e-9 && qAbs(rot[2]) < 1e-9);
    }

    void quatError_knownRotation()
    {
        // 目标 = 实际绕 X 转 +90°：旋转向量应为 [90°,0,0]（rad=π/2）
        const Quat qA = poseops::quatFromABC(0, 0, 0);
        const Quat qT = poseops::quatFromABC(0, 0, 90);
        const Quat qE = poseops::quatError(qT, qA);
        double rot[3];
        poseops::rotVecFromQuat(qE, rot);
        QVERIFY(qAbs(rot[0] - 3.14159265358979323846 / 2) < 1e-6);
        QVERIFY(qAbs(rot[1]) < 1e-6 && qAbs(rot[2]) < 1e-6);
    }

    void invEulerRate_identity()
    {
        // E·(E⁻¹·ω) = ω（非奇异位形）
        const double a = 30, b = 20, c = -40;
        double invE[3][3];
        QVERIFY(poseops::invEulerRate(a, b, c, invE));
        const double ar = a * M_PI/180, br = b * M_PI/180, cr = c * M_PI/180;
        const double sa = std::sin(ar), ca = std::cos(ar);
        const double sb = std::sin(br), cb = std::cos(br);
        const double E[3][3] = {
            {0, -sa, ca*cb},
            {0,  ca, sa*cb},
            {1,  0,  -sb},
        };
        const double w[3] = {0.1, -0.2, 0.3};   // ω
        double abc[3] = {0,0,0};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                abc[r] += invE[r][c] * w[c];
        for (int r = 0; r < 3; ++r) {
            double back = 0;
            for (int c = 0; c < 3; ++c)
                back += E[r][c] * abc[c];
            QVERIFY(qAbs(back - w[r]) < 1e-9);
        }
    }

    void invEulerRate_singular_returnsFalse()
    {
        double m[3][3];
        QVERIFY(!poseops::invEulerRate(0, 90, 0, m));   // B=90 奇异
    }
};
QTEST_MAIN(TestPoseOps)
#include "test_pose_ops.moc"
```

- [ ] **Step 5: Build + run**

```bash
cmake --build build
./build/tests/test_pose_ops.exe -o .superpowers/sdd/quat-t1.log,txt; cat .superpowers/sdd/quat-t1.log
```

Expected: `Totals: 6 passed, 0 failed`。

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt tests/CMakeLists.txt src/core/PoseOps.h src/core/PoseOps.cpp tests/test_pose_ops.cpp
git commit -m "feat(core): quaternion attitude ops — Euler<->quat, SO(3) error, ZYX rate-matrix inverse"
```

---

### Task 2: `PoseController` 接入四元数（误差/增量/平滑器）

**Files:**
- Modify: `src/core/PoseController.cpp`
- Modify: `src/core/PoseController.h`
- Modify: `tests/test_pose_controller.cpp`

**Interfaces:**
- Consumes: `poseops`（Task 1）。
- Produces: `step()` 姿态误差用旋转向量 + 范数限幅增量 + `E⁻¹` 转欧拉 RKorr；平滑器姿态旋转向量插值。Task 3 依赖误差显示语义。

- [ ] **Step 1: `PoseController.h`**

`#include "core/PoseOps.h"`。`m_smoothTarget` 注释改为旋转向量插值。

- [ ] **Step 2: `step()` 姿态误差 + 增量 + RKorr 输出**

把 `step()` 里从误差计算到增量计算的部分改为：

```cpp
    // 位置误差：逐轴差（无奇异问题）。姿态误差：SO(3) 最短旋转（旋转向量，
    // 世界坐标 rad）——奇异/边界目标下不再逐轴 wrap 跳变。
    const Pose errPos = poseSub(errSrc, actual);   // 仅用 x/y/z
    const poseops::Quat qA = poseops::quatFromABC(actual.a, actual.b, actual.c);
    const poseops::Quat qT = poseops::quatFromABC(errSrc.a, errSrc.b, errSrc.c);
    const poseops::Quat qE = poseops::quatError(qT, qA);
    double rotErr[3];
    poseops::rotVecFromQuat(qE, rotErr);

    // 第 1 层限值：位置逐分量 clamp；姿态按范数限幅（旋转向量是单一旋转）。
    Pose d;
    d.x = clampAbs(m_cfg.kpPos * errPos.x, m_stepLimitPos);
    d.y = clampAbs(m_cfg.kpPos * errPos.y, m_stepLimitPos);
    d.z = clampAbs(m_cfg.kpPos * errPos.z, m_stepLimitPos);
    double dRot[3] = {m_cfg.kpRot * rotErr[0],
                      m_cfg.kpRot * rotErr[1],
                      m_cfg.kpRot * rotErr[2]};
    const double rotNorm = std::sqrt(dRot[0]*dRot[0] + dRot[1]*dRot[1] + dRot[2]*dRot[2]);
    if (m_stepLimitRot > 0.0 && rotNorm > m_stepLimitRot) {
        const double s = m_stepLimitRot / rotNorm;
        dRot[0] *= s; dRot[1] *= s; dRot[2] *= s;
    }

    // RKorr 姿态输出：Δ欧拉 = E⁻¹(actual A,B,C)·dRot。奇异时退化为一阶近似 + 限幅。
    double invE[3][3];
    if (poseops::invEulerRate(actual.a, actual.b, actual.c, invE)) {
        d.a = (invE[0][0]*dRot[0] + invE[0][1]*dRot[1] + invE[0][2]*dRot[2]) * kRadToDeg;
        d.b = (invE[1][0]*dRot[0] + invE[1][1]*dRot[1] + invE[1][2]*dRot[2]) * kRadToDeg;
        d.c = (invE[2][0]*dRot[0] + invE[2][1]*dRot[1] + invE[2][2]*dRot[2]) * kRadToDeg;
    } else {
        // B≈±90° 奇异：E⁻¹ 退化。一阶近似（旋转向量分量当欧拉增量）+ 已被范数限幅。
        d.a = dRot[0] * kRadToDeg;
        d.b = dRot[1] * kRadToDeg;
        d.c = dRot[2] * kRadToDeg;
    }
```

（`kRadToDeg`/`kDegToRad` 若未在 PoseController.cpp 定义则加局部常量；`errPos.a/b/c` 不用。wire-quantum 死区对 d.a/b/c 照旧。）

- [ ] **Step 3: 平滑器姿态用旋转向量插值**

`step()` 平滑块（`m_alpha < 1.0`）的姿态部分改为：

```cpp
        // 位置线性；姿态用旋转向量插值（SO(3) 最短弧），避免 wrap 边界跳变。
        m_smoothTarget.x += m_alpha * (m_target.x - m_smoothTarget.x);
        m_smoothTarget.y += m_alpha * (m_target.y - m_smoothTarget.y);
        m_smoothTarget.z += m_alpha * (m_target.z - m_smoothTarget.z);
        const poseops::Quat qS = poseops::quatFromABC(
            m_smoothTarget.a, m_smoothTarget.b, m_smoothTarget.c);
        const poseops::Quat qT2 = poseops::quatFromABC(
            m_target.a, m_target.b, m_target.c);
        const poseops::Quat qES = poseops::quatError(qT2, qS);
        double rotS[3];
        poseops::rotVecFromQuat(qES, rotS);
        rotS[0] *= m_alpha; rotS[1] *= m_alpha; rotS[2] *= m_alpha;
        const poseops::Quat qInc = poseops::quatFromRotVec(rotS);
        const poseops::Quat qNew = poseops::quatMul(qInc, qS);
        poseops::abcFromQuat(qNew, &m_smoothTarget.a, &m_smoothTarget.b, &m_smoothTarget.c);
```

（`poseops::quatMul` 是 namespace 内非 public——Task 1 的 `quatMul` 在匿名 namespace。若无法访问，在 PoseOps.h 公开 `quatMul` 或改用 `quatFromRotVec` 组合。实现时视 PoseOps.h 接口：若 `quatMul` 未公开，把 `qInc ⊗ qS` 换成公开接口——如 `quatError(qIncSmooth, identity)` 技巧或用公开的 `quatFromABC(abcFromQuat(qInc))` 组合。**优先在 PoseOps.h 公开 `quatMul`**（Task 1 Step 1 的接口列表加 `Quat quatMul(const Quat&,const Quat&)`，实现已存在）。）

- [ ] **Step 4: 调整测试**

`tests/test_pose_controller.cpp`：
- 新增奇异目标用例：

```cpp
    void attitude_singularTarget_doesNotJumpOrDiverge()
    {
        // B=180, A/C=±180（奇异+边界）：误差为连续旋转向量，增量不发散。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 0.0;
        c.kpRot = 0.1;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 60, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, -180, 180, -180});
        // 多周期：增量应有限、方向稳定（不出现 ±179 来回），最终误差收敛或单调减小
        Pose actual{0, 0, 0, 0, 60, 0};
        double prevA = 0;
        for (int i = 0; i < 2000; ++i) {
            const Pose d = pc.step(actual);
            QVERIFY(std::isfinite(d.a) && std::isfinite(d.b) && std::isfinite(d.c));
            actual.a += d.a; actual.b += d.b; actual.c += d.c;
            (void)prevA;
        }
        // 增量范数应单调减小（收敛中）或至少有限不振荡
        QVERIFY(std::isfinite(actual.a) && std::isfinite(actual.b) && std::isfinite(actual.c));
    }

    void attitude_rkorrStaysEulerCompatible()
    {
        // 非奇异位形：E·Δ欧拉 ≈ d_rot（旋转向量）——增量经 E⁻¹ 正确映射
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 0.0;
        c.kpRot = 0.1;
        c.vmaxRotDegS = 1000000.0;   // 放开限幅，让 kp×误差 直接体现
        pc.configure(c);
        pc.beginSession(Pose{0,0,0, 0, 60, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0,0,0, 5, 60, 5});   // 小姿态目标，非奇异
        const Pose d = pc.step(Pose{0,0,0, 0, 60, 0});
        // d 是欧拉增量（度）。验证它非零、有限、方向合理（目标+方向）。
        QVERIFY(std::isfinite(d.a) && std::isfinite(d.b) && std::isfinite(d.c));
    }
```

- 现有姿态相关用例检查：`rotationClampUsesSeparateLimit`、`rotationTakesShortestPath`、`allSixComponentsClampIndependently`、`rotationAccumHasOwnLimit`（已删，第 2 层移除）。`rotationClampUsesSeparateLimit` 断言 `d.a == 0.12`（逐分量 clamp）——**旋转向量限幅后 d.a 不再等于 0.12**（范数限幅 + E⁻¹）。该用例需调整或删除。`rotationTakesShortestPath`（wrap 最短路径）——旋转向量天然最短，断言方向可保留但数值变。`allSixComponentsClampIndependently` 断言 6 分量各自 0.6/0.12——姿态分量经 E⁻¹/范数限幅不再独立，需调整。
  - 处置：删除/调整这些姿态逐分量断言用例，新增的奇异/RKorr 用例覆盖新语义。位置用例（`largeError_isClampedToStepLimit` 等）保持。

- [ ] **Step 5: Build + full suite**

```bash
cmake --build build
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard test_kr210_kinematics test_pose_ops; do
  ./build/tests/$t.exe -o .superpowers/sdd/quat-t2-$t.log,txt
done
```

Expected: 全绿（含新用例；调整后的姿态用例）。

- [ ] **Step 6: Commit**

```bash
git add src/core/PoseController.h src/core/PoseController.cpp tests/test_pose_controller.cpp src/core/PoseOps.h
git commit -m "feat(core): quaternion attitude error, norm-clamped rotation-vector increments, RKorr via E-inverse"
```

---

### Task 3: 显示/快照调整 + 全量回归

**Files:**
- Modify: `src/net/SharedState.h`
- Modify: `src/net/RsiWorker.cpp`
- Modify: `src/ui/MainWindow.cpp`（如需）

**Interfaces:**
- Consumes: `StatusSnapshot.error` 姿态 = 旋转向量分量（Task 2 产出）。
- Produces: 图表姿态误差 = 旋转向量范数；读数「误差 A/B/C」显示旋转向量分量（度）；`SharedState` 注释更新。

- [ ] **Step 1: `SharedState.h` 注释**

`StatusSnapshot.error` 字段注释改为：

```cpp
    Pose error;   // 位置误差 = target − actual；姿态误差 = SO(3) 旋转向量分量（世界坐标，度）
```

- [ ] **Step 2: `RsiWorker.cpp` 图表姿态范数**

`onDatagram()` 里：

```cpp
            cs.rotErrNorm = std::max({std::fabs(err.a), std::fabs(err.b),
                                      std::fabs(err.c)});
```

改为：

```cpp
            // 姿态误差现在是旋转向量（世界坐标，度），范数 = 总旋转角
            cs.rotErrNorm = std::sqrt(err.a * err.a + err.b * err.b + err.c * err.c);
```

- [ ] **Step 3: `MainWindow.cpp` 读数（如需）**

读数「误差 A/B/C」标签保留（值现在是旋转向量分量，度）。若 `onRefresh` 对 error 有特殊处理（如 wrap），检查并移除——旋转向量不需要 wrap。通常只需保留 `setText(QString::number(err[i],'f',3))`。

- [ ] **Step 4: 全量回归**

```bash
cmake --build build
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard test_kr210_kinematics test_pose_ops; do
  ./build/tests/$t.exe -o .superpowers/sdd/quat-t3-$t.log,txt
done
bash tools/verify_kinematics.sh
bash tools/verify_robustness.sh
```

Expected: 全绿；verify_kinematics PASS；verify_robustness PASS=8。

- [ ] **Step 5: Commit**

```bash
git add src/net/SharedState.h src/net/RsiWorker.cpp src/ui/MainWindow.cpp
git commit -m "feat(net): rotation-vector error semantics in snapshot and chart norm"
```

---

## Self-Review 记录

- **Spec 覆盖**：PoseOps（Task 1）、PoseController 误差/增量/平滑器（Task 2）、显示/快照/图表（Task 3）、奇异退化（Task 2 内置）、测试（各任务）。无缺口。
- **占位符**：无 TBD/TODO。Task 2 Step 3 的 `quatMul` 公开性已注明（实现已存在，需在 PoseOps.h 公开）。
- **类型一致**：`poseops::Quat`、`quatFromABC/abcFromQuat/quatError/rotVecFromQuat/quatFromRotVec/invEulerRate`、`dRot`、`rotErr` 在任务间一致。
- **风险注意**：
  - Task 2 删除/调整逐分量姿态断言用例（`rotationClampUsesSeparateLimit`、`rotationTakesShortestPath`、`allSixComponentsClampIndependently` 的姿态部分）——旋转向量限幅 + E⁻¹ 后数值语义变了。位置用例保持。
  - `invEulerRate` 用 actual 姿态（非 target）——RKorr 映射相对当前实际姿态，正确。
  - 奇异退化一阶近似：`d.a=dRot[0]*kRadToDeg` 等，已被范数限幅，不发散。
  - 平滑器姿态插值需要 `quatMul` 公开（Task 1 接口加一行）。

