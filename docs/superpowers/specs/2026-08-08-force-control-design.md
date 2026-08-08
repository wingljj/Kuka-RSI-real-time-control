# 力控功能设计规格

**日期**: 2026-08-08
**状态**: 待评审
**范围**: 在 KUKA RSI 上位机中新增力控制模式（与位姿跟踪互斥），通过 SRI 六维力传感器实现导纳控制

---

## 1. 概述

### 1.1 目标

为现有 KUKA RSI 上位机增加力控制功能。操作员可在线切换"位姿跟踪"和"力控制"两种工作模式，力控通过 SRI 六维力传感器实现柔顺装配。

### 1.2 使用场景

1. 操作员手动将机械臂粗对位到预装配位置
2. 切换到力控制页面，检查传感器连接状态
3. 力传感器清零（去皮）
4. 使能力控制，机械臂根据接触力自动柔顺调整位姿
5. 装配完成后停止力控

### 1.3 关键决策

| 决策 | 结论 |
|------|------|
| SRI 连接方式 | 直连主机（TCP），独立读取线程 |
| 力/位关系 | 互斥，不同时启用 |
| 控制算法 | sigmoid 导纳控制（力→速度→位置增量），直通不经过 kp |
| 自由度 | 六轴可独立开关（方向混合矩阵） |
| 死区 | 力/力矩各自独立的矢量球域死区 |
| 滤波 | SRI 线程窗口均值抗混叠 + 通信线程 Butterworth 二阶低通 |
| UI 布局 | 力控/位姿跟踪两个独立页面，QStackedWidget 切换 |

---

## 2. 架构

### 2.1 模块分解

```
src/
├── core/
│   ├── Wrench.h                  新增: 六维力数据结构
│   ├── SriProtocol.h/cpp         新增: SRI 二进制帧解析器
│   ├── SriDriver.h/cpp           新增: SRI TCP 读取线程
│   ├── ForceFilter.h/cpp         新增: Butterworth 二阶低通
│   ├── ForceController.h/cpp     新增: 力控管线
│   └── PoseController.h/cpp      不变
├── net/
│   ├── RsiWorker.h/cpp           修改: 集成力控分支
│   └── SharedState.h             修改: 增加力数据字段
└── ui/
    ├── ForceChart.h/cpp          新增: 力曲线图表 (30s)
    ├── ForcePanel.h/cpp          新增: 力控配置面板
    └── MainWindow.h/cpp          修改: 页面切换, 菜单项
```

### 2.2 依赖关系

```
SriDriver ──→ SriProtocol          (TCP连接 + 帧解析)
                   ↓ WrenchFrame (mutex)
RsiWorker ──→ ForceController       (取力→滤波→死区→sigmoid→Δpose)
ForceController ──→ ForceFilter     (Butterworth 二阶低通)
               ↓ Δpose (Pose)
RsiWorker ──→ PoseController.step() (现有管线，力控模式下直通)
```

### 2.3 设计原则

- `SriDriver`: 只管 TCP 和帧解析，不碰控制逻辑
- `ForceController`: 纯计算单元（无 IO、无信号槽），可在通信线程直接调用——与 `PoseController` 同风格
- 力控和位姿跟踪互斥: `RsiWorker::step()` 顶层分支
- SRI 断连时力控自动退出，不继续用旧力值修正

---

## 3. 线程模型

### 3.1 双线程架构

```
SRI 读取线程 (独立 QThread):         通信线程 (4ms RSI 周期):
  TCP recv → feed parser                 lock → 取最新力值
  → 解析出 6×float                        力 → 滤波 → 死区 → sigmoid
  → 窗口均值累加器 += raw                 → 速度 → Δpose
  → lock → 写入 m_latestWrench            台账记账
  → unlock                                send RKorr
```

### 3.2 SRI 协议

- **传输**: TCP 流，默认 `192.168.0.108:4008`
- **帧格式**: `\xAA\x55\x00\x1B` (4B 头) + 2B 填充 + 6×float32 LE = 31 字节
- **数据顺序**: Fx, Fy, Fz, Mx, My, Mz（工程单位）
- **采样率**: 2kHz，降采样至 250Hz（RSI 周期）
- **操作**: 发 `AT+GSD\r\n` 启动推流

### 3.3 SRI 重连

TCP 断开后指数退避重连 (0.5s→1s→2s→...→5s max)。重连成功后首帧作为新偏置（自动清零）。连续无新鲜帧超过 `stale_timeout_ms` → Fault → 力控自动退出。

---

## 4. 数据流

### 4.1 力控模式每周期处理

```
1. SRI 共享区取最新力
   m_sriLatest → 窗口均值 (降采样 + 抗混叠)
   
2. 传感器→工具→BASE 坐标变换
   F_sensor → sensor_T_tool (静态配置) → q_actual (动态) → F_tool_base
   
3. ForceController 管线
   F_base → Butterworth LPF → 矢量死区 → 方向混合矩阵 → sigmoid → 速度矢量
   
4. 速度→位置增量
   v × dt → ΔXYZ
   ω × dt → ΔABC (rotVec → E⁻¹)
   
5. vmax 硬限幅 + 量化
   
6. 台账记账
   m_cmd += Δpose
   
7. 发送 RKorr 给 KRC
```

### 4.2 力控启停

**使能瞬间**:
- 力传感器清零（记录当前窗口均值为偏置，后续减去）
- 台账重对齐: `m_cmd = RIst`
- Butterworth 状态用首帧力值预填充（避免阶跃冲击）

**清零前置检查**:
- 最近 N 帧增量全部 < 阈值（机器人必须静止）
- 清零偏置力矢量大小不应超过工具重力的 2 倍
- 保存偏置值到 SharedState，力曲线可切换"原始力/净力"视图

**关闭条件**:
- 操作员点击"停止"
- SRI 掉线超过 stale 阈值
- 任何 Fault

**关闭后**: 台账保留，状态回 Idle

### 4.3 模式切换

操作员通过工具栏切换"位姿跟踪"和"力控制"页面。当前模式先停止（停止跟踪或退出力控），`m_cmdSynced = false`，再切换 `QStackedWidget` 页面。力控活跃时不允许切换。

---

## 5. 力控管线算法

### 5.1 处理链

```
力矢量(BASE)             力矩矢量(BASE)
     │                       │
     ▼                       ▼
Butterworth LPF          Butterworth LPF      二阶 10Hz, 可配置
     │                       │
     ▼                       ▼
矢量死区                  矢量死区              ‖F‖ < deadzone → 归零
dead_force_n              dead_torque_nm       球域半径
     │                       │
     ▼                       ▼
方向混合矩阵              方向混合矩阵          6×6 对角矩阵
M_force                   M_torque             diag(en_x..en_c)
     │                       │
     ▼                       ▼
sigmoid 力                sigmoid 力矩          ‖F‖ → v (标量)
v = f(‖F‖)                ω = g(‖M‖)
     │                       │
     ▼                       ▼
速度分解到各轴            速度分解到各轴
v_axis = v × F_axis/‖F‖  ω_axis = ω × M_axis/‖M‖
     │                       │
     ▼                       ▼
Δpose = v × dt            Δrot = ω × dt → E⁻¹ → ΔABC
```

### 5.2 sigmoid 函数

```
v = v_max × tanh( gain × max(0, ‖F‖ − deadzone) )
```

| 参数 | 物理含义 |
|------|----------|
| `deadzone` | 力/力矩矢量在此以下输出零速度 |
| `gain` | 线性区导纳增益。`gain ≈ 1/(F_sat − deadzone) × atanh(0.95)` |
| `v_max` | 饱和速度上限 |

力和力矩各自一套参数。默认值:

| 参数 | 位置 | 姿态 |
|------|------|------|
| `deadzone` | 5.0 N | 1.0 Nm |
| `gain` | 0.05 N⁻¹ | 0.5 Nm⁻¹ |
| `v_max` | 5.0 mm/s | 1.0 deg/s |

### 5.3 Butterworth 二阶低通

```
H(z) = (b₀ + b₁z⁻¹ + b₂z⁻²) / (1 + a₁z⁻¹ + a₂z⁻²)

fc = 10 Hz (默认, 可配置)
fs = 250 Hz (RSI 周期)
```

系数在 `fc` 变化时离线预计算。每周期 4 次乘法 + 3 次加法，无堆分配。使能时用首帧力值预填充状态。

### 5.4 方向混合矩阵

在死区之后、sigmoid 之前施加。死区计算用原始矢量大小（不受方向开关影响）。

```json
"axes": { "en_x": false, "en_y": false, "en_z": true,
          "en_a": false, "en_b": false, "en_c": false }
```

所有轴 `false` → UI 红色边框提示。

### 5.5 坐标变换

```
# 静态: 配置加载时预计算
sensor_T_tool = invert(flange_T_sensor) × flange_T_tool

# 动态: 每 RSI 周期
F_tool_sensor = sensor_T_tool[:3,:3] × F_raw
q_actual = quatFromABC(RIst.a, RIst.b, RIst.c)
F_tool_base = q_actual ⊗ F_tool_sensor ⊗ q_actual⁻¹
```

只有旋转部分，不需要平移——POSCORR BASE 模式直接对应。方向通道符号 (`channel_signs`) 处理坐标系手性翻转。

---

## 6. 配置

### 6.1 配置结构

`rsi_config.json` 中新增 `force_control` 块:

```json
"force_control": {
  "sensor": {
    "host": "192.168.0.108",
    "port": 4008,
    "channel_signs": [1, 1, 1, 1, 1, 1],
    "torque_scale": 1.0,
    "force_capacity_n": [7200, 7200, 18000],
    "torque_capacity_nm": [1400, 1400, 1400],
    "capacity_warning_ratio": 0.70,
    "stale_timeout_ms": 100.0
  },
  "mounting": {
    "flange_T_sensor": {
      "x_mm": 0.0, "y_mm": 0.0, "z_mm": 85.0,
      "a_deg": 0.0, "b_deg": 0.0, "c_deg": 0.0
    },
    "flange_T_tool": {
      "x_mm": 0.0, "y_mm": 0.0, "z_mm": 150.0,
      "a_deg": 0.0, "b_deg": 0.0, "c_deg": 0.0
    }
  },
  "filter": { "cutoff_hz": 10.0 },
  "deadzone": { "force_n": 5.0, "torque_nm": 1.0 },
  "admittance": {
    "gain_force": 0.05, "gain_torque": 0.5,
    "vmax_pos_mm_s": 5.0, "vmax_rot_deg_s": 1.0
  },
  "axes": {
    "en_x": false, "en_y": false, "en_z": true,
    "en_a": false, "en_b": false, "en_c": false
  }
}
```

### 6.2 校验规则

| 参数 | 约束 |
|------|------|
| `host` | 非空 |
| `port` | 1–65535 |
| `channel_signs` | 6 个, 每个 ∈ {−1, +1} |
| `cutoff_hz` | > 0, ≤ 60 |
| `deadzone_force_n/torque_nm` | ≥ 0 |
| `gain_force/torque` | > 0 |
| `vmax_pos_mm_s` | > 0, `vmax × cycle_ms ≤ 35` |
| `vmax_rot_deg_s` | > 0, `vmax × cycle_ms ≤ 35°` |
| `stale_timeout_ms` | > 0 |

---

## 7. 异常处理

### 7.1 故障分级

沿用现有三态: 告警(可自愈) → Fault(需确认) → KRC 硬限(停机)

### 7.2 传感器层

| 故障 | 级别 | 检测 | 恢复 |
|------|------|------|------|
| SRI 连接断开 | Fault | 连续 `stale_timeout_ms/cycle_ms` 帧无新帧 | 重连后手动使能 |
| 力超量程 >70% | 告警 | 窗口均值 > capacity × 0.7 | 自愈 |
| 力超量程 >95% | Fault | 窗口均值 > capacity × 0.95 | 手动复位 |
| 非法帧(NaN/Inf) | 静默丢弃 | `isfinite` 检查 | 不计入 stale |

### 7.3 控制层

| 故障 | 级别 | 检测 | 恢复 |
|------|------|------|------|
| 力控超速 | Fault | Δpose 被 vmax 限幅连续 10 帧 | 手动复位 |
| 台账累积超限 | 告警 | `‖m_cmd‖` 接近 KRC 硬限 | 归零 |
| 方向全部关闭 | UI 提示 | `en_*` 全部 false | 红色边框 |

### 7.4 通信层

现有看门狗、积压排空、帧回退检测在力控模式下继续工作。

---

## 8. UI 布局

### 8.1 页面结构

```
QMainWindow
  ├── 工具栏: [监听] [位姿跟踪] [力控制] [归零] [故障复位]
  ├── 状态栏 (通用)
  ├── QStackedWidget
  │   ├── 页面 0: 位姿跟踪 (现有七个 dock 面板)
  │   └── 页面 1: 力控制 (新面板)
  └── 通信指标面板 (共用)
```

### 8.2 力控制页面

```
┌──────────────────────────────────────────────────────────────┐
│  [传感器状态]               [连接] [断开] [力清零]            │
│  IP: 192.168.0.108:4008     ●已连接  │F│= 12.3N              │
├──────────────────────┬───────────────────────────────────────┤
│ 滤波器               │  力曲线 (30s 窗口)                     │
│ 截止频率: 10 Hz ←──→│                                       │
│                      │  Fx Fy Fz (N)     [原始/滤波] 切换    │
│ 死区                 │  ─────────────────────────────────    │
│ 力: 5.0 N           │                                       │
│ 力矩: 1.0 Nm        │  力图表区域                            │
│                      │                                       │
│ 导纳                 │  ─────────────────────────────────    │
│ 力增益: 0.05        │  Mx My Mz (Nm)     [原始/滤波] 切换    │
│ 力矩增益: 0.5       │                                       │
│ vmax: 5.0 mm/s     │  力矩图表区域                          │
│       1.0 deg/s     │                                       │
│                      │  ─────────────────────────────────    │
│ 方向使能             │  当前力矢量:  15.2 N  (0.3,0.1,15.1)  │
│ ☐X ☐Y ☑Z           │  当前力矩矢量: 0.8 Nm  (0.1,0.0,0.8)  │
│ ☐A ☐B ☐C           │                                       │
│                      │                                       │
│ [力控使能] [停止]    │                                       │
└──────────────────────┴───────────────────────────────────────┘
```

### 8.3 力曲线

- 30s 窗口，与现有 10s 误差图表区分
- 力和力矩各一个图表区域
- 每个图表可独立切换"原始力/滤波力"视图
- 数据通过 SharedState 从通信线程传递到 UI 线程

---

## 9. 数据结构

### 9.1 WrenchFrame

```cpp
struct WrenchFrame {
    double fx, fy, fz;   // 力 (N)
    double mx, my, mz;   // 力矩 (Nm)
    bool fresh = false;  // 本轮有新数据
};
```

### 9.2 StatusSnapshot 扩展

```cpp
// 新增字段
WrenchFrame wrenchRaw;       // 原始力 (降采样后)
WrenchFrame wrenchFiltered;  // 滤波力
WrenchFrame wrenchBias;      // 清零偏置
bool forceControlActive = false;
double forceVectorNorm = 0.0;   // 合力矢量大小
double torqueVectorNorm = 0.0;  // 合力矩矢量大小
```

### 9.3 ChartSample 扩展

```cpp
struct ForceChartSample {
    double tSec = 0.0;
    double fx, fy, fz;   // 原始力
    double mx, my, mz;   // 原始力矩
    double ffx, ffy, ffz; // 滤波力
    double fmx, fmy, fmz; // 滤波力矩
};
```

---

## 10. 测试策略

### 10.1 单元测试

| 模块 | 测试内容 |
|------|----------|
| `SriProtocol` | 帧头检测、不完整帧缓冲、NaN/Inf 丢弃、字节流边界 |
| `ForceFilter` | 阶跃响应时间、稳态增益、截止频率验证 |
| `ForceController` | 死区抑制、sigmoid 输出范围、方向混合矩阵、vmax 限幅 |

### 10.2 集成测试

| 场景 | 验证点 |
|------|--------|
| SRI 断连重连 | stale 计数、Fault 产生、自动退出力控 |
| 力清零 | 静止检测、偏置合理性、清零后净力归零 |
| 模式切换 | 互斥保证、台账重对齐、页面切换 |
| 力超量程 | 告警产生与消除 |
| 力控超速 | vmax 限幅、连续限幅 Fault |

### 10.3 UI 测试

| 场景 | 验证点 |
|------|--------|
| 页面切换 | QStackedWidget 正确切换、dock 可见性 |
| 参数编辑 | 校验规则生效、非法输入拦截 |
| 力曲线 | 原始/滤波切换、30s 滚动、缩放 |
| 方向使能 | CheckBox 状态与配置同步 |
| 力控使能/停止 | 按钮状态、状态栏更新 |

### 10.4 系统测试（真机）

| 场景 | 验证点 |
|------|--------|
| 空载力噪声 | 静止时滤波后力波动 < 死区半径 |
| 柔顺接触 | 缓慢靠近工件，力控响应平滑、无抖动 |
| 侧向力 | 非力控方向的力不产生位移 |
| 断连测试 | 拔网线后力控退出、重连后恢复 |
| 长时间运行 | 台账累积在限值内、无漂移 |

---

## 11. 不包含的范围

- 动态姿态下的重力在线补偿（后续预留）
- ROS2 集成（本程序为独立上位机）
- 力/位混合控制（力控和位置跟踪互斥，不同时启用）
- 力传感器的自动标定流程（手动输入配置）
