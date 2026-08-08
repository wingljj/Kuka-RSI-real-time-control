#pragma once
#include <QMetaType>
#include <QString>
#include <array>

// ── 力控（force control）配置块 ──
// 传感器、安装、滤波器/死区/导纳参数、启用轴，分别对应 config.json
// 中 force_control 块的各子对象；缺失字段一律保留默认值。

struct ForceSensorConfig
{
    QString host                  = "192.168.0.108";
    quint16 port                  = 4008;
    std::array<int, 6> channelSigns = {1, 1, 1, 1, 1, 1};
    double  torqueScale           = 1.0;
    double  forceCapacityN[3]     = {7200.0, 7200.0, 18000.0};
    double  torqueCapacityNm[3]   = {1400.0, 1400.0, 1400.0};
    double  capacityWarningRatio  = 0.70;
    double  staleTimeoutMs        = 100.0;
};

struct MountingConfig
{
    double flangeTSensor[6] = {0.0, 0.0, 85.0, 0.0, 0.0, 0.0};  // XYZ mm, ABC deg
    double flangeTTool[6]   = {0.0, 0.0, 150.0, 0.0, 0.0, 0.0}; // XYZ mm, ABC deg
};

struct ForceControlAxes
{
    bool enX = false;
    bool enY = false;
    bool enZ = true;
    bool enA = false;
    bool enB = false;
    bool enC = false;
};

struct ForceControlParams
{
    double cutoffHz         = 10.0;
    double deadzoneForceN   = 5.0;
    double deadzoneTorqueNm = 1.0;
    double gainForce        = 0.05;
    double gainTorque       = 0.5;
    double vmaxPosMmS       = 5.0;
    double vmaxRotDegS      = 1.0;
};

struct ForceControlConfig
{
    ForceSensorConfig sensor;
    MountingConfig    mounting;
    ForceControlParams params;
    ForceControlAxes  axes;
};

struct AppConfig
{
    QString listenIp          = "192.168.44.1";
    quint16 listenPort        = 59152;

    double  cycleMs           = 12.0;
    QString senType           = "ImFree";
    // 连续丢包多少帧转 Fault。25 帧 ≈ 100ms（4ms 周期），与 KRC 侧 Timeout
    // （100 周期）同量级；旧值 3（12ms）对真实网络过紧，一次普通抖动即停跟踪。
    int     watchdogMissLimit = 25;

    // 联锁与运行时保护（见 SessionGuard）：
    int     krcTimeoutCycles      = 100;       // KRC ETHERNET Timeout（周期数）
    double  krcPoscorrLimitPosMm  = 25.0;      // KRC POSCORR 位置累积限值
    double  krcPoscorrLimitRotDeg = 25.0;      // KRC POSCORR 姿态累积限值
    int     rxBufferBytes         = 1048576;   // socket 接收缓冲（字节）

    // 判定"这是一个全新的 RSI 会话"所需的静默时长。必须显著大于 KRC 侧
    // ETHERNET 对象的 Timeout（计划值 100 个 IPO 周期，12ms 周期下约 1200ms）：
    // 只要主机的判定阈值低于 KRC 的容忍度，就存在一个窗口——KRC 认为会话
    // 从未中断、仍按原始起始位姿累计修正，而主机已把安全锚点移到当前位置
    // 并发放了一份全新的预算。一次 300ms 的调度停顿即可触发，且可重复。
    // 与看门狗间隔（用于"连接丢失"显示）是两个不同的问题，不可共用一个值。
    double  sessionGapMs      = 2000.0;

    double  targetTrajectoryMs = 1000.0; // 目标轨迹时长 ms（0 = 立即完成 = 直通）

    double  kpPos             = 0.30;
    double  kpRot             = 0.30;
    double  vmaxPosMmS        = 50.0;
    double  vmaxRotDegS       = 10.0;
    double  accumLimitPosMm   = 30.0;
    double  accumLimitRotDeg  = 15.0;

    // 反馈异常剔除：单帧位置/旋转跳变超物理极限（v_max × dt）判为陈旧帧
    //（回零增量 + 计数），连续 staleFrameLimit 帧超限 → Fault（仅在 Tracking
    // 下；非 Tracking 只累计不 Fault，下一帧不超限即清零自愈）
    double  physVmaxPosMmS    = 500.0;
    double  physVmaxRotDegS   = 60.0;
    int     staleFrameLimit   = 10;

    // ── 到位精修 settle-and-trim(2026-08-07,界面可勾选)──
    // 停稳后测残差(target − RIst),在窗口内则把台账重对齐到实测、让残差
    // 重新流经正常管线补发。离散迭代(限次限频、只在静止时触发),与 4ms
    // 控制环隔三个数量级时间尺度,不构成连续反馈、不引入振荡。
    bool    trimEnabled       = false;
    double  trimMinMm         = 0.02;   // 低于此值视为已到位
    double  trimMaxMm         = 2.0;    // 高于此值视为被物理挡住,不修(需人查)
    double  trimMinDeg        = 0.02;
    double  trimMaxDeg        = 2.0;
    double  trimSettleMs      = 200.0;  // 增量连续静默满此时长才算"停稳"
    double  trimCooldownMs    = 1000.0; // 两次精修的最小间隔
    int     trimMaxAttempts   = 3;      // 同一目标最多修几次(修不动=报警停手)

    // 轨迹巡航速度上限(>0 启用):时长自动拉长使五次多项式峰值速度
    //(1.875×平均)不超过它——远目标不再退化成"vmax 饱和爬行"。0 = 固定时长。
    double  targetCruiseMmS   = 0.0;
    double  targetCruiseDegS  = 0.0;

    int     refreshMs         = 33;
    int     chartWindowS      = 10;
    double  trackingQualityWarnPct     = 0.5;   // 跟踪质量警告阈值（误差/累积 占限值比例）
    double  trackingQualityCriticalPct = 0.8;   // 跟踪质量严重阈值

    ForceControlConfig forceControl;  // 力控配置；默认值由 ForceControlConfig{} 提供
    bool forceControlEnabled = false;  // runtime flag, not persisted（UI 切换力控页面时置位）

    static AppConfig defaults() { return AppConfig{}; }

    // 未出现的字段保留 out 中原有值（即默认值）
    static bool loadFromFile(const QString &path, AppConfig *out,
                             QString *error);
};

// 必需：AppConfig 会通过 Q_ARG 跨线程排队传递（Task 10 的 applyConfig），
// 未注册元类型会导致队列连接在运行时静默失败。
Q_DECLARE_METATYPE(AppConfig)
