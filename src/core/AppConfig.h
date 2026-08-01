#pragma once
#include <QMetaType>
#include <QString>

struct AppConfig
{
    QString listenIp          = "192.168.44.1";
    quint16 listenPort        = 59152;

    double  cycleMs           = 12.0;
    QString senType           = "ImFree";
    int     watchdogMissLimit = 3;

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

    int     refreshMs         = 33;
    int     chartWindowS      = 10;

    static AppConfig defaults() { return AppConfig{}; }

    // 未出现的字段保留 out 中原有值（即默认值）
    static bool loadFromFile(const QString &path, AppConfig *out,
                             QString *error);
};

// 必需：AppConfig 会通过 Q_ARG 跨线程排队传递（Task 10 的 applyConfig），
// 未注册元类型会导致队列连接在运行时静默失败。
Q_DECLARE_METATYPE(AppConfig)
