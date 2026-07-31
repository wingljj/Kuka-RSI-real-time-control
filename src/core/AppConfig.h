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

    // 判定"这是一个全新的 RSI 会话"所需的静默时长。必须显著大于 KRC 侧
    // ETHERNET 对象的 Timeout（计划值 100 个 IPO 周期，12ms 周期下约 1200ms）：
    // 只要主机的判定阈值低于 KRC 的容忍度，就存在一个窗口——KRC 认为会话
    // 从未中断、仍按原始起始位姿累计修正，而主机已把安全锚点移到当前位置
    // 并发放了一份全新的预算。一次 300ms 的调度停顿即可触发，且可重复。
    // 与看门狗间隔（用于"连接丢失"显示）是两个不同的问题，不可共用一个值。
    double  sessionGapMs      = 2000.0;

    double  kpPos             = 0.30;
    double  kpRot             = 0.30;
    double  vmaxPosMmS        = 50.0;
    double  vmaxRotDegS       = 10.0;
    double  accumLimitPosMm   = 30.0;
    double  accumLimitRotDeg  = 15.0;

    int     refreshMs         = 33;
    int     chartWindowS      = 20;

    static AppConfig defaults() { return AppConfig{}; }

    // 未出现的字段保留 out 中原有值（即默认值）
    static bool loadFromFile(const QString &path, AppConfig *out,
                             QString *error);
};

// 必需：AppConfig 会通过 Q_ARG 跨线程排队传递（Task 10 的 applyConfig），
// 未注册元类型会导致队列连接在运行时静默失败。
Q_DECLARE_METATYPE(AppConfig)
