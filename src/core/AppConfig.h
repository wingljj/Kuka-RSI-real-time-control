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
