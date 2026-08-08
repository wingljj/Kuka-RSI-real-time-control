#pragma once
#include <QMetaType>

// 六维力/力矩采样帧（力控数据层基元）。fresh=false 表示帧未更新
//（传感器掉线、超时或首帧之前），消费方必须以此做陈旧判定。
struct WrenchFrame
{
    double fx = 0.0;   // N
    double fy = 0.0;   // N
    double fz = 0.0;   // N
    double mx = 0.0;   // Nm
    double my = 0.0;   // Nm
    double mz = 0.0;   // Nm
    bool   fresh = false;
};

// 跨线程排队传递（信号/槽或 Q_ARG）需要注册元类型，否则运行时静默失败。
Q_DECLARE_METATYPE(WrenchFrame)
