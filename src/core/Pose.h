#pragma once
#include <QMetaType>
#include <cmath>

struct Pose
{
    double x = 0.0;   // mm
    double y = 0.0;   // mm
    double z = 0.0;   // mm
    double a = 0.0;   // deg
    double b = 0.0;   // deg
    double c = 0.0;   // deg
};

// 归一化到 (-180, 180]
inline double wrap180(double deg)
{
    double r = std::fmod(deg + 180.0, 360.0);
    if (r <= 0.0)
        r += 360.0;
    return r - 180.0;
}

// 逐分量相减；姿态分量取最短角路径
inline Pose poseSub(const Pose &lhs, const Pose &rhs)
{
    return Pose{
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z,
        wrap180(lhs.a - rhs.a),
        wrap180(lhs.b - rhs.b),
        wrap180(lhs.c - rhs.c),
    };
}

// 必需：Pose 会通过 Q_ARG 跨线程排队传递（Task 10/12），
// 未注册元类型会导致队列连接在运行时静默失败。
Q_DECLARE_METATYPE(Pose)
