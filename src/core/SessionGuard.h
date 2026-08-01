#pragma once
#include <QString>
#include <QStringList>
#include "core/AppConfig.h"

// 联锁规则，纯函数，无 IO、无状态。返回未通过项的说明列表；为空即全部通过。
class SessionGuard
{
public:
    // 静态联锁：仅依据配置评估，绑定后即可查。
    static QStringList staticChecks(const AppConfig &cfg);

    // 动态联锁：使能跟踪时评估。measuredCycleMs <= 0 表示尚无实测周期（未收到帧）。
    static QStringList enableChecks(const AppConfig &cfg, double measuredCycleMs);
};
