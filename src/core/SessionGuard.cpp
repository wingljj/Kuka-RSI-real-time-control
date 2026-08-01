#include "core/SessionGuard.h"

#include <cmath>

QStringList SessionGuard::staticChecks(const AppConfig &cfg)
{
    QStringList out;

    if (!(cfg.cycleMs > 0.0))
        out << QStringLiteral("cycle_ms=%1 must be > 0").arg(cfg.cycleMs);

    if (!(cfg.sessionGapMs > 0.0))
        out << QStringLiteral("session_gap_ms=%1 must be > 0").arg(cfg.sessionGapMs);

    // krc_timeout_cycles 必须为正，否则与 cycle_ms 的乘积为负/零，下面的
    // session_gap 比较会被恒真地绕过，联锁静默失效。
    if (!(cfg.krcTimeoutCycles > 0))
        out << QStringLiteral("krc_timeout_cycles=%1 must be > 0")
                .arg(cfg.krcTimeoutCycles);

    // 会话判定阈值必须大于 KRC 的容忍度，否则存在窗口：KRC 认为会话未断、
    // 仍按原始起始位姿累计修正，而主机已把安全锚点移到当前位置并发放新预算。
    if (!(cfg.sessionGapMs > cfg.krcTimeoutCycles * cfg.cycleMs))
        out << QStringLiteral(
            "session_gap_ms=%1 must exceed krc_timeout_cycles(%2) × cycle_ms(%3) = %4")
                .arg(cfg.sessionGapMs)
                .arg(cfg.krcTimeoutCycles)
                .arg(cfg.cycleMs)
                .arg(cfg.krcTimeoutCycles * cfg.cycleMs);

    // 【第 2 层已按用户决定移除（2026-08-01）】：accumLimit 不再参与联锁。
    // KRC 侧层 4/5（POSCORR ±25 / POSCORRMON 45）是唯一兜底。

    if (cfg.senType.trimmed().isEmpty())
        out << QStringLiteral("sen_type must be non-empty");

    return out;
}

QStringList SessionGuard::enableChecks(const AppConfig &cfg, double measuredCycleMs)
{
    QStringList out = staticChecks(cfg);
    // 实测周期与配置周期偏差 > 10% 即拦。首帧后才有实测值；未收到帧时
    // measuredCycleMs 保持 0，调用方传入 -1 表示"尚无实测"，不参与判定。
    if (measuredCycleMs > 0.0 && cfg.cycleMs > 0.0) {
        const double tol = 0.10 * cfg.cycleMs;
        if (std::fabs(measuredCycleMs - cfg.cycleMs) > tol)
            out << QStringLiteral(
                "measured cycle %1 ms deviates from configured %2 ms by more than 10%%")
                    .arg(measuredCycleMs)
                    .arg(cfg.cycleMs);
    }
    return out;
}
