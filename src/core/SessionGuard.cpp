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

    // ── 增益/限值防线(2026-08-06 审查:配置输入曾完全不设防)──────────
    // kp 的数学发散界是 2(台账闭环极点 1−kp),工程上限取 1:kp>1 意味着
    // 单帧就要求走完全部误差,除了顶格限幅没有任何意义,不留这个边。
    // kp≤0:负值让机器人背向目标满速运动(范数限幅保方向只限幅值),
    // 零值则 Tracking 显示正常却永远发 0——两者都是"界面全绿的失控"。
    if (!(cfg.kpPos > 0.0 && cfg.kpPos <= 1.0))
        out << QStringLiteral("kp_pos=%1 must be in (0, 1]").arg(cfg.kpPos);
    if (!(cfg.kpRot > 0.0 && cfg.kpRot <= 1.0))
        out << QStringLiteral("kp_rot=%1 must be in (0, 1]").arg(cfg.kpRot);

    // miss/stale 限值非正会使对应保护"使能即触发",形同禁用跟踪。
    if (!(cfg.watchdogMissLimit > 0))
        out << QStringLiteral("watchdog_miss_limit=%1 must be > 0")
                .arg(cfg.watchdogMissLimit);
    if (!(cfg.staleFrameLimit > 0))
        out << QStringLiteral("stale_frame_limit=%1 must be > 0")
                .arg(cfg.staleFrameLimit);

    // 物理极限非正会把所有运动判为 stale 帧。
    if (!(cfg.physVmaxPosMmS > 0.0))
        out << QStringLiteral("phys_vmax_pos_mm_s=%1 must be > 0")
                .arg(cfg.physVmaxPosMmS);
    if (!(cfg.physVmaxRotDegS > 0.0))
        out << QStringLiteral("phys_vmax_rot_deg_s=%1 must be > 0")
                .arg(cfg.physVmaxRotDegS);

    // vmax 非正 = Tracking 下永远发 0 增量的静默停摆。
    if (!(cfg.vmaxPosMmS > 0.0))
        out << QStringLiteral("vmax_pos_mm_s=%1 must be > 0").arg(cfg.vmaxPosMmS);
    if (!(cfg.vmaxRotDegS > 0.0))
        out << QStringLiteral("vmax_rot_deg_s=%1 must be > 0").arg(cfg.vmaxRotDegS);

    // 单帧步长不得超过 KRC 侧 Limit 对象的 ±35mm/帧:超过后 KRC 静默削波,
    // 指令台账与真实设定值从此发散(主机毫无察觉)。
    constexpr double kKrcFrameLimitMm = 35.0;
    const double stepMm = cfg.vmaxPosMmS * cfg.cycleMs / 1000.0;
    if (cfg.cycleMs > 0.0 && stepMm > kKrcFrameLimitMm)
        out << QStringLiteral(
            "vmax_pos_mm_s(%1) × cycle_ms(%2) = %3 mm/frame exceeds the KRC "
            "Limit object ±35 mm; corrections would be silently clipped")
                .arg(cfg.vmaxPosMmS)
                .arg(cfg.cycleMs)
                .arg(stepMm);

    // ── 到位精修(2026-08-07):仅启用时校验,关着的错参数不拦人 ──
    if (cfg.trimEnabled) {
        // 窗口为空 = 永不精修却显示已启用,是静默失效
        if (!(cfg.trimMinMm > 0.0 && cfg.trimMinMm < cfg.trimMaxMm))
            out << QStringLiteral("trim_min_mm(%1) must be in (0, trim_max_mm=%2)")
                    .arg(cfg.trimMinMm).arg(cfg.trimMaxMm);
        if (!(cfg.trimMinDeg > 0.0 && cfg.trimMinDeg < cfg.trimMaxDeg))
            out << QStringLiteral("trim_min_deg(%1) must be in (0, trim_max_deg=%2)")
                    .arg(cfg.trimMinDeg).arg(cfg.trimMaxDeg);
        // 不等停稳就修 = 连续反馈,重蹈滞后振荡;不限次 = 顶死时无限重试
        if (!(cfg.trimSettleMs > 0.0))
            out << QStringLiteral("trim_settle_ms(%1) must be > 0").arg(cfg.trimSettleMs);
        if (!(cfg.trimCooldownMs > 0.0))
            out << QStringLiteral("trim_cooldown_ms(%1) must be > 0").arg(cfg.trimCooldownMs);
        if (!(cfg.trimMaxAttempts > 0))
            out << QStringLiteral("trim_max_attempts(%1) must be > 0").arg(cfg.trimMaxAttempts);
    }

    // 巡航速度必须留在 vmax 以内,否则轨迹必然饱和,自适应时长失去意义
    if (cfg.targetCruiseMmS < 0.0 || cfg.targetCruiseMmS > cfg.vmaxPosMmS)
        out << QStringLiteral(
            "target_cruise_mm_s(%1) must be in [0, vmax_pos_mm_s=%2]")
                .arg(cfg.targetCruiseMmS).arg(cfg.vmaxPosMmS);
    if (cfg.targetCruiseDegS < 0.0 || cfg.targetCruiseDegS > cfg.vmaxRotDegS)
        out << QStringLiteral(
            "target_cruise_deg_s(%1) must be in [0, vmax_rot_deg_s=%2]")
                .arg(cfg.targetCruiseDegS).arg(cfg.vmaxRotDegS);

    // ── 力控参数校验(2026-08-08):默认值安全,始终校验(与 trim 的"仅启用时
    // 校验"不同——力控参数即使当前未启用也必须在配置加载时就是合法的)────
    {
        const auto &fc = cfg.forceControl;
        if (fc.sensor.host.trimmed().isEmpty())
            out << QStringLiteral("force_control.sensor.host is empty");
        if (fc.sensor.port < 1 || fc.sensor.port > 65535)
            out << QStringLiteral("force_control.sensor.port out of range");
        for (int i = 0; i < 6; ++i) {
            if (fc.sensor.channelSigns[i] != -1 && fc.sensor.channelSigns[i] != 1)
                out << QStringLiteral("force_control.sensor.channel_signs[%1] must be -1 or 1").arg(i);
        }
        if (fc.params.cutoffHz <= 0.0 || fc.params.cutoffHz > 60.0)
            out << QStringLiteral("force_control.filter.cutoff_hz must be in (0, 60]");
        if (fc.params.deadzoneForceN < 0.0)
            out << QStringLiteral("force_control.deadzone.force_n must be >= 0");
        if (fc.params.deadzoneTorqueNm < 0.0)
            out << QStringLiteral("force_control.deadzone.torque_nm must be >= 0");
        if (fc.params.gainForce <= 0.0)
            out << QStringLiteral("force_control.admittance.gain_force must be > 0");
        if (fc.params.gainTorque <= 0.0)
            out << QStringLiteral("force_control.admittance.gain_torque must be > 0");
        if (fc.params.vmaxPosMmS <= 0.0)
            out << QStringLiteral("force_control.admittance.vmax_pos_mm_s must be > 0");
        if (fc.params.vmaxRotDegS <= 0.0)
            out << QStringLiteral("force_control.admittance.vmax_rot_deg_s must be > 0");
        if (fc.sensor.staleTimeoutMs <= 0.0)
            out << QStringLiteral("force_control.sensor.stale_timeout_ms must be > 0");

        // vmax × cycle_ms ≤ 35mm constraint（每帧步长,与上方位姿 vmax 校验
        // 同式: mm/s × ms / 1000 = mm/帧;KRC Limit 对象超限后静默削波）
        const double maxStepPos = fc.params.vmaxPosMmS * cfg.cycleMs / 1000.0;
        if (maxStepPos > 35.0)
            out << QStringLiteral("force_control vmax_pos × cycle_ms = %1 mm exceeds KRC 35mm limit").arg(maxStepPos);
        const double maxStepRot = fc.params.vmaxRotDegS * cfg.cycleMs / 1000.0;
        if (maxStepRot > 35.0)
            out << QStringLiteral("force_control vmax_rot × cycle_ms = %1 deg exceeds KRC 35deg limit").arg(maxStepRot);
    }

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
