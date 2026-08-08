#include <QtTest>
#include "core/SessionGuard.h"

namespace {

// 通过全部静态联锁的基准配置。accumLimit 已不参与联锁（第 2 层移除，2026-08-01），
// 这里仍显式给出 20/20 仅供 UI 显示参考。
AppConfig good()
{
    AppConfig c = AppConfig::defaults();
    c.cycleMs           = 12.0;
    c.sessionGapMs      = 2000.0;    // > 100 × 12 = 1200
    c.accumLimitPosMm   = 20.0;      // 仅 UI 显示参考
    c.accumLimitRotDeg  = 20.0;      // 仅 UI 显示参考
    c.senType           = "ImFree";
    return c;
}

} // namespace

class TestSessionGuard : public QObject
{
    Q_OBJECT
private slots:
    void goodConfig_passes()
    {
        QVERIFY(SessionGuard::staticChecks(good()).isEmpty());
        QVERIFY(SessionGuard::enableChecks(good(), 12.5).isEmpty());
    }

    void cycleZero_fails()
    {
        AppConfig c = good();
        c.cycleMs = 0.0;
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void sessionGapZero_fails()
    {
        AppConfig c = good();
        c.sessionGapMs = 0.0;
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("session_gap_ms"));
    }

    void sessionGapBelowTimeoutProduct_fails()
    {
        AppConfig c = good();
        c.sessionGapMs = 500.0;      // < 100 × 12 = 1200
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void krcTimeoutCyclesNegative_fails()
    {
        AppConfig c = good();
        c.krcTimeoutCycles = -100;
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("krc_timeout_cycles"));
    }

    void senTypeEmpty_fails()
    {
        AppConfig c = good();
        c.senType = "   ";
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void measuredCycleDeviation_fails()
    {
        AppConfig c = good();
        const QStringList r = SessionGuard::enableChecks(c, 15.0);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("cycle"));
    }

    void measuredCycleWithinTolerance_passes()
    {
        AppConfig c = good();
        QVERIFY(SessionGuard::enableChecks(c, 12.6).isEmpty());   // 差 0.6 < 1.2
    }

    void measuredCycleUnknown_doesNotFail()
    {
        AppConfig c = good();
        QVERIFY(SessionGuard::enableChecks(c, -1.0).isEmpty());   // 无实测不拦
    }

    // ── 增益/限值防线(2026-08-06 审查发现:配置输入不设防)────────────
    // kp<0:增量与误差反向,机器人以 vmax 满速背向目标,直到撞 KRC 硬限;
    // kp>2:台账闭环极点 |1−kp|>1 发散,被单帧限幅截成 vmax 满幅极限环,
    // 即用配置重现 2026-08-04 的真机抖动;kp=0:Tracking 显示正常却纹丝不动。
    void kpZero_fails()
    {
        AppConfig c = good();
        c.kpPos = 0.0;
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("kp_pos"));
    }

    void kpNegative_fails()
    {
        AppConfig c = good();
        c.kpRot = -0.3;
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("kp_rot"));
    }

    void kpAboveOne_fails()
    {
        AppConfig c = good();
        c.kpPos = 1.5;              // 工程上限 1.0(数学发散界 2.0,不留边)
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void kpExactlyOne_passes()
    {
        AppConfig c = good();
        c.kpPos = 1.0;
        c.kpRot = 1.0;
        QVERIFY(SessionGuard::staticChecks(c).isEmpty());
    }

    void missLimitZero_fails()
    {
        AppConfig c = good();
        c.watchdogMissLimit = 0;    // m_missed >= 0 恒真,使能即 Fault
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("watchdog_miss_limit"));
    }

    void staleLimitZero_fails()
    {
        AppConfig c = good();
        c.staleFrameLimit = 0;
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void physVmaxZero_fails()
    {
        AppConfig c = good();
        c.physVmaxPosMmS = 0.0;     // 任何运动都会被判 stale
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void vmaxZeroOrNegative_fails()
    {
        AppConfig c = good();
        c.vmaxPosMmS = 0.0;         // Tracking 显示正常却永远发 0 增量
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void stepBeyondKrcFrameLimit_fails()
    {
        AppConfig c = good();
        c.vmaxPosMmS = 4000.0;      // 4000×0.012 = 48mm/帧 > KRC Limit ±35mm
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("35"));
    }

    // ── 到位精修参数(2026-08-07):仅启用时校验 ──
    void trimWindowInverted_failsOnlyWhenEnabled()
    {
        AppConfig c = good();
        c.trimMinMm = 3.0;          // min > max:窗口为空,永不精修却显示已启用
        c.trimMaxMm = 1.0;
        QVERIFY(SessionGuard::staticChecks(c).isEmpty());   // 未启用:不拦
        c.trimEnabled = true;
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("trim"));
    }

    void trimNonPositiveThrottle_failsWhenEnabled()
    {
        AppConfig c = good();
        c.trimEnabled = true;
        c.trimSettleMs = 0.0;       // 不等停稳就修 = 连续反馈,重蹈抖动
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
        c = good();
        c.trimEnabled = true;
        c.trimMaxAttempts = 0;
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void cruiseAboveVmax_fails()
    {
        AppConfig c = good();
        c.targetCruiseMmS = 100.0;  // > vmax(50):轨迹必然饱和,自适应失去意义
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("cruise"));
    }

    void cruiseZero_passes()
    {
        AppConfig c = good();
        c.targetCruiseMmS = 0.0;    // 0 = 固定时长,旧行为
        QVERIFY(SessionGuard::staticChecks(c).isEmpty());
    }

    // ── 力控参数校验(2026-08-08):默认值安全,始终校验 ──
    void forceSensorHostEmpty_fails()
    {
        AppConfig c = good();
        c.forceControl.sensor.host = "";
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("force_control.sensor.host"));
    }

    void forceSensorPortZero_fails()
    {
        AppConfig c = good();
        c.forceControl.sensor.port = 0;
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("force_control.sensor.port"));
    }

    void forceCutoffHzOutOfRange_fails()
    {
        AppConfig c = good();
        c.forceControl.params.cutoffHz = 0.0;   // 下限:不滤波=直通高频噪声
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
        c = good();
        c.forceControl.params.cutoffHz = 100.0; // 上限 60Hz:超过则 Butterworth 参数无意义
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void forceVmaxExceedsKrcLimit_fails()
    {
        AppConfig c = good();
        c.forceControl.params.vmaxPosMmS = 4000.0;  // 4000×12/1000 = 48mm/帧 > KRC ±35mm
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("force_control"));
    }

    void forceCycleMsZero_fails()
    {
        // cycle_ms 是滤波器设计采样率的分母：0 会让 fs=1000/0=inf，
        // Butterworth 静默回退直通（不滤波），必须拦截。
        AppConfig c = good();
        c.forceControl.params.cycleMs = 0.0;
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("force_control.params.cycle_ms"));
    }
};
QTEST_MAIN(TestSessionGuard)
#include "test_session_guard.moc"
