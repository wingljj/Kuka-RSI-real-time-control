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
};
QTEST_MAIN(TestSessionGuard)
#include "test_session_guard.moc"
