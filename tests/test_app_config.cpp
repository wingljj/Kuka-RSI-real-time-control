#include <QtTest>
#include <QTemporaryFile>
#include "core/AppConfig.h"

class TestAppConfig : public QObject
{
    Q_OBJECT
private slots:
    void defaults_matchSpec()
    {
        const AppConfig c = AppConfig::defaults();
        QCOMPARE(c.kpPos, 0.30);
        QCOMPARE(c.kpRot, 0.30);
        QCOMPARE(c.vmaxPosMmS, 50.0);
        QCOMPARE(c.vmaxRotDegS, 10.0);
        QCOMPARE(c.accumLimitPosMm, 30.0);
        QCOMPARE(c.accumLimitRotDeg, 15.0);
        QCOMPARE(c.watchdogMissLimit, 3);
    }

    void load_readsAllFields()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({
          "network": { "listen_ip": "10.0.0.5", "listen_port": 12345 },
          "rsi": { "cycle_ms": 4.0, "sen_type": "MyType",
                   "watchdog_miss_limit": 7 },
          "control": { "kp_pos": 0.1, "kp_rot": 0.2,
                       "vmax_pos_mm_s": 11.0, "vmax_rot_deg_s": 22.0,
                       "accum_limit_pos_mm": 33.0,
                       "accum_limit_rot_deg": 44.0 },
          "ui": { "refresh_ms": 50, "chart_window_s": 60 }
        })");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err),
                 qPrintable(err));
        QCOMPARE(c.listenIp, QString("10.0.0.5"));
        QCOMPARE(c.listenPort, quint16(12345));
        QCOMPARE(c.cycleMs, 4.0);
        QCOMPARE(c.senType, QString("MyType"));
        QCOMPARE(c.watchdogMissLimit, 7);
        QCOMPARE(c.kpPos, 0.1);
        QCOMPARE(c.accumLimitRotDeg, 44.0);
        QCOMPARE(c.refreshMs, 50);
        QCOMPARE(c.chartWindowS, 60);
    }

    void load_readsSessionGapMs()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({ "rsi": { "session_gap_ms": 3500.0 } })");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err), qPrintable(err));
        QCOMPARE(c.sessionGapMs, 3500.0);
    }

    void defaults_sessionGapExceedsKrcTimeout()
    {
        // KRC 的 ETHERNET Timeout 计划值为 100 个 IPO 周期；12ms 周期下 1200ms。
        // 会话判定阈值必须显著高于它，否则主机会在 KRC 仍认为会话连续时
        // 移动安全锚点，凭空发放一份新的修正预算。
        const AppConfig c = AppConfig::defaults();
        QVERIFY(c.sessionGapMs > 100.0 * c.cycleMs);
    }

    void load_missingFieldKeepsDefault()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({ "control": { "kp_pos": 0.9 } })");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY(AppConfig::loadFromFile(f.fileName(), &c, &err));
        QCOMPARE(c.kpPos, 0.9);                        // 覆盖
        QCOMPARE(c.vmaxPosMmS, 50.0);                  // 保留默认
        QCOMPARE(c.listenIp, QString("192.168.44.1")); // 保留默认
    }

    void load_malformedJsonReportsError()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write("{ this is not json ");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY(!AppConfig::loadFromFile(f.fileName(), &c, &err));
        QVERIFY(!err.isEmpty());
    }

    void load_missingFileReportsError()
    {
        AppConfig c;
        QString err;
        QVERIFY(!AppConfig::loadFromFile("Z:/nonexistent.json", &c, &err));
        QVERIFY(!err.isEmpty());
    }

    void load_wrongTypeKeepsDefault()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({
          "network": { "listen_ip": 12345, "listen_port": "59152" },
          "rsi":     { "watchdog_miss_limit": "7",
                       "session_gap_ms": "3500" },
          "control": { "kp_pos": "0.9" },
          "ui":      { "refresh_ms": null }
        })");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err), qPrintable(err));
        // 类型不符的字段必须保留默认值，而不是被静默写成 0 或空
        QCOMPARE(c.listenIp, QString("192.168.44.1"));
        QCOMPARE(c.listenPort, quint16(59152));
        QCOMPARE(c.watchdogMissLimit, 3);
        QCOMPARE(c.sessionGapMs, 2000.0);
        QCOMPARE(c.kpPos, 0.30);
        QCOMPARE(c.refreshMs, 33);
    }

    void load_outOfRangePortKeepsDefault()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({ "network": { "listen_port": 70000 } })");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err), qPrintable(err));
        QCOMPARE(c.listenPort, quint16(59152));   // 不得被 quint16 截断成 4464
    }

    void load_nonObjectRootIsError()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write("[1,2,3]");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY(!AppConfig::loadFromFile(f.fileName(), &c, &err));
        QVERIFY(!err.isEmpty());
    }

    void load_successClearsError()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({ "control": { "kp_pos": 0.5 } })");
        f.flush();

        AppConfig c;
        QString err = "stale text from a previous call";
        QVERIFY(AppConfig::loadFromFile(f.fileName(), &c, &err));
        QVERIFY(err.isEmpty());
    }
};

QTEST_MAIN(TestAppConfig)
#include "test_app_config.moc"
