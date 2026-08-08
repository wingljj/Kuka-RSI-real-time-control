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
        // 25 帧 ≈ 100ms（4ms 周期），与 KRC 侧 Timeout（100 周期）同量级。
        // 旧值 3（12ms）对真实网络过紧：一次普通抖动就足以停掉跟踪。
        QCOMPARE(c.watchdogMissLimit, 25);
        QCOMPARE(c.targetTrajectoryMs, 1000.0);
        QCOMPARE(c.physVmaxPosMmS, 500.0);
        QCOMPARE(c.physVmaxRotDegS, 60.0);
        QCOMPARE(c.staleFrameLimit, 10);
        // 到位精修(2026-08-07):默认关,窗口/节流参数给保守值
        QCOMPARE(c.trimEnabled, false);
        QCOMPARE(c.trimMinMm, 0.02);
        QCOMPARE(c.trimMaxMm, 2.0);
        QCOMPARE(c.trimMinDeg, 0.02);
        QCOMPARE(c.trimMaxDeg, 2.0);
        QCOMPARE(c.trimSettleMs, 200.0);
        QCOMPARE(c.trimCooldownMs, 1000.0);
        QCOMPARE(c.trimMaxAttempts, 3);
        // 轨迹巡航速度:0 = 固定时长(旧行为)
        QCOMPARE(c.targetCruiseMmS, 0.0);
        QCOMPARE(c.targetCruiseDegS, 0.0);
    }

    void load_readsTrimAndCruiseFields()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({
          "control": { "trim_enabled": true,
                       "trim_min_mm": 0.05, "trim_max_mm": 1.0,
                       "trim_min_deg": 0.1, "trim_max_deg": 3.0,
                       "trim_settle_ms": 300.0, "trim_cooldown_ms": 2000.0,
                       "trim_max_attempts": 5,
                       "target_cruise_mm_s": 5.0,
                       "target_cruise_deg_s": 2.0 }
        })");
        f.flush();
        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err), qPrintable(err));
        QCOMPARE(c.trimEnabled, true);
        QCOMPARE(c.trimMinMm, 0.05);
        QCOMPARE(c.trimMaxMm, 1.0);
        QCOMPARE(c.trimMinDeg, 0.1);
        QCOMPARE(c.trimMaxDeg, 3.0);
        QCOMPARE(c.trimSettleMs, 300.0);
        QCOMPARE(c.trimCooldownMs, 2000.0);
        QCOMPARE(c.trimMaxAttempts, 5);
        QCOMPARE(c.targetCruiseMmS, 5.0);
        QCOMPARE(c.targetCruiseDegS, 2.0);
    }

    void load_readsAllFields()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({
          "network": { "listen_ip": "10.0.0.5", "listen_port": 12345 },
          "rsi": { "cycle_ms": 4.0, "sen_type": "MyType",
                   "watchdog_miss_limit": 7,
                   "target_trajectory_ms": 88.0 },
          "control": { "kp_pos": 0.1, "kp_rot": 0.2,
                       "vmax_pos_mm_s": 11.0, "vmax_rot_deg_s": 22.0,
                       "accum_limit_pos_mm": 33.0,
                       "accum_limit_rot_deg": 44.0,
                       "phys_vmax_pos_mm_s": 1234.0,
                       "phys_vmax_rot_deg_s": 777.0,
                       "stale_frame_limit": 5 },
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
        QCOMPARE(c.targetTrajectoryMs, 88.0);
        QCOMPARE(c.physVmaxPosMmS, 1234.0);
        QCOMPARE(c.physVmaxRotDegS, 777.0);
        QCOMPARE(c.staleFrameLimit, 5);
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
        QCOMPARE(c.watchdogMissLimit, 25);
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

    void loadsForceControlDefaults()
    {
        const AppConfig cfg = AppConfig::defaults();
        QCOMPARE(cfg.forceControl.sensor.host, QString("192.168.0.108"));
        QCOMPARE(cfg.forceControl.sensor.port, quint16(4008));
        QCOMPARE(cfg.forceControl.sensor.channelSigns[0], 1);
        QCOMPARE(cfg.forceControl.sensor.torqueScale, 1.0);
        QCOMPARE(cfg.forceControl.sensor.forceCapacityN[2], 18000.0);
        QCOMPARE(cfg.forceControl.sensor.torqueCapacityNm[1], 1400.0);
        QCOMPARE(cfg.forceControl.sensor.capacityWarningRatio, 0.70);
        QCOMPARE(cfg.forceControl.sensor.staleTimeoutMs, 100.0);
        QCOMPARE(cfg.forceControl.mounting.flangeTSensor[2], 85.0);
        QCOMPARE(cfg.forceControl.mounting.flangeTTool[2], 150.0);
        QCOMPARE(cfg.forceControl.params.cutoffHz, 10.0);
        QCOMPARE(cfg.forceControl.params.deadzoneForceN, 5.0);
        QCOMPARE(cfg.forceControl.params.deadzoneTorqueNm, 1.0);
        QCOMPARE(cfg.forceControl.params.gainForce, 0.05);
        QCOMPARE(cfg.forceControl.params.gainTorque, 0.5);
        QCOMPARE(cfg.forceControl.params.vmaxPosMmS, 5.0);
        QCOMPARE(cfg.forceControl.params.vmaxRotDegS, 1.0);
        QCOMPARE(cfg.forceControl.axes.enZ, true);
        QCOMPARE(cfg.forceControl.axes.enX, false);
        QCOMPARE(cfg.forceControl.axes.enA, false);
    }

    void loadsForceControlFromJson()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({
          "force_control": {
            "sensor": { "host": "10.0.0.1", "port": 5000 },
            "deadzone": { "force_n": 3.0 },
            "axes": { "en_y": true, "en_z": false }
          }
        })");
        f.flush();
        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err), qPrintable(err));
        QCOMPARE(c.forceControl.sensor.host, QString("10.0.0.1"));
        QCOMPARE(c.forceControl.sensor.port, quint16(5000));
        QCOMPARE(c.forceControl.params.deadzoneForceN, 3.0);
        // 未提到的字段保留默认
        QCOMPARE(c.forceControl.params.cutoffHz, 10.0);
        QCOMPARE(c.forceControl.axes.enY, true);
        QCOMPARE(c.forceControl.axes.enZ, false);
    }

    void loadsForceControlFullBlock()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({
          "force_control": {
            "sensor": {
              "host": "10.1.2.3", "port": 6000,
              "channel_signs": [-1, 1, -1, 1, -1, 1],
              "torque_scale": 0.98,
              "force_capacity_n": [1000.0, 2000.0, 3000.0],
              "torque_capacity_nm": [100.0, 200.0, 300.0],
              "capacity_warning_ratio": 0.8,
              "stale_timeout_ms": 50.0
            },
            "mounting": {
              "flange_T_sensor": { "x_mm": 1, "y_mm": 2, "z_mm": 90,
                                   "a_deg": 10, "b_deg": 20, "c_deg": 30 },
              "flange_T_tool":   { "x_mm": 3, "y_mm": 4, "z_mm": 160,
                                   "a_deg": 40, "b_deg": 50, "c_deg": 60 }
            },
            "filter": { "cutoff_hz": 8.0 },
            "deadzone": { "force_n": 2.0, "torque_nm": 0.5 },
            "admittance": { "gain_force": 0.1, "gain_torque": 1.0,
                            "vmax_pos_mm_s": 10.0, "vmax_rot_deg_s": 2.0 },
            "axes": { "en_x": true, "en_y": true, "en_z": false,
                      "en_a": true, "en_b": true, "en_c": true }
          }
        })");
        f.flush();
        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err), qPrintable(err));
        QCOMPARE(c.forceControl.sensor.host, QString("10.1.2.3"));
        QCOMPARE(c.forceControl.sensor.port, quint16(6000));
        QCOMPARE(c.forceControl.sensor.channelSigns[0], -1);
        QCOMPARE(c.forceControl.sensor.channelSigns[2], -1);
        QCOMPARE(c.forceControl.sensor.channelSigns[5], 1);
        QCOMPARE(c.forceControl.sensor.torqueScale, 0.98);
        QCOMPARE(c.forceControl.sensor.forceCapacityN[0], 1000.0);
        QCOMPARE(c.forceControl.sensor.forceCapacityN[2], 3000.0);
        QCOMPARE(c.forceControl.sensor.torqueCapacityNm[1], 200.0);
        QCOMPARE(c.forceControl.sensor.capacityWarningRatio, 0.8);
        QCOMPARE(c.forceControl.sensor.staleTimeoutMs, 50.0);
        QCOMPARE(c.forceControl.mounting.flangeTSensor[0], 1.0);
        QCOMPARE(c.forceControl.mounting.flangeTSensor[2], 90.0);
        QCOMPARE(c.forceControl.mounting.flangeTSensor[3], 10.0);
        QCOMPARE(c.forceControl.mounting.flangeTSensor[5], 30.0);
        QCOMPARE(c.forceControl.mounting.flangeTTool[2], 160.0);
        QCOMPARE(c.forceControl.mounting.flangeTTool[4], 50.0);
        QCOMPARE(c.forceControl.params.cutoffHz, 8.0);
        QCOMPARE(c.forceControl.params.deadzoneForceN, 2.0);
        QCOMPARE(c.forceControl.params.deadzoneTorqueNm, 0.5);
        QCOMPARE(c.forceControl.params.gainForce, 0.1);
        QCOMPARE(c.forceControl.params.gainTorque, 1.0);
        QCOMPARE(c.forceControl.params.vmaxPosMmS, 10.0);
        QCOMPARE(c.forceControl.params.vmaxRotDegS, 2.0);
        QCOMPARE(c.forceControl.axes.enX, true);
        QCOMPARE(c.forceControl.axes.enZ, false);
        QCOMPARE(c.forceControl.axes.enC, true);
    }

    void load_forceControlWrongTypesKeepDefaults()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({
          "force_control": {
            "sensor": { "host": 12345, "port": "6000",
                        "channel_signs": [1, "x", 1, 1, 1, 1],
                        "torque_scale": "1.5",
                        "force_capacity_n": [1, 2, "x"] },
            "deadzone": { "force_n": "3.0" },
            "axes": { "en_z": 1, "en_x": "true" }
          }
        })");
        f.flush();
        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err), qPrintable(err));
        // 类型不符的字段必须保留默认值
        QCOMPARE(c.forceControl.sensor.host, QString("192.168.0.108"));
        QCOMPARE(c.forceControl.sensor.port, quint16(4008));
        QCOMPARE(c.forceControl.sensor.channelSigns[1], 1);
        QCOMPARE(c.forceControl.sensor.torqueScale, 1.0);
        QCOMPARE(c.forceControl.sensor.forceCapacityN[2], 18000.0);
        QCOMPARE(c.forceControl.params.deadzoneForceN, 5.0);
        QCOMPARE(c.forceControl.axes.enZ, true);
        QCOMPARE(c.forceControl.axes.enX, false);
    }

    void load_forceControlOutOfRangePortKeepsDefault()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({ "force_control": { "sensor": { "port": 70000 } } })");
        f.flush();
        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err), qPrintable(err));
        QCOMPARE(c.forceControl.sensor.port, quint16(4008)); // 不得截断成 4464
    }
};

QTEST_MAIN(TestAppConfig)
#include "test_app_config.moc"
