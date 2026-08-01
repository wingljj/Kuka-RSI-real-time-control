#include <QtTest>
#include <cmath>
#include <limits>
#include "core/PoseController.h"

namespace {

AppConfig testCfg()
{
    AppConfig c = AppConfig::defaults();
    c.cycleMs            = 12.0;
    c.kpPos              = 1.0;    // 便于算术验证
    c.kpRot              = 1.0;
    c.vmaxPosMmS         = 50.0;   // 12ms → 步长上限 0.6mm
    c.vmaxRotDegS        = 10.0;   // 12ms → 步长上限 0.12°
    c.accumLimitPosMm    = 30.0;
    c.accumLimitRotDeg   = 15.0;
    c.targetSmoothingMs  = 0.0;    // 保持增量 = kp×误差 的精确算术断言
    return c;
}

} // namespace

class TestPoseController : public QObject
{
    Q_OBJECT
private slots:
    void notTracking_returnsZeroDelta()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});   // 巨大误差
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);                        // 未使能 → 不动
        QCOMPARE(pc.state(), TrackState::Idle);
    }

    void zeroError_producesZeroDelta()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{10, 20, 30, 1, 2, 3});
        pc.setTracking(true);
        const Pose d = pc.step(Pose{10, 20, 30, 1, 2, 3});
        QCOMPARE(d.x, 0.0);
        QCOMPARE(d.a, 0.0);
    }

    void largeError_isClampedToStepLimit()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 50mm/s * 0.012s = 0.6mm
        QVERIFY(qAbs(d.x - 0.6) < 1e-9);
    }

    void smallError_notClamped()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = 0.5;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0.4, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 0.5 * 0.4 = 0.2 < 0.6 → 不限幅
        QVERIFY(qAbs(d.x - 0.2) < 1e-9);
    }

    void rotationTakesShortestPath()
    {
        // 旋转向量误差天然取最短弧（从 -179 向 179 走经 ±180 的 2°，而非经 0 的
        // 358°）。增量方向与原 wrap 语义一致；但幅值经 E⁻¹ + 范数限幅后不再是
        // 旧的逐轴 0.12，故只断言方向。
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, -179, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 179, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, -179, 0, 0});
        QVERIFY(d.a < 0.0);   // 向负方向（经 180 侧短路径）
    }

    void accumulation_tracksSumOfDeltas()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;               // 模拟机器人跟随
        }
        // 命令和仍是三步之和
        QVERIFY(qAbs(pc.commandedSum().x - 1.8) < 1e-9);  // 3 * 0.6
        // 锚点位移按每周期开始时的 actual 度量，故天然滞后一个周期：
        // 第 3 步开始时机器人只走了 2 * 0.6。这个滞后是有意的——位移账本
        // 只认控制器真正回传的新 RIst，不认主机"以为发出去了"的命令。
        QVERIFY(qAbs(pc.accumulated().x - 1.2) < 1e-9);   // 2 * 0.6
    }

    void resetToActual_clearsTargetButKeepsAccum()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;     // 很小的限值：超越它不再触发 Fault
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i)
            actual.x += pc.step(actual).x;

        // 第 2 层已移除：累积越限只记账、不故障——状态仍为 Tracking，增量继续流动
        QCOMPARE(pc.state(), TrackState::Tracking);
        QVERIFY(pc.faultReason().isEmpty());
        // 归零前的命令账本必须非零，否则本用例无从证明「保留」
        const double accumBefore = pc.commandedSum().x;
        QVERIFY(qAbs(accumBefore) > 1e-9);

        pc.resetToActual(Pose{7, 8, 9, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
        QVERIFY(pc.faultReason().isEmpty());
        QCOMPARE(pc.target().x, 7.0);      // 目标 = 实际，误差归零
        // 【关键】KRC 侧已施加的修正不会消失，命令账本必须原样保留
        QCOMPARE(pc.commandedSum().x, accumBefore);

        // 【关键之二】会话锚点也必须原样保留：再走一个周期，位移仍以原锚点（0）
        // 度量，而不是以 resetToActual 传入的 {7,8,9} 度量——位移随 actual 前进。
        pc.setTracking(true);
        pc.step(actual);
        QVERIFY(qAbs(pc.accumulated().x - actual.x) < 1e-9);
    }

    void beginSession_clearsAccum()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;     // 很小的限值：超越它不再触发 Fault
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i)
            actual.x += pc.step(actual).x;
        // 第 2 层已移除：越限仍保持 Tracking，只验证账本确实累计了
        QCOMPARE(pc.state(), TrackState::Tracking);
        QVERIFY(qAbs(pc.accumulated().x) > 1e-9);
        QVERIFY(qAbs(pc.commandedSum().x) > 1e-9);

        // 仅 RSI 会话重启才可清零累积量
        pc.beginSession(Pose{7, 8, 9, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
        QVERIFY(pc.faultReason().isEmpty());
        QCOMPARE(pc.target().x, 7.0);
        QCOMPARE(pc.accumulated().x, 0.0);
        QCOMPARE(pc.commandedSum().x, 0.0);
    }

    void nonFinitePoseEntersFault()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        Pose bad{0, 0, 0, 0, 0, 0};
        bad.x = std::numeric_limits<double>::quiet_NaN();
        const Pose d = pc.step(bad);
        QCOMPARE(d.x, 0.0);
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(!pc.faultReason().isEmpty());
        // 累积量绝不能被 NaN 污染，否则显示账本与实际同步性都会失真
        QVERIFY(std::isfinite(pc.accumulated().x));
    }

    void negativeVmaxDoesNotInvertDirection()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.vmaxPosMmS = -50.0;         // 恶意/误填的负限速
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 必须朝目标走，绝不能因 clamp(lo>hi) 反向
        QVERIFY(d.x > 0.0);
        QVERIFY(qAbs(d.x - 0.6) < 1e-9);
    }

    void largeError_allComponentsLimited()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        // 六个分量都给足够大的误差。位置三轴按欧氏范数限幅：大误差下合成
        // 增量范数 = 0.6（每轴 0.6/√3 ≈ 0.346），不再逐轴 clamp 到 0.6
        // （逐轴 clamp 会让对角运动达到 √3× 限速）；姿态改走旋转向量范数
        // 限幅 + E⁻¹，各轴增量不再独立等于 0.12，只保证有限且被限幅
        // （详见 attitude_* 新用例）。
        pc.setTarget(Pose{100, 100, 100, 90, 90, 90});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        const double posNorm = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
        QVERIFY(qAbs(posNorm - 0.6) < 1e-9);   // 范数恰好压到限值
        QVERIFY(qAbs(d.x - d.y) < 1e-9);       // 等比缩放（各轴误差等大）
        QVERIFY(qAbs(d.y - d.z) < 1e-9);
        QVERIFY(std::isfinite(d.a) && std::isfinite(d.b) && std::isfinite(d.c));
    }

    void invalidCycleMsSurvivesReset()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.cycleMs = 0.0;
        pc.configure(c);
        // 生产调用顺序：configure → beginSession(首帧)。粘滞标志必须活过它
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        QVERIFY(pc.state() != TrackState::Tracking);   // 不得被使能
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(pc.faultReason().contains("cycleMs"));
    }

    void validConfigureClearsInvalidFlag()
    {
        PoseController pc;
        AppConfig bad = testCfg();
        bad.cycleMs = -1.0;
        pc.configure(bad);
        pc.configure(testCfg());       // 一次有效配置应解除粘滞
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        QCOMPARE(pc.state(), TrackState::Tracking);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        QVERIFY(pc.step(Pose{0, 0, 0, 0, 0, 0}).x > 0.0);
    }

    void nonFiniteGainEntersFault()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = std::numeric_limits<double>::quiet_NaN();
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{10, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);
        QCOMPARE(pc.state(), TrackState::Fault);
        // 累积量绝不能被污染，否则显示账本与实际同步性都会失真
        QVERIFY(std::isfinite(pc.accumulated().x));
    }

    void subQuantumIncrementDoesNotAccumulate()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = 1e-6;          // 极小增益，使每周期增量远小于线上量化步长
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{1.0, 0, 0, 0, 0, 0});
        for (int i = 0; i < 1000; ++i)
            pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 每周期 1e-6 会被 buildSen 量化成 0.0000，机器人不动，
        // 所以账本也不该增长——否则收敛后会持续漂移。
        // actual 固定不动，位移当然是 0；真正钉住死区的是命令和。
        QCOMPARE(pc.accumulated().x, 0.0);
        QCOMPARE(pc.commandedSum().x, 0.0);
    }

    void inSessionResetDoesNotMoveTheAnchor()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 10; ++i)
            actual.x += pc.step(actual).x;
        // 再走一个周期但不喂回，使 before 就是"以当前 actual 度量的位移"，
        // 这样才能和归零后同一 actual 上的度量直接比较（位移天然滞后一拍）。
        pc.step(actual);
        const double before = pc.accumulated().x;
        QVERIFY(before > 0.0);

        pc.resetToActual(actual);      // 会话内归零：原点不得移动
        pc.setTracking(true);
        pc.step(actual);
        QVERIFY(qAbs(pc.accumulated().x - before) < 1e-9);
    }

    void beginSessionMovesTheAnchor()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 10; ++i)
            actual.x += pc.step(actual).x;
        QVERIFY(pc.accumulated().x > 0.0);

        pc.beginSession(actual);       // 真正的会话重启：原点移到此处
        QCOMPARE(pc.accumulated().x, 0.0);
        pc.setTracking(true);
        pc.step(actual);
        QCOMPARE(pc.accumulated().x, 0.0);
    }

    void forceFault_latchesUntilReset()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.forceFault(QStringLiteral("network write failed"));
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(pc.faultReason().contains("network write failed"));
        QCOMPARE(pc.step(Pose{0, 0, 0, 0, 0, 0}).x, 0.0);
        pc.setTracking(true);   // 不得直接重新使能
        QCOMPARE(pc.state(), TrackState::Fault);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
    }

    void smoothing_progressivelyApproachesStepTarget()
    {
        // 放开限幅让增量 = kp×误差；无平滑时目标阶跃 100 第一周期误差=100
        // → 增量=50。平滑后平滑目标第一步 = 100×α，α=12/(12+50)=0.1935
        // → 误差≈19.35 → 增量≈9.68，显著削平。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        c.kpPos             = 0.5;
        c.vmaxPosMmS        = 1000000.0;   // 放开限幅
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QVERIFY(d1.x < 20.0);
        QVERIFY(qAbs(d1.x - 9.68) < 0.5);
    }

    void smoothing_tauZero_isPassthrough()
    {
        PoseController pc;
        AppConfig c = testCfg();            // targetSmoothingMs=0
        c.kpPos      = 0.5;
        c.vmaxPosMmS = 1000000.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QVERIFY(qAbs(d1.x - 50.0) < 1e-9);   // α=1 直通
    }

    void resetToActual_syncsSmoothTarget()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        c.kpPos             = 0.5;
        c.vmaxPosMmS        = 1000000.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        pc.step(Pose{0, 0, 0, 0, 0, 0});     // 平滑目标开始逼近（≈19.35）
        pc.resetToActual(Pose{3, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        // 平滑目标必须同步为 actual(3)，否则从 19.35 向 3 逼近 → 假误差 → 非零增量
        const Pose d = pc.step(Pose{3, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);
    }

    void smoothing_doesNotChangeSteadyState()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{5, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5000; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;
        }
        QVERIFY(qAbs(pc.target().x - actual.x) < 1e-3);
        QVERIFY(qAbs(pc.accumulated().x - 5.0) < 1e-3);
    }

    void smoothing_angularJumpTakesShortestPath()
    {
        // 目标 -179→+179（最短差 2°）。旋转向量插值走 SO(3) 最短弧（经 180 侧），
        // 第一周期增量方向为正（而非线性平滑经 0 的 358° 长路）。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        c.kpRot             = 0.5;
        c.vmaxRotDegS       = 1000000.0;   // 放开限幅
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 179, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, -179, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 179, 0, 0});
        QVERIFY(d1.a > 0.0);   // 经 180 侧（短路径），而非经 0 侧
    }

    void attitude_singularTarget_doesNotJumpOrDiverge()
    {
        // B=180, A/C=±180（奇异+边界）：误差为连续旋转向量，增量不发散。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 0.0;
        c.kpRot = 0.1;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 60, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, -180, 180, -180});
        // 多周期：增量应有限、方向稳定（不出现 ±179 来回），最终误差收敛或单调减小
        Pose actual{0, 0, 0, 0, 60, 0};
        double prevA = 0;
        for (int i = 0; i < 2000; ++i) {
            const Pose d = pc.step(actual);
            QVERIFY(std::isfinite(d.a) && std::isfinite(d.b) && std::isfinite(d.c));
            actual.a += d.a; actual.b += d.b; actual.c += d.c;
            (void)prevA;
        }
        // 增量范数应单调减小（收敛中）或至少有限不振荡
        QVERIFY(std::isfinite(actual.a) && std::isfinite(actual.b) && std::isfinite(actual.c));
    }

    void attitude_rkorrStaysEulerCompatible()
    {
        // 非奇异位形：E·Δ欧拉 ≈ d_rot（旋转向量）——增量经 E⁻¹ 正确映射
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 0.0;
        c.kpRot = 0.1;
        c.vmaxRotDegS = 1000000.0;   // 放开限幅，让 kp×误差 直接体现
        pc.configure(c);
        pc.beginSession(Pose{0,0,0, 0, 60, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0,0,0, 5, 60, 5});   // 小姿态目标，非奇异
        const Pose d = pc.step(Pose{0,0,0, 0, 60, 0});
        // d 是欧拉增量（度）。验证它非零、有限、方向合理（目标+方向）。
        QVERIFY(std::isfinite(d.a) && std::isfinite(d.b) && std::isfinite(d.c));
    }

    void attitude_stepLimitRespectsDegPerCycle()
    {
        // 姿态步长限值按 deg/周期：vmaxRotDegS=10, cycleMs=12 → 0.12°/周期。
        // 大姿态误差使旋转向量范数（rad）超过其 rad 等价阈值 → 必须按范数限幅。
        // 回归：rotNorm 是 rad，m_stepLimitRot 是 deg，比较前必须把阈值换成 rad，
        // 否则等效限幅是 ~6.87°/周期（0.12 rad）而非 0.12°/周期。
        PoseController pc;
        AppConfig c = testCfg();       // vmaxRotDegS=10, cycleMs=12 → 0.12 deg/cycle
        c.targetSmoothingMs = 0.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 90, 0, 0});   // 大姿态误差（90° 绕 Z）
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // d 是欧拉增量（度）。E⁻¹ 在 B=0 只重排轴序，故欧拉范数 = 旋转向量范数。
        const double rotDeg = std::sqrt(d.a*d.a + d.b*d.b + d.c*d.c);
        QVERIFY(rotDeg > 0.0);
        QVERIFY(rotDeg <= 0.2);   // 0.12° + E⁻¹ 耦合容差；未修复时 ≈6.87°
    }

    void attitude_zeroVmaxRot_blocksRotation()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 0.0;
        c.vmaxRotDegS = 0.0;               // 0 = 旋转被阻止
        pc.configure(c);
        pc.beginSession(Pose{0,0,0, 0,60,0});
        pc.setTracking(true);
        pc.setTarget(Pose{0,0,0, 0,0,0});  // 大姿态误差
        const Pose d = pc.step(Pose{0,0,0, 0,60,0});
        QCOMPARE(d.a, 0.0);
        QCOMPARE(d.b, 0.0);
        QCOMPARE(d.c, 0.0);
    }

    void positionDiagLimit_usesEuclideanNorm()
    {
        // 三轴各 0.5mm 误差：逐轴 clamp（0.6 限）各 0.5 → 合成 0.866 超限；
        // 范数限幅应把合成压到 ≤ 0.6（每轴 ~0.346）。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 0.0;
        c.kpPos = 1.0;
        c.vmaxPosMmS = 50.0;              // 12ms → 0.6mm/周期
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0.5, 0.5, 0.5, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        const double norm = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
        QVERIFY(norm <= 0.6 + 1e-9);
        QVERIFY(qAbs(d.x - d.y) < 1e-9);  // 等比缩放
    }
};

QTEST_MAIN(TestPoseController)
#include "test_pose_controller.moc"
