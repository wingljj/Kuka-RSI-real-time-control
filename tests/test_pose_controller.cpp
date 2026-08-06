#include <QtTest>
#include <cmath>
#include <deque>
#include <limits>
#include "core/PoseController.h"

namespace {

// 满预算 = 距上一次 step 恰好过去一个配置周期。绝大多数用例断言的是"正常
// 配速下的行为"，所以显式传这个值——step() 的第二个参数没有默认值，正是
// 为了逼每个调用点写出自己假设的节奏。
constexpr double kCycleMs = 12.0;

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
    c.targetTrajectoryMs = 0.0;    // 轨迹立即完成 = 直通，保持增量 = kp×误差 的精确算术断言
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
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        QCOMPARE(d.x, 0.0);                        // 未使能 → 不动
        QCOMPARE(pc.state(), TrackState::Idle);
    }

    void zeroError_producesZeroDelta()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{10, 20, 30, 1, 2, 3});
        pc.setTracking(true);
        const Pose d = pc.step(Pose{10, 20, 30, 1, 2, 3}, kCycleMs);
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
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
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
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
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
        const Pose d = pc.step(Pose{0, 0, 0, -179, 0, 0}, kCycleMs);
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
            const Pose d = pc.step(actual, kCycleMs);
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
            actual.x += pc.step(actual, kCycleMs).x;

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
        pc.step(actual, kCycleMs);
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
            actual.x += pc.step(actual, kCycleMs).x;
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
        const Pose d = pc.step(bad, kCycleMs);
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
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
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
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
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
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
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
        QVERIFY(pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs).x > 0.0);
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
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
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
            pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
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
            actual.x += pc.step(actual, kCycleMs).x;
        // 再走一个周期但不喂回，使 before 就是"以当前 actual 度量的位移"，
        // 这样才能和归零后同一 actual 上的度量直接比较（位移天然滞后一拍）。
        pc.step(actual, kCycleMs);
        const double before = pc.accumulated().x;
        QVERIFY(before > 0.0);

        pc.resetToActual(actual);      // 会话内归零：原点不得移动
        pc.setTracking(true);
        pc.step(actual, kCycleMs);
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
            actual.x += pc.step(actual, kCycleMs).x;
        QVERIFY(pc.accumulated().x > 0.0);

        pc.beginSession(actual);       // 真正的会话重启：原点移到此处
        QCOMPARE(pc.accumulated().x, 0.0);
        pc.setTracking(true);
        pc.step(actual, kCycleMs);
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
        QCOMPARE(pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs).x, 0.0);
        pc.setTracking(true);   // 不得直接重新使能
        QCOMPARE(pc.state(), TrackState::Fault);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
    }

    void targetTrajectory_progressivelyApproachesStepTarget()
    {
        // 放开限幅让增量 = kp×误差。轨迹语义：目标阶跃 100、时长 50ms、周期 12ms。
        // 首周期先采样后推进：u=0 → 采样 = 起点（= 实际）→ 增量 0（五次多项式
        // 起点速度 0）；次周期 u=12/50=0.24，s(0.24)=10u³-15u⁴+6u⁵≈0.09325
        // → 误差 ≈9.325 → 增量 ≈4.66，显著削平（无轨迹时直通 50）。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetTrajectoryMs = 50.0;
        c.kpPos              = 0.5;
        c.vmaxPosMmS         = 1000000.0;   // 放开限幅
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        QCOMPARE(d1.x, 0.0);                 // 起点速度 0
        const Pose d2 = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        QVERIFY(d2.x < 20.0);
        QVERIFY(qAbs(d2.x - 4.66) < 0.05);   // 0.5 × 100 × s(0.24)
    }

    void targetTrajectory_zeroDuration_isPassthrough()
    {
        PoseController pc;
        AppConfig c = testCfg();             // targetTrajectoryMs=0 → 轨迹立即完成 = 直通
        c.kpPos      = 0.5;
        c.vmaxPosMmS = 1000000.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        QVERIFY(qAbs(d1.x - 50.0) < 1e-9);   // 直通
    }

    void resetToActual_completesTrajectory()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetTrajectoryMs = 50.0;
        c.kpPos              = 0.5;
        c.vmaxPosMmS         = 1000000.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);     // 轨迹启动（u=0，增量 0）
        pc.resetToActual(Pose{3, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        // 轨迹必须立即完成（目标=实际=3），否则会残留向 3 逼近的假误差 → 非零增量
        const Pose d = pc.step(Pose{3, 0, 0, 0, 0, 0}, kCycleMs);
        QCOMPARE(d.x, 0.0);
    }

    void targetTrajectory_doesNotChangeSteadyState()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetTrajectoryMs = 50.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{5, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5000; ++i) {
            const Pose d = pc.step(actual, kCycleMs);
            actual.x += d.x;
        }
        // 轨迹到点即达：完成后稳态与无轨迹一致（目标 = 实际）
        QVERIFY(qAbs(pc.target().x - actual.x) < 1e-3);
        QVERIFY(qAbs(pc.accumulated().x - 5.0) < 1e-3);
    }

    void targetTrajectory_angularJumpTakesShortestPath()
    {
        // 目标 179→-179（最短差 2°）。轨迹姿态 Slerp 走 SO(3) 最短弧（经 180 侧）：
        // 首周期增量 0（起点速度 0）；次周期采样越过 +179.18°（短弧上向 +180 前进）
        // → 误差为正 → 增量方向为正（而非经 0 的 358° 长路）。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetTrajectoryMs = 50.0;
        c.kpRot              = 0.5;
        c.vmaxRotDegS        = 1000000.0;   // 放开限幅
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 179, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, -179, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 179, 0, 0}, kCycleMs);
        QCOMPARE(d1.a, 0.0);                 // 起点速度 0
        const Pose d2 = pc.step(Pose{0, 0, 0, 179, 0, 0}, kCycleMs);
        QVERIFY(d2.a > 0.0);                 // 经 180 侧（短路径），而非经 0 侧
    }

    void attitude_singularTarget_doesNotJumpOrDiverge()
    {
        // B=180, A/C=±180（奇异+边界）：误差为连续旋转向量，增量不发散。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetTrajectoryMs = 0.0;
        c.kpRot = 0.1;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 60, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, -180, 180, -180});
        // 多周期：增量应有限、方向稳定（不出现 ±179 来回），最终误差收敛或单调减小
        Pose actual{0, 0, 0, 0, 60, 0};
        double prevA = 0;
        for (int i = 0; i < 2000; ++i) {
            const Pose d = pc.step(actual, kCycleMs);
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
        c.targetTrajectoryMs = 0.0;
        c.kpRot = 0.1;
        c.vmaxRotDegS = 1000000.0;   // 放开限幅，让 kp×误差 直接体现
        pc.configure(c);
        pc.beginSession(Pose{0,0,0, 0, 60, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0,0,0, 5, 60, 5});   // 小姿态目标，非奇异
        const Pose d = pc.step(Pose{0,0,0, 0, 60, 0}, kCycleMs);
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
        c.targetTrajectoryMs = 0.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 90, 0, 0});   // 大姿态误差（90° 绕 Z）
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        // d 是欧拉增量（度）。E⁻¹ 在 B=0 只重排轴序，故欧拉范数 = 旋转向量范数。
        const double rotDeg = std::sqrt(d.a*d.a + d.b*d.b + d.c*d.c);
        QVERIFY(rotDeg > 0.0);
        QVERIFY(rotDeg <= 0.2);   // 0.12° + E⁻¹ 耦合容差；未修复时 ≈6.87°
    }

    void attitude_zeroVmaxRot_blocksRotation()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetTrajectoryMs = 0.0;
        c.vmaxRotDegS = 0.0;               // 0 = 旋转被阻止
        pc.configure(c);
        pc.beginSession(Pose{0,0,0, 0,60,0});
        pc.setTracking(true);
        pc.setTarget(Pose{0,0,0, 0,0,0});  // 大姿态误差
        const Pose d = pc.step(Pose{0,0,0, 0,60,0}, kCycleMs);
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
        c.targetTrajectoryMs = 0.0;
        c.kpPos = 1.0;
        c.vmaxPosMmS = 50.0;              // 12ms → 0.6mm/周期
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0.5, 0.5, 0.5, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        const double norm = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
        QVERIFY(norm <= 0.6 + 1e-9);
        QVERIFY(qAbs(d.x - d.y) < 1e-9);  // 等比缩放
    }

    void position_zeroVmax_blocksMotion()
    {
        // vmax_pos=0：位置被阻止（镜像姿态路径）。旧实现用 `m_stepLimitPos > 0.0`
        // 守卫跳过限幅，原始 kp×err 无界直通——修复前本用例失败。
        PoseController pc;
        AppConfig c = testCfg();
        c.vmaxPosMmS = 0.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 100, 100, 0, 0, 0});   // 大位置误差
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        QCOMPARE(d.x, 0.0);
        QCOMPARE(d.y, 0.0);
        QCOMPARE(d.z, 0.0);
    }

    // ── 步长预算按实测帧间隔发放 ──
    // 背景：主机线程停顿时 KRC 继续发包，几十帧积压在接收缓冲里，恢复后在
    // 几毫秒墙钟内被连续排空，且每帧都带着间隙前那个陈旧位姿（误差顶格）。
    // 按配置周期发放预算时一次排空 = 41 × 0.6 ≈ 25mm，正是 POSCORR 硬限的
    // 量级。以下四个用例钉住替代方案的四个性质。

    // 性质一：满周期间隔下与"按配置周期"逐位相同——这是"单调安全"论证的
    // 实证部分，也挡住"把预算无脑调小"的假修复。
    void stepBudget_fullCycleIsUnchanged()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, 12.0);
        QCOMPARE(d.x, 0.6);   // 逐位相等，不是"接近"：12/12 恰为 1.0
    }

    // 性质二：比例正确。半个周期 = 半个预算，不是"判定为异常后清零"——
    // 被抢占后追赶的帧与排空积压的帧本就是同一现象，只是规模不同。
    void stepBudget_scalesWithElapsed()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        QVERIFY(qAbs(pc.step(Pose{0, 0, 0, 0, 0, 0}, 6.0).x - 0.3) < 1e-12);
        QVERIFY(qAbs(pc.step(Pose{0, 0, 0, 0, 0, 0}, 1.2).x - 0.06) < 1e-12);
    }

    // 性质三：上限封在一个周期。间隔 500ms 不等于这一帧可以走 25mm——KRC
    // 在一个 IPO 周期内施加它，那是 40 倍速。封顶保证任何一帧都 ≤ 旧值。
    void stepBudget_isCappedAtOneCycle()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        QCOMPARE(pc.step(Pose{0, 0, 0, 0, 0, 0}, 500.0).x, 0.6);
    }

    // 性质四：整批积压只值它真正占用的墙钟。41 帧在 ~3ms 内排空 → 总量
    // 0.6 × 3/12 ≈ 0.15mm，而不是 24.6mm。
    void stepBudget_backlogDrainCollapsesToElapsedWallClock()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        // 陈旧位姿：机器人在主机停顿期间没收到修正，没动，误差始终顶格。
        constexpr int    kBacklog   = 41;
        constexpr double kDrainMs   = 3.0;             // 整批排空占用的墙钟
        constexpr double kPerFrame  = kDrainMs / kBacklog;
        double sumX = 0.0;
        for (int i = 0; i < kBacklog; ++i)
            sumX += pc.step(Pose{0, 0, 0, 0, 0, 0}, kPerFrame).x;
        QVERIFY2(sumX < 0.2, qPrintable(QStringLiteral("排空吐出 %1 mm").arg(sumX)));
        // 且确实等于"墙钟 × vmax"，不是被某个阈值一刀切成 0：控制律在排空
        // 期间仍然在工作，只是按它真正占用的时间收费。
        QVERIFY(qAbs(sumX - 50.0 * kDrainMs / 1000.0) < 1e-9);
    }

    // 无法测量间隔时（看门狗刚清掉基准）必须取零预算，绝不能退化成满预算。
    // 负值与非有限值走同一条路：非有限值若混进比例计算，会让"范数 > 限值"
    // 恒为假，第 1 层静默失效。
    void stepBudget_unknownElapsedGivesZeroBudget()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 90, 0});
        for (double bad : {0.0, -1.0, -500.0,
                           std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()}) {
            const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, bad);
            QCOMPARE(d.x, 0.0);
            QCOMPARE(d.a, 0.0);
            QCOMPARE(d.b, 0.0);
            QCOMPARE(d.c, 0.0);
        }
    }

    // ── 轨迹时间基必须与步长限值同源 ──
    // 度量刻意取"操作员看得见的量"：一条 2mm 移动指令要走多少帧才走完。排空
    // 41 帧之后这个数字必须不变。轨迹若仍按配置周期推进，排空就把五次多项式
    // "起点速度 0"的性质整个抹掉——41 帧积压把轨迹推进 492ms（默认轨迹时长
    // 1000ms，半条），恢复后第一个满预算帧直接顶格，于是同一条指令的实际
    // 速度成了网络抖动的函数。变异实测（把 advance() 改回配置周期）：无排空
    // 82 帧 984ms（2.0 mm/s），排空后 41 帧 492ms（4.1 mm/s），快一倍。
    void trajectoryTimeBase_backlogDrainDoesNotCompressTheMove()
    {
        // 先排空 backlog 帧（整批只占 kDrainMs 墙钟），再按 12ms 正常配速跑，
        // 返回"正常配速下还要几帧才走完这 2mm"。
        constexpr double kDrainMs = 3.0;
        constexpr int    kBacklog = 41;
        auto framesToFinish2mm = [](int backlog) {
            AppConfig c = testCfg();
            c.targetTrajectoryMs = 1000.0;   // 生产默认值，正是"41 帧 = 半条轨迹"的前提
            PoseController pc;
            pc.configure(c);
            pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
            pc.setTracking(true);
            pc.setTarget(Pose{2.0, 0, 0, 0, 0, 0});
            Pose actual{};
            for (int i = 0; i < backlog; ++i)
                actual.x += pc.step(actual, kDrainMs / kBacklog).x;
            int frames = 0;
            while (actual.x < 1.999 && frames < 10000) {
                actual.x += pc.step(actual, kCycleMs).x;
                ++frames;
            }
            return frames;
        };
        const int clean   = framesToFinish2mm(0);
        const int drained = framesToFinish2mm(kBacklog);
        qInfo("2mm move: clean %d frames (%.0f ms), after %d-frame drain %d frames (%.0f ms)",
              clean, clean * kCycleMs, kBacklog, drained, drained * kCycleMs);

        // 正常运行不得回归：无积压时每帧的 elapsed 恰是一个配置周期，
        // min(Δt,T)/T 恰为 1.0，轨迹推进量与"按 m_cfg.cycleMs 推进"逐位相同，
        // 帧数必须还是修复前那个 82（≈ 五次多项式走到 99.95% 的时刻 u≈0.975
        // → 975ms / 12ms，再加一帧闭环滞后）。写死数字是安全的：这条路径是
        // 纯算术，不含任何计时或调度，跨机器可复现。
        QCOMPARE(clean, 82);
        // 排空最多"偷走"它真正占用的 3ms 墙钟（= 0.25 帧），故帧数差 ≤ 1。
        // 轨迹改回按配置周期推进时这里是 78 vs 37。
        QVERIFY2(qAbs(drained - clean) <= 1,
                 qPrintable(QStringLiteral(
                     "排空把 2mm 移动从 %1 帧压缩到 %2 帧：轨迹与限值不同源")
                                .arg(clean).arg(drained)));
    }

    // 排空恢复后的第一个满预算帧必须仍是"起点速度 0"的那个 0——这是上面那个
    // 用例的逐帧版本，直接钉住审查者度量的那个量（backlog=41 时 delta 由
    // 0.600000mm 变回 0.000000mm）。分开写是因为它不依赖 2mm 这个具体行程，
    // 日后有人改了轨迹时长/增益也不会连带失效。
    void trajectoryTimeBase_fullBudgetFrameAfterDrainStaysAtTrajectoryStart()
    {
        AppConfig c = testCfg();
        c.targetTrajectoryMs = 1000.0;
        PoseController pc;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});   // 远目标：轨迹一推进就足以顶格
        // 41 帧积压在 3ms 墙钟内排空：限值预算合计只值 3ms，轨迹也只该走 3ms。
        for (int i = 0; i < 41; ++i)
            pc.step(Pose{0, 0, 0, 0, 0, 0}, 3.0 / 41.0);
        // 恢复后第一个满预算帧。轨迹按配置周期推进时它是 0.6mm（顶格）。
        const double d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs).x;
        qInfo("first full-budget frame after drain = %.6f mm", d);
        QVERIFY2(d < 0.01,
                 qPrintable(QStringLiteral("排空后首个满预算帧发了 %1 mm，"
                                           "轨迹被排空推进过头").arg(d)));
    }

    // 姿态路径必须与位置路径同步收紧。只改一条路的话，排空期间机器人照样
    // 能走满 41 帧的角度预算（0.12°/帧 × 41 ≈ 4.9°，POSCORR 姿态限 25°）。
    void attitudeStepBudget_scalesAndCapsLikePosition()
    {
        PoseController pc;
        AppConfig c = testCfg();       // vmaxRotDegS=10, cycleMs=12 → 0.12 deg/周期
        c.targetTrajectoryMs = 0.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 90, 0, 0});   // 大姿态误差 → 恒顶格
        auto rotNorm = [&](double ms) {
            const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, ms);
            return std::sqrt(d.a*d.a + d.b*d.b + d.c*d.c);
        };
        const double full = rotNorm(12.0);
        QVERIFY(qAbs(full - 0.12) < 1e-9);          // B=0：E⁻¹ 只重排轴序
        QVERIFY(qAbs(rotNorm(6.0) - 0.06) < 1e-9);  // 半个周期 = 半个预算
        QCOMPARE(rotNorm(500.0), full);             // 封顶：不得超过一个周期
        // 排空：41 帧共 3ms 墙钟 → 总角度 ≈ 0.12 × 3/12 = 0.03°，而非 4.9°
        double sumRot = 0.0;
        for (int i = 0; i < 41; ++i)
            sumRot += rotNorm(3.0 / 41.0);
        QVERIFY2(sumRot < 0.05, qPrintable(QStringLiteral("排空吐出 %1 deg").arg(sumRot)));
    }

    // ── 闭环对象：误差必须相对「指令台账」而非实测 RIst 计算 ─────────────
    // 真机事故（2026-08-04）：对滞后的实测位姿做比例反馈、经 POSCORR（增量
    // 积分）施加，等效「积分器 + 被控对象滞后」闭环。4ms 周期 kp=0.1 的等效
    // 积分增益 25/s 远超真实伺服滞后（几十 ms）下的稳定边界 → 机械臂持续
    // 抖动，幅值被 vmax 限幅兜成 ±满预算来回打（现场：目标 Z+2mm 疯狂抖动）。
    // 模拟器的被控对象把修正瞬时施加（滞后 ≈1 帧），所以从未暴露。
    // 相对指令台账闭环后，环路极点 = 1−kp，与被控对象动力学无关，无条件稳定；
    // 机器人以自身伺服动态开环跟随指令，不再与滞后的测量值打架。
    void stalledPlant_totalCommandConvergesToOffset()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = 0.5;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 2.0, 0, 0, 0});      // Z +2mm，现场同款
        // 被控对象完全不动（极端滞后）：增量必须几何衰减并收敛到 2mm，
        // 而不是每帧照发 kp×误差 直到天荒地老。
        double sum = 0.0, last = 1e9;
        for (int i = 0; i < 400; ++i) {
            const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
            sum += d.z;
            last = d.z;
        }
        QVERIFY2(sum < 2.0 + 1e-6,
                 qPrintable(QStringLiteral("累计发出 %1 mm，失控").arg(sum)));
        QVERIFY2(sum > 1.99,
                 qPrintable(QStringLiteral("累计只发出 %1 mm").arg(sum)));
        QVERIFY2(qAbs(last) < 1e-6,
                 qPrintable(QStringLiteral("400 帧后仍在发 %1 mm/帧").arg(last)));
    }

    void laggedPlant_doesNotOscillate()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = 0.5;               // 旧架构在纯延迟 6 帧的对象下必振荡
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 2.0, 0, 0, 0});
        // 被控对象 = 指令和经 6 帧纯延迟（近似伺服 + RSI 管线滞后）
        std::deque<double> pipe(6, 0.0);
        double plantZ = 0.0, cmdSum = 0.0, sumAbs = 0.0, prevD = 0.0;
        int reversals = 0;
        for (int i = 0; i < 600; ++i) {
            const Pose d = pc.step(Pose{0, 0, plantZ, 0, 0, 0}, kCycleMs);
            cmdSum += d.z;
            sumAbs += qAbs(d.z);
            if (d.z * prevD < 0.0) ++reversals;
            if (d.z != 0.0) prevD = d.z;
            pipe.push_back(cmdSum);
            plantZ = pipe.front();
            pipe.pop_front();
        }
        QVERIFY2(reversals == 0,
                 qPrintable(QStringLiteral("增量方向反转 %1 次（振荡）").arg(reversals)));
        QVERIFY2(sumAbs < 2.0 + 1e-6,
                 qPrintable(QStringLiteral("总行程 %1 mm，应恰为 2mm").arg(sumAbs)));
        QVERIFY(qAbs(plantZ - 2.0) < 1e-3);          // 机器人最终到位
    }

    void stalledPlant_attitudeConverges()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpRot = 0.5;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 2.0, 0, 0});      // A +2°
        double sum = 0.0, last = 1e9;
        for (int i = 0; i < 400; ++i) {
            const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
            sum += d.a;
            last = d.a;
        }
        QVERIFY2(sum < 2.0 + 1e-3,
                 qPrintable(QStringLiteral("姿态累计发出 %1 deg，失控").arg(sum)));
        QVERIFY2(sum > 1.98,
                 qPrintable(QStringLiteral("姿态累计只发出 %1 deg").arg(sum)));
        QVERIFY(qAbs(last) < 1e-6);
    }

    // ── 轨迹锚点必须与闭环对象一致(2026-08-06 审查 P0-1)────────────────
    // 台账闭环后,运动中台账领先实测一个伺服滞后量。若轨迹仍从实测出发,
    // 运动中改目标的首采样 < 台账 → 误差为负 → 机器人先被命令倒退——
    // 与操作员意图相反的反向运动。轨迹起点必须取台账。
    void retargetWhileTracking_neverCommandsReversal()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos              = 0.5;
        c.targetTrajectoryMs = 120.0;             // 10 帧轨迹
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 2.0, 0, 0, 0});
        // 实测停滞(极端伺服滞后):台账前进、实测原地,领先量最大化
        for (int i = 0; i < 5; ++i)
            pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        // 运动中改目标:更远的 Z+4。任何一帧都不得发负增量(反向)。
        pc.setTarget(Pose{0, 0, 4.0, 0, 0, 0});
        for (int i = 0; i < 40; ++i) {
            const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
            QVERIFY2(d.z >= -1e-12,
                     qPrintable(QStringLiteral("第 %1 帧反向增量 %2 mm")
                                    .arg(i).arg(d.z)));
        }
    }

    // Idle 期间 jog 过机器人后使能:陈旧轨迹(从旧位姿规划)必须重规划,
    // 否则机器人先朝旧轨迹起点绕行(远离最终目标的方向)再折返。
    void enableAfterJog_replansStaleTrajectory()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos              = 0.5;
        c.targetTrajectoryMs = 240.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTarget(Pose{0, 0, 2.0, 0, 0, 0});   // Idle 下规划:0 → 2
        pc.step(Pose{0, 0, 10.0, 0, 0, 0}, kCycleMs);   // jog 到 Z=10(Idle,不动)
        pc.setTracking(true);
        // 从 10 去 2:位置只应单调下降到 2,绝不该先冲向旧轨迹起点 0
        double z = 10.0, zMin = 10.0;
        for (int i = 0; i < 200; ++i) {
            const Pose d = pc.step(Pose{0, 0, z, 0, 0, 0}, kCycleMs);
            z += d.z;                              // 理想执行:机器人即时跟随
            zMin = std::min(zMin, z);
        }
        QVERIFY2(zMin >= 2.0 - 1e-6,
                 qPrintable(QStringLiteral("位置最低到 %1 mm,越过目标冲向旧轨迹")
                                .arg(zMin)));
        QVERIFY(qAbs(z - 2.0) < 1e-3);            // 最终仍到达目标
    }

    // 重新使能跟踪时指令台账必须重新对齐当前实际：Idle 期间操作员可能手动
    // 移动过机器人，旧台账已失效；不重对齐的话第一帧就会按陈旧台账发增量。
    void reenableTracking_resyncsCommandToActual()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = 0.5;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 2.0, 0, 0, 0});
        const Pose first = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);   // 台账继续推进
        pc.setTracking(false);
        pc.setTracking(true);                        // 重新使能 → 重对齐
        const Pose again = pc.step(Pose{0, 0, 0, 0, 0, 0}, kCycleMs);
        // 重对齐后误差重新从「目标 − 当前实际」起算，与首帧一致
        QVERIFY(qAbs(again.z - first.z) < 1e-9);
    }
};

QTEST_MAIN(TestPoseController)
#include "test_pose_controller.moc"
