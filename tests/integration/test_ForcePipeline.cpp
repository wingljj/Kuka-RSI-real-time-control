#include <QtTest>
#include <cmath>
#include <cstring>
#include "core/ForceController.h"
#include "core/Pose.h"
#include "core/SriProtocol.h"
#include "core/Wrench.h"

// 力控全管线端到端集成测试（无硬件）：
//   SRI 二进制帧 → SriFrameParser → WrenchFrame → ForceController::step → Pose delta
// 与生产数据路径（RsiWorker/SriDriver）一致；通道符号与窗口均值逻辑由
// test_sri_driver 覆盖，此处按 brief 直接映射，聚焦「帧 → 控制量」主链路。

class TestForcePipeline : public QObject
{
    Q_OBJECT

    // 解析一帧携带 vals 的 SRI 二进制帧，映射为 WrenchFrame（模拟
    // SriDriver 的通道转换）。QTest 宏失败时会提前 return，故用出参。
    void parseFrame(const float vals[6], WrenchFrame *out)
    {
        // 4 字节头 + 6×float32 载荷，共 31 字节
        SriFrameParser parser;
        uint8_t frame[31] = {};
        frame[0] = 0xAA; frame[1] = 0x55; frame[2] = 0x00; frame[3] = 0x1B;
        std::memcpy(frame + 6, vals, sizeof(float) * 6);
        const auto frames = parser.feed(frame, sizeof(frame));
        QCOMPARE(frames.size(), size_t(1));
        QVERIFY(parser.discardedCount() == 0);

        WrenchFrame w;
        w.fx = frames[0].values[0];
        w.fy = frames[0].values[1];
        w.fz = frames[0].values[2];
        w.mx = frames[0].values[3];
        w.my = frames[0].values[4];
        w.mz = frames[0].values[5];
        w.fresh = true;
        *out = w;
    }

    // 配置并 enable 一个 ForceController，以 dtS 秒步长跑 nSteps 步
    //（让 Butterworth 收敛），累积的 delta 输出经出参返回。
    void runController(const ForceControlConfig &cfg, const WrenchFrame &w,
                       int nSteps, double dtS, Pose *out)
    {
        ForceController fc;
        fc.configure(cfg);
        WrenchFrame bias;  // 零 tare
        fc.enable(Pose{}, bias);
        QVERIFY(fc.isActive());

        // 500 步 × 4ms = 2s ≫ 20Hz 滤波器的 settling 时间（≈50ms）
        Pose total;
        for (int i = 0; i < nSteps; ++i) {
            const Pose d = fc.step(w, Pose{}, dtS);
            total.x += d.x; total.y += d.y; total.z += d.z;
            total.a += d.a; total.b += d.b; total.c += d.c;
        }
        *out = total;
    }

    // 基线配置：只启用 Z 平移轴，5N 死区，20Hz 截止（比生产 10Hz 收敛更快）。
    // 传感器与工具坐标系重合（identity 旋转），力方向不做旋转变换。
    // cycleMs = 4ms：滤波器设计采样率 = 1000/4 = 250Hz，与本测试 4ms 步长
    // 模拟的输入速率一致（保持历史行为；生产默认 12ms → 83.3Hz）。
    static ForceControlConfig baseConfig()
    {
        ForceControlConfig cfg;
        cfg.params.cycleMs = 4.0;
        cfg.axes.enZ = true;
        cfg.params.deadzoneForceN = 5.0;
        cfg.params.gainForce = 0.05;
        cfg.params.vmaxPosMmS = 5.0;
        cfg.params.cutoffHz = 20.0;
        cfg.mounting.flangeTSensor[2] = 0.0;
        cfg.mounting.flangeTTool[2] = 0.0;
        return cfg;
    }

private slots:
    void completePipelineFromFrameToDelta()
    {
        // 50N 沿 +Z，其余通道为零
        const float vals[6] = {0.0f, 0.0f, 50.0f, 0.0f, 0.0f, 0.0f};
        WrenchFrame w;
        parseFrame(vals, &w);
        Pose total;
        runController(baseConfig(), w, 500, 0.004, &total);

        // 只有 Z 响应：X/Y 保持为零，无虚假旋转
        QVERIFY(std::abs(total.x) < 0.001);
        QVERIFY(std::abs(total.y) < 0.001);
        QVERIFY(std::abs(total.a) < 1e-3);
        QVERIFY(std::abs(total.b) < 1e-3);
        QVERIFY(std::abs(total.c) < 1e-3);
        // 稳态速度 v = vmax·tanh(gain·(50−5)) ≈ 4.89 mm/s，2s ≈ 9.8mm；
        // 2s 远超 20Hz 滤波器上升时间（≈50ms），斜坡损失可忽略。
        QVERIFY(total.z > 5.0);
    }

    void deadzoneBlocksSubThresholdForce()
    {
        // 2N 沿 +Z，低于 5N 向量死区 → 速度恒为 0，任何轴都不该移动
        const float vals[6] = {0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f};
        WrenchFrame w;
        parseFrame(vals, &w);
        Pose total;
        runController(baseConfig(), w, 500, 0.004, &total);

        QVERIFY(std::abs(total.x) < 1e-9);
        QVERIFY(std::abs(total.y) < 1e-9);
        QVERIFY(std::abs(total.z) < 1e-9);
        QVERIFY(std::abs(total.a) < 1e-9);
        QVERIFY(std::abs(total.b) < 1e-9);
        QVERIFY(std::abs(total.c) < 1e-9);
    }

    void axisMaskBlocksDisabledDirections()
    {
        ForceControlConfig cfg = baseConfig();
        cfg.axes.enX = true;
        cfg.axes.enY = false;
        cfg.axes.enZ = false;  // Z 被屏蔽

        // 50N 在 Z：enZ=false 必须完全挡住 Z 移动
        const float zForce[6] = {0.0f, 0.0f, 50.0f, 0.0f, 0.0f, 0.0f};
        WrenchFrame wz;
        parseFrame(zForce, &wz);
        Pose totalZ;
        runController(cfg, wz, 500, 0.004, &totalZ);
        QVERIFY(std::abs(totalZ.x) < 0.001);
        QVERIFY(std::abs(totalZ.y) < 0.001);
        QVERIFY(std::abs(totalZ.z) < 0.001);

        // 阳性对照：启用的 X 轴在 50N 沿 X 时必须照常响应，
        // 被屏蔽的 Y/Z 仍保持为零——证明掩码是「按轴挡」而非「全关」。
        const float xForce[6] = {50.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        WrenchFrame wx;
        parseFrame(xForce, &wx);
        Pose totalX;
        runController(cfg, wx, 500, 0.004, &totalX);
        QVERIFY(totalX.x > 5.0);
        QVERIFY(std::abs(totalX.y) < 0.001);
        QVERIFY(std::abs(totalX.z) < 0.001);
    }
};
QTEST_MAIN(TestForcePipeline)
#include "test_ForcePipeline.moc"
