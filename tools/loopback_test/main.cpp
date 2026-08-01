// 把 RsiWorker 跑在真实通信线程上，监听 127.0.0.1，
// 供 krc_simulator 打靶，用于端到端实时性验证。
#include <QCoreApplication>
#include <QCommandLineParser>
#include <QThread>
#include <QTimer>
#include <cstdio>
#include "core/AppConfig.h"
#include "net/RsiWorker.h"
#include "net/SharedState.h"

namespace {

// 注意：verify 脚本 grep 的 state=Fault / state=Tracking 依赖这里的字符串，
// "Fault" 与 "Tracking" 不得改名。
const char *stateName(ControlState s)
{
    switch (s) {
    case ControlState::Disconnected:     return "Disconnected";
    case ControlState::WaitingFirstFrame: return "WaitingFirstFrame";
    case ControlState::Syncing:          return "Syncing";
    case ControlState::Ready:            return "Ready";
    case ControlState::Tracking:         return "Tracking";
    case ControlState::StaleFrame:       return "StaleFrame";
    case ControlState::Fault:            return "Fault";
    }
    return "?";
}

void dumpSnapshot(const char *tag, const StatusSnapshot &s)
{
    std::printf("%s frames=%llu missed=%d cycle_ms=%.2f max_reply_us=%.1f\n",
                tag,
                static_cast<unsigned long long>(s.frameCount),
                s.missedCount, s.measuredCycleMs, s.maxReplyUs);
    std::printf("%s accum X=%.3f  err X=%.3f  actual X=%.3f  target X=%.3f "
                " state=%s%s%s\n",
                tag, s.accum.x, s.error.x, s.actual.x, s.target.x,
                stateName(s.state),
                s.faultReason.isEmpty() ? "" : "  fault=",
                s.faultReason.isEmpty() ? "" : qPrintable(s.faultReason));
    // 姿态误差 = SO(3) 旋转向量范数（度）；姿态本身（欧拉，度）
    const double rotN = std::sqrt(s.error.a * s.error.a
                                  + s.error.b * s.error.b
                                  + s.error.c * s.error.c);
    std::printf("%s rotErr=%.3f deg (A=%.1f B=%.1f C=%.1f)"
                "  actual(A=%.1f B=%.1f C=%.1f)  target(A=%.1f B=%.1f C=%.1f)\n",
                tag, rotN, s.error.a, s.error.b, s.error.c,
                s.actual.a, s.actual.b, s.actual.c,
                s.target.a, s.target.b, s.target.c);
    std::fflush(stdout);
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);

    // Q_ARG 跨线程排队传递 Pose/AppConfig：显式注册元类型，避免队列连接
    // 在运行时静默失败。
    qRegisterMetaType<Pose>();
    qRegisterMetaType<AppConfig>();

    QCommandLineParser p;
    p.addHelpOption();
    QCommandLineOption oPort("port", "listen port", "n", "59152");
    QCommandLineOption oSecs("seconds", "run duration", "s", "10");
    QCommandLineOption oTrack("track", "enable tracking with X offset",
                              "mm", "0");
    QCommandLineOption oTrackAbc("track-abc", "attitude target \"A B C\" (deg), enables tracking",
                                 "a b c", "");
    QCommandLineOption oRestart("restart-at-ms",
                                "stop()+start() N ms after launch (0=off)",
                                "ms", "0");
    // 用来构造一段真实的通信间隙。默认 0 = 立刻 start()，即原来的
    // "快速 stop()→start()"（间隙 < 1ms）。给一个正值就能落进
    // "看门狗间隔 < 间隙 < 会话间隔" 这个区间，正是本次要验证的窗口。
    QCommandLineOption oStopFor("stop-for-ms",
                                "with --restart-at-ms: hold the socket closed "
                                "N ms before start() (0=immediate)",
                                "ms", "0");
    // 会话判定阈值可覆盖，用于对照旧行为（旧代码用的是看门狗间隔 240ms）。
    QCommandLineOption oGap("session-gap-ms",
                            "override AppConfig::sessionGapMs", "ms", "");
    p.addOptions({oPort, oSecs, oTrack, oTrackAbc, oRestart, oStopFor, oGap});
    p.process(app);

    AppConfig cfg = AppConfig::defaults();
    cfg.listenIp   = "127.0.0.1";
    cfg.listenPort = quint16(p.value(oPort).toUShort());
    if (!p.value(oGap).isEmpty())
        cfg.sessionGapMs = p.value(oGap).toDouble();
    std::printf("session_gap_ms=%.1f\n", cfg.sessionGapMs);
    std::fflush(stdout);

    // 刻意用 static：SampleRing 约 96KB，放在 main() 的栈上虽仍在 Windows
    // 默认 1MB 栈内，但没有必要占用那份余量。
    static SharedState state;
    static SampleRing  ring;

    QThread commThread;
    auto *worker = new RsiWorker(cfg, &state, &ring);
    worker->moveToThread(&commThread);

    // started 在通信线程内发射，worker 亦属通信线程 → 直连，
    // 于是 socket 与 watchdog 都获得通信线程的亲和性。
    QObject::connect(&commThread, &QThread::started,
                     worker, &RsiWorker::start);

    // 以下三个信号都以 &app（主线程）为 context 对象：跨线程 AutoConnection
    // 即 QueuedConnection，槽体在主线程执行，绝不在 onDatagram() 循环体中途
    // 回调进 worker。
    QObject::connect(worker, &RsiWorker::bindFailed, &app,
                     [](const QString &why) {
                         std::fprintf(stderr, "bind failed: %s\n",
                                      qPrintable(why));
                         QCoreApplication::exit(2);
                     });
    QObject::connect(worker, &RsiWorker::listening, &app, [] {
        std::printf("listening\n");
        std::fflush(stdout);
    });

    const double offset = p.value(oTrack).toDouble();
    // --track-abc "A B C"：姿态目标（度）+ 使能跟踪，用于验证奇异/大姿态收敛。
    const QString abcStr = p.value(oTrackAbc);
    const bool useAbc = !abcStr.isEmpty();
    std::array<double, 3> abc{0, 0, 0};
    if (useAbc) {
        const QStringList parts = abcStr.split(QLatin1Char(' '), Qt::SkipEmptyParts);
        bool allOk = parts.size() == 3;
        for (int i = 0; allOk && i < 3; ++i) {
            bool ok = false;
            const double v = parts[i].toDouble(&ok);
            if (!ok) allOk = false; else abc[i] = v;
        }
        if (!allOk) {
            std::fprintf(stderr, "--track-abc needs \"A B C\" (degrees)\n");
            return 2;
        }
    }
    // 重启后 RsiWorker 会再次发出 firstFrameReceived。若那时重新下发目标并
    // 重新使能跟踪，累积量会再涨一轮，就分辨不出"会话检测是否清零了累积量"。
    // 因此只在第一次首帧下发一次。
    bool appliedOnce = false;
    QObject::connect(worker, &RsiWorker::firstFrameReceived, &app,
                     [worker, offset, useAbc, abc, &appliedOnce] {
        const bool first = !appliedOnce;
        appliedOnce = true;
        std::printf("first frame received%s\n",
                    first ? "" : " (after restart; target not re-applied)");
        std::fflush(stdout);
        if (first && (offset != 0.0 || useAbc)) {
            Pose t = state.snapshot().actual;
            if (useAbc) {
                t.a = abc[0]; t.b = abc[1]; t.c = abc[2];
                std::printf("applying target A=%.1f B=%.1f C=%.1f, tracking on\n",
                            abc[0], abc[1], abc[2]);
            } else {
                t.x += offset;
                std::printf("applying target X=%.3f, tracking on\n", t.x);
            }
            std::fflush(stdout);
            QMetaObject::invokeMethod(worker, "applyTarget",
                                      Qt::QueuedConnection,
                                      Q_ARG(Pose, t));
            QMetaObject::invokeMethod(worker, "setTracking",
                                      Qt::QueuedConnection,
                                      Q_ARG(bool, true));
        }
    });

    commThread.start();

    const int restartAtMs = p.value(oRestart).toInt();
    const int stopForMs   = p.value(oStopFor).toInt();
    if (restartAtMs > 0) {
        QTimer::singleShot(restartAtMs, &app, [worker, stopForMs] {
            // 快照必须早于 stop()：stop() 会发布一份空的 StatusSnapshot，
            // 那会抹掉 accum 的可见值（控制器内部的 m_accum 不受影响）。
            dumpSnapshot("[before-restart]", state.snapshot());
            std::printf("restart: invoking stop(), then start() after %d ms\n",
                        stopForMs);
            std::fflush(stdout);
            QMetaObject::invokeMethod(worker, "stop", Qt::QueuedConnection);
            if (stopForMs > 0) {
                // singleShot 的 context 是 worker → 定时器在通信线程里跑，
                // start() 直接在那条线程上执行，socket 亲和性与首个 start()
                // 一致。
                QTimer::singleShot(stopForMs, worker, [worker] {
                    std::printf("gap over: invoking start()\n");
                    std::fflush(stdout);
                    worker->start();
                });
            } else {
                QMetaObject::invokeMethod(worker, "start",
                                          Qt::QueuedConnection);
            }
        });
        // 重启后 stop() 发布的空快照要等下一帧才被真实数据覆盖，
        // 因此隔 1s 再采一次。
        QTimer::singleShot(restartAtMs + stopForMs + 1000, &app, [] {
            dumpSnapshot("[after-restart]", state.snapshot());
        });
    }

    QTimer::singleShot(p.value(oSecs).toInt() * 1000, &app,
                       [&commThread, worker] {
        const StatusSnapshot s = state.snapshot();
        std::printf("frames=%llu missed=%d cycle_ms=%.2f "
                    "max_reply_us=%.1f\n",
                    static_cast<unsigned long long>(s.frameCount),
                    s.missedCount, s.measuredCycleMs, s.maxReplyUs);
        std::printf("accum X=%.3f  err X=%.3f\n", s.accum.x, s.error.x);
        dumpSnapshot("[final]", s);
        QMetaObject::invokeMethod(worker, "stop",
                                  Qt::BlockingQueuedConnection);
        commThread.quit();
        commThread.wait(2000);
        QCoreApplication::quit();
    });

    const int rc = app.exec();
    if (commThread.isRunning()) {
        commThread.quit();
        commThread.wait(2000);
    }
    delete worker;
    return rc;
}
