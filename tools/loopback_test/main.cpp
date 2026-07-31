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

const char *stateName(TrackState s)
{
    switch (s) {
    case TrackState::Idle:     return "Idle";
    case TrackState::Tracking: return "Tracking";
    case TrackState::Fault:    return "Fault";
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
    QCommandLineOption oRestart("restart-at-ms",
                                "stop()+start() N ms after launch (0=off)",
                                "ms", "0");
    p.addOptions({oPort, oSecs, oTrack, oRestart});
    p.process(app);

    AppConfig cfg = AppConfig::defaults();
    cfg.listenIp   = "127.0.0.1";
    cfg.listenPort = quint16(p.value(oPort).toUShort());

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
    // 重启后 RsiWorker 会再次发出 firstFrameReceived。若那时重新下发目标并
    // 重新使能跟踪，累积量会再涨一轮，就分辨不出"会话检测是否清零了累积量"。
    // 因此只在第一次首帧下发一次。
    bool appliedOnce = false;
    QObject::connect(worker, &RsiWorker::firstFrameReceived, &app,
                     [worker, offset, &appliedOnce] {
        const bool first = !appliedOnce;
        appliedOnce = true;
        std::printf("first frame received%s\n",
                    first ? "" : " (after restart; target not re-applied)");
        std::fflush(stdout);
        if (first && offset != 0.0) {
            Pose t = state.snapshot().actual;
            t.x += offset;
            std::printf("applying target X=%.3f, tracking on\n", t.x);
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
    if (restartAtMs > 0) {
        QTimer::singleShot(restartAtMs, &app, [worker] {
            // 快照必须早于 stop()：stop() 会发布一份空的 StatusSnapshot，
            // 那会抹掉 accum 的可见值（控制器内部的 m_accum 不受影响）。
            dumpSnapshot("[before-restart]", state.snapshot());
            std::printf("restart: invoking stop() then start() (queued)\n");
            std::fflush(stdout);
            QMetaObject::invokeMethod(worker, "stop", Qt::QueuedConnection);
            QMetaObject::invokeMethod(worker, "start", Qt::QueuedConnection);
        });
        // 重启后 stop() 发布的空快照要等下一帧才被真实数据覆盖，
        // 因此隔 1s 再采一次。
        QTimer::singleShot(restartAtMs + 1000, &app, [] {
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
