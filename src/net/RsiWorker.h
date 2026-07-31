#pragma once
#include <QElapsedTimer>
#include <QHostAddress>
#include <QObject>
#include <QTimer>
#include <QUdpSocket>
#include "core/AppConfig.h"
#include "core/PoseController.h"
#include "net/SharedState.h"

// 运行在独立通信线程。绝不触碰 GUI 对象，绝不做文件 IO。
class RsiWorker : public QObject
{
    Q_OBJECT
public:
    RsiWorker(const AppConfig &cfg, SharedState *state, SampleRing *ring,
              QObject *parent = nullptr);

public slots:
    void start();
    void stop();
    void applyTarget(Pose t);
    void setTracking(bool on);
    void resetToActual();
    void applyConfig(AppConfig cfg);

signals:
    void bindFailed(QString reason);
    void listening();
    void firstFrameReceived();

private slots:
    void onDatagram();
    void onWatchdog();

private:
    void publishSnapshot(const Pose &actual, const Pose &err,
                         quint64 ipoc, bool connected);

    AppConfig      m_cfg;
    SharedState   *m_state = nullptr;
    SampleRing    *m_ring  = nullptr;
    QUdpSocket    *m_sock  = nullptr;
    QTimer        *m_watchdog = nullptr;
    PoseController m_ctl;

    QHostAddress m_peerAddr;
    quint16      m_peerPort = 0;

    bool    m_haveFirstFrame = false;
    quint64 m_lastIpoc       = 0;
    int     m_missed         = 0;
    quint64 m_frameCount     = 0;
    double  m_maxReplyUs     = 0.0;

    QElapsedTimer m_sessionTimer;   // 曲线时间轴
    QElapsedTimer m_cycleTimer;     // 实测周期
    bool          m_cycleTimerValid = false;
    double        m_measuredCycleMs = 0.0;
};
