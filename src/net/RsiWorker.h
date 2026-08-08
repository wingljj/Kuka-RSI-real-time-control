#pragma once
#include <QElapsedTimer>
#include <array>
#include <QHostAddress>
#include <QObject>
#include <QTimer>
#include <QUdpSocket>
#include "core/IpocTracker.h"
#include "core/AppConfig.h"
#include "core/ForceController.h"
#include "core/PoseController.h"
#include "core/SriDriver.h"
#include "net/SharedState.h"

// 运行在独立通信线程。绝不触碰 GUI 对象，绝不做文件 IO。
class RsiWorker : public QObject
{
    Q_OBJECT
public:
    RsiWorker(const AppConfig &cfg, SharedState *state, SampleRing *ring,
              ForceRing *forceRing = nullptr, QObject *parent = nullptr);

    // 【连接方式契约】以下所有槽与信号都必须以 Qt::QueuedConnection 连接。
    // 不支持直连：
    //  - applyTarget/setTracking/applyConfig 会改写 m_ctl / m_cfg，而通信线程
    //    正在 step() 与 buildSen() 中读取它们。applyConfig 尤其危险：AppConfig
    //    含两个 QString，直连会在通信线程读 senType 的同时改写其引用计数，
    //    那是堆损坏而不仅是脏读。
    //  - 直连的 bindFailed/listening/firstFrameReceived 槽若调用 stop()，会在
    //    onDatagram() 的循环体中途把 m_sock/m_watchdog 置空。
    //  - 力控七个槽（setForceMode/applyForceConfig/zeroForceSensor/startSri/
    //    stopSri/enableForceControl/disableForceControl）同样必须排队：它们
    //    改写 m_forceMode / m_cfg.forceControl，且 SriDriver 与其 QTcpSocket
    //    都归属通信线程——直连会让槽在 GUI 线程执行，跨线程触碰 socket 与
    //    窗口均值累加器（通信线程每周期在 drainAccumulator() 里与它互取）。
    //    enableForceControl 还会读 m_sriDriver 与 m_lastActual，只能在通信
    //    线程执行。
public slots:
    void start();
    void stop();
    void applyTarget(Pose t);
    void setTracking(bool on);
    void resetToActual();
    void applyConfig(AppConfig cfg);
    // ── 力控（ForceControlConfig 已注册元类型，可经队列连接传参）──
    void setForceMode(bool on);
    void applyForceConfig(const ForceControlConfig &cfg);
    void zeroForceSensor();
    void startSri();
    void stopSri();
    // 使能/停止力控。enable 取一帧新鲜 SRI 窗口均值作偏置、变换到 BASE 系
    // 后交给 ForceController::enable（记录台账起点 + 预填充滤波器）；取不到
    // 新鲜数据或力控已活跃时静默返回。两者都必须在通信线程执行（读
    // m_sriDriver / m_lastActual / m_forceCtl），经队列连接到达。
    void enableForceControl();
    void disableForceControl();

signals:
    void bindFailed(QString reason);
    void listening();
    void firstFrameReceived();

private slots:
    void onDatagram();
    void onWatchdog();

private:
    void publishSnapshot(const Pose &actual, const Pose &err,
                         quint64 ipoc, bool connected, bool wasFirstFrame);

    AppConfig      m_cfg;
    SharedState   *m_state = nullptr;
    SampleRing    *m_ring  = nullptr;
    ForceRing     *m_forceRing = nullptr;  // 力曲线环形缓冲（可选，nullptr=不记录）
    QUdpSocket    *m_sock  = nullptr;
    QTimer        *m_watchdog = nullptr;
    PoseController m_ctl;

    // ── 力控（与位姿跟踪互斥：onDatagram 按 m_forceMode 择一执行）──
    SriDriver       *m_sriDriver = nullptr;
    ForceController  m_forceCtl;
    bool m_forceMode = false;   // true = 力控页活跃（setForceMode 槽置位）
    WrenchFrame m_sriLatest;    // 最近一次从 SRI 取走的窗口均值（zeroForceSensor 回退）
    int m_forceSpeedClampCount = 0;  // 连续 vmax 饱和帧计数（力控超速检测）

    IpocTracker m_ipocTracker;

    // 会话安全
    QHostAddress m_peerAddr;
    quint16      m_peerPort     = 0;
    bool         m_peerLocked   = false;
    int          m_peerRejected = 0;
    int          m_sendFails    = 0;
    quint64      m_lastDelay    = 0;
    int          m_delayRising  = 0;

    static constexpr int kMaxBurst = 8;   // 每轮 onDatagram 最多处理的积压帧数

    int     m_missed         = 0;
    quint64 m_frameCount     = 0;
    double  m_replyUs        = 0.0;   // 最近一帧回包耗时
    double  m_maxReplyUs     = 0.0;   // 会话内最大

    // 反馈异常剔除：上一有效帧位姿 + 连续 stale 帧计数
    Pose m_prevValidPose;
    bool m_havePrevPose = false;
    int  m_staleCount   = 0;

    QElapsedTimer m_sessionTimer;   // 曲线时间轴
    QElapsedTimer m_cycleTimer;     // 实测周期
    QElapsedTimer m_sinceLastFrame;   // 最后一次收到有效帧的时刻
    // 上一次调用 m_ctl.step() 的时刻 = 步长预算的时间基准。
    // 刻意不复用 m_cycleTimer（它测的是帧间隔）：被判 stale、重复、回退的帧
    // 照样重启 m_cycleTimer 却不发修正，用它算预算会把那些帧占用的时间白白
    // 扣掉。预算的物理含义是"距上一次真正发出修正过去了多久"，基准必须是
    // step 而不是收帧。invalidate() 状态表示"基准不可信"，见 onWatchdog()。
    QElapsedTimer m_sinceLastStep;
    bool          m_cycleTimerValid = false;
    double        m_measuredCycleMs = 0.0;

    // 诊断：累计丢包（会话内只增不减）、最近增量、周期直方（定容，实时无分配）
    quint64 m_lifetimeLost = 0;
    Pose    m_lastDelta;
    static constexpr int kCycleHist = 256;
    std::array<double, kCycleHist> m_cycleHist{};
    int m_cycleHead  = 0;
    int m_cycleCount = 0;

    Pose m_lastActual;   // 最近一帧的实际位姿，供 resetToActual() 槽使用
};
