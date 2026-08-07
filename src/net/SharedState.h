#pragma once
#include <QMutex>
#include <QMutexLocker>
#include <QString>
#include <algorithm>
#include <array>
#include <cstring>
#include "core/PoseController.h"
#include "core/Pose.h"

// 7 态会话/控制状态（UI 显示 + 行为语义）。由 RsiWorker 在 publishSnapshot 中
// 组合：Fault > StaleFrame > Syncing(首帧瞬间) > Tracking > Ready >
// WaitingFirstFrame > Disconnected。TrackState 保留给 PoseController 内部使用。
enum class ControlState {
    Disconnected, WaitingFirstFrame, Syncing, Ready, Tracking, StaleFrame, Fault,
};

// 跟踪质量：独立于控制状态，评估误差与累积修正相对限值的比例。
enum class TrackingQuality {
    Normal,       // 误差 < 50% 限值
    LargeError,   // 误差 50–80% 限值
    NearLimit,    // 误差 80–100% 限值
    OverLimit,    // 超过限值
    Inactive,     // 未跟踪（状态不是 Tracking）
};

struct StatusSnapshot
{
    Pose       actual;
    Pose       target;
    Pose       error;  // 位置误差 = target − actual；姿态误差 = SO(3) 旋转向量分量（世界坐标，度）
    Pose       accum;
    quint64    ipoc            = 0;
    ControlState state         = ControlState::Disconnected;
    QString    faultReason;
    int        missedCount     = 0;
    double     measuredCycleMs = 0.0;
    // 回包耗时分两个字段。只有 maxReplyUs 的话，「当前回包耗时」这个量在快照里
    // 根本不存在：它是会话内单调最大值，一次瞬时尖峰会永久留在读数上，界面
    // 也就无法用来观察链路是否已经恢复——超过门限后卡片会永远锁在告警色，
    // 把一条历史告警当成当前状态呈现。所以瞬时值必须单独有一份。
    double     replyUs         = 0.0;  // 最近一帧的收到→发出耗时
    double     maxReplyUs      = 0.0;  // 会话内最大（start() 归零）
    quint64    frameCount      = 0;
    bool       connected       = false;
    quint32    peerIp4      = 0;    // 对端 IPv4（0=未锁定）
    quint16    peerPort     = 0;    // 对端端口（0=未锁定）
    quint64    lifetimeLost = 0;    // 累计丢包（区别于连续 missedCount）
    quint64    trimCount    = 0;    // 到位精修累计执行次数（事件日志按增量记条）
    Pose       lastDelta;           // 最近一帧 RKorr 增量
    double     cycleMeanMs  = 0.0;  // 周期均值（最近 256 样本窗口）
    double     cycleMaxMs   = 0.0;  // 周期最大
    double     cycleP99Ms   = 0.0;  // 周期 P99
    quint64    krcDelay        = 0;   // KRC 统计的迟到/丢失回包数（<Delay D=...>）
    int        peerRejected    = 0;   // 被对端锁定丢弃的异源帧数
    int        sendFails       = 0;   // writeDatagram 连续失败计数

    // 跟踪质量与限值比例（由 publishSnapshot 计算）
    TrackingQuality trackingQuality = TrackingQuality::Inactive;
    double    accumPosPct     = 0.0;  // 累积位置 / 位置限值（0–∞）
    double    accumRotPct     = 0.0;  // 累积姿态 / 姿态限值（0–∞）
    double    errorPosPct     = 0.0;  // 位置误差 / 位置限值（0–∞）
    double    errorRotPct     = 0.0;  // 姿态误差 / 姿态限值（0–∞）
    bool      accumOverLimit  = false; // 累计修正超限（accumPosPct>=1 或 accumRotPct>=1）
};

// 通信线程 publish，GUI 线程 snapshot。锁持有时间仅够一次结构体拷贝。
class SharedState
{
public:
    // 注意：m_snap = s 会销毁前一个 m_snap.faultReason。稳态下 faultReason 是
    // 默认构造的 QString（内部指针为空），赋值与析构都不碰堆；但若某一帧它
    // 非空，comm 线程上的引用计数递减可能归零并触发 free()。因此 faultReason
    // 只允许在状态发生转换时赋值，绝不可每周期构造新字符串——那会让本该
    // 无分配的实时路径每帧都进堆。
    void publish(const StatusSnapshot &s)
    {
        QMutexLocker lock(&m_mutex);
        m_snap = s;
    }

    StatusSnapshot snapshot() const
    {
        QMutexLocker lock(&m_mutex);
        return m_snap;
    }

private:
    mutable QMutex m_mutex;
    StatusSnapshot m_snap;
};

struct ChartSample
{
    double tSec       = 0.0;
    double posErrNorm = 0.0;   // mm
    double rotErrNorm = 0.0;   // deg
};

// 定容环形缓冲：push 无分配，可在实时路径调用。
class SampleRing
{
public:
    static constexpr int kCapacity = 4096;

    void push(const ChartSample &s)
    {
        QMutexLocker lock(&m_mutex);
        m_buf[m_head] = s;
        m_head = (m_head + 1) % kCapacity;
        if (m_size < kCapacity)
            ++m_size;
    }

    // 按时间先后写入 dst，返回实际写入数量。
    int copyOut(ChartSample *dst, int maxCount) const
    {
        if (!dst || maxCount <= 0)
            return 0;                   // 负数会让下面的 % 触发有符号溢出
        QMutexLocker lock(&m_mutex);
        const int n = std::min(m_size, maxCount);
        const int start = (m_head - n + kCapacity) % kCapacity;
        // 用两段连续 memcpy 代替逐元素取模循环。持锁时长直接决定 comm 线程
        // push() 的最坏等待：GUI 线程是普通优先级且 QMutex 无优先级继承，
        // 若它在持锁期间被 OS 抢占，push() 会一直停到它被重新调度——最多
        // 一个时间片（默认定时器精度下约 10–15ms），超过 4–12ms 的 RSI 周期，
        // 于是迟到回复→丢包→停机。缩短持锁窗口是直接的风险削减。
        // ChartSample 是三个 double，trivially copyable，memcpy 安全。
        const int first = std::min(n, kCapacity - start);
        std::memcpy(dst, &m_buf[start],
                    size_t(first) * sizeof(ChartSample));
        if (n > first)
            std::memcpy(dst + first, &m_buf[0],
                        size_t(n - first) * sizeof(ChartSample));
        return n;
    }

    void clear()
    {
        QMutexLocker lock(&m_mutex);
        m_head = 0;
        m_size = 0;
    }

private:
    mutable QMutex m_mutex;
    std::array<ChartSample, kCapacity> m_buf{};
    int m_head = 0;
    int m_size = 0;
};
