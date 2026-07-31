#pragma once
#include <QMutex>
#include <QMutexLocker>
#include <QString>
#include <array>
#include "core/PoseController.h"
#include "core/Pose.h"

struct StatusSnapshot
{
    Pose       actual;
    Pose       target;
    Pose       error;
    Pose       accum;
    quint64    ipoc            = 0;
    TrackState state           = TrackState::Idle;
    QString    faultReason;
    int        missedCount     = 0;
    double     measuredCycleMs = 0.0;
    double     maxReplyUs      = 0.0;
    quint64    frameCount      = 0;
    bool       connected       = false;
};

// 通信线程 publish，GUI 线程 snapshot。锁持有时间仅够一次结构体拷贝。
class SharedState
{
public:
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
        QMutexLocker lock(&m_mutex);
        const int n = std::min(m_size, maxCount);
        const int start = (m_head - n + kCapacity) % kCapacity;
        for (int i = 0; i < n; ++i)
            dst[i] = m_buf[(start + i) % kCapacity];
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
