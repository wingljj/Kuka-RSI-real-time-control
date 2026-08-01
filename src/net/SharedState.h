#pragma once
#include <QMutex>
#include <QMutexLocker>
#include <QString>
#include <algorithm>
#include <array>
#include <cstring>
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
    quint32    peerIp4      = 0;    // 对端 IPv4（0=未锁定）
    quint16    peerPort     = 0;    // 对端端口（0=未锁定）
    quint64    lifetimeLost = 0;    // 累计丢包（区别于连续 missedCount）
    Pose       lastDelta;           // 最近一帧 RKorr 增量
    double     cycleMeanMs  = 0.0;  // 周期均值（最近 256 样本窗口）
    double     cycleMaxMs   = 0.0;  // 周期最大
    double     cycleP99Ms   = 0.0;  // 周期 P99
    quint64    krcDelay        = 0;   // KRC 统计的迟到/丢失回包数（<Delay D=...>）
    int        peerRejected    = 0;   // 被对端锁定丢弃的异源帧数
    int        sendFails       = 0;   // writeDatagram 连续失败计数
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
