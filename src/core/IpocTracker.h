#pragma once
#include <QtGlobal>

// RSI IPOC 序列分类。纯状态机，无 IO、无 Qt 对象，可在通信线程内直接调用。
struct IpocEvent
{
    enum Kind { First, Normal, Gap, Duplicate, Backward };
    Kind    kind = Normal;
    quint64 gapCount = 0;   // 仅 Gap：前向跳号缺失的周期数
};

class IpocTracker
{
public:
    IpocEvent classify(quint64 ipoc);
    quint64  lastGood() const { return m_lastGood; }
    bool     haveFirst() const { return m_haveFirst; }
    // 学到的每帧 IPOC 增量（真机 = cycle_ms，模拟器 = cycle_ms；0 = 尚未学到）
    quint64  step() const { return m_step; }
    void     reset();

private:
    bool    m_haveFirst = false;
    quint64 m_lastGood  = 0;
    quint64 m_step      = 0;
};
