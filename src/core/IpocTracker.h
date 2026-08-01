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
    void     reset();

private:
    bool    m_haveFirst = false;
    quint64 m_lastGood  = 0;
};
