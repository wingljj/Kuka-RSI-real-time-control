#include "core/IpocTracker.h"

IpocEvent IpocTracker::classify(quint64 ipoc)
{
    IpocEvent ev;
    if (!m_haveFirst) {
        m_haveFirst = true;
        m_lastGood  = ipoc;
        ev.kind = IpocEvent::First;
        return ev;
    }
    if (ipoc == m_lastGood) {
        ev.kind = IpocEvent::Duplicate;      // 不推进 lastGood
        return ev;
    }
    if (ipoc < m_lastGood) {
        ev.kind = IpocEvent::Backward;       // 不推进 lastGood
        return ev;
    }
    // ipoc > m_lastGood
    if (ipoc == m_lastGood + 1) {
        ev.kind = IpocEvent::Normal;
    } else {
        ev.kind     = IpocEvent::Gap;
        ev.gapCount = ipoc - m_lastGood - 1;
    }
    m_lastGood = ipoc;
    return ev;
}

void IpocTracker::reset()
{
    m_haveFirst = false;
    m_lastGood  = 0;
}
