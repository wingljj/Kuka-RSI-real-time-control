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

    // ── 前向：IPOC 的步长不是 1，必须学习 ──
    // 真机 KRC 的 IPOC 是毫秒计数，每帧 +cycle_ms（4ms 模式 +4、12ms 模式
    // +12）；本项目的模拟器曾经每帧 +1。假设 +1 的实现在真机上把每一个
    // 健康帧都判成 Gap(3)，连续丢包计数永不清零，一到 watchdog_miss_limit
    // 就静默停跟踪（2026-08-04 现场：连续 32535、累计 83208，均为 3 的
    // 倍数，机械臂"纹丝不动且无报错"）。
    // 步长取历史最小正增量：首个增量恰是真实丢包时会学到偏大步长，但
    // 后续任一正常帧都会把它收紧回真值——误差只在启动瞬间，方向是少计。
    const quint64 delta = ipoc - m_lastGood;
    if (m_step == 0 || delta < m_step)
        m_step = delta;

    if (delta == m_step) {
        ev.kind = IpocEvent::Normal;
    } else {
        ev.kind = IpocEvent::Gap;
        // 缺失帧数 = delta/step - 1。非整倍数的余数说明节拍紊乱，至少计 1。
        ev.gapCount = qMax<quint64>(1, delta / m_step - 1);
    }
    m_lastGood = ipoc;
    return ev;
}

void IpocTracker::reset()
{
    m_haveFirst = false;
    m_lastGood  = 0;
    m_step      = 0;
}
