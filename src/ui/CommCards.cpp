#include "ui/CommCards.h"
#include <QGridLayout>
#include <QFrame>

CommCards::CommCards(QWidget *parent) : QWidget(parent)
{
    auto *grid = new QGridLayout(this);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(4);

    auto makeCard = [&](Card &c, const QString &title, int col) {
        auto *frame = new QFrame(this);
        frame->setFrameShape(QFrame::StyledPanel);
        auto *v = new QVBoxLayout(frame);
        v->setContentsMargins(6, 4, 6, 4);
        v->setSpacing(1);
        c.title = new QLabel(title, frame);
        c.title->setStyleSheet("font-size: 8px; color: #888; font-weight: bold;");
        v->addWidget(c.title);
        c.line1 = new QLabel("--", frame);
        c.line1->setStyleSheet("font-size: 11px; font-family: Consolas, monospace;");
        v->addWidget(c.line1);
        c.line2 = new QLabel("", frame);
        c.line2->setStyleSheet("font-size: 8px; color: #666; font-family: Consolas, monospace;");
        v->addWidget(c.line2);
        grid->addWidget(frame, 0, col);
    };

    makeCard(m_cycle, "周期 ms", 0);
    makeCard(m_reply, "回包 µs", 1);
    makeCard(m_loss,  "丢包",    2);
    makeCard(m_ipoc,  "IPOC",    3);
}

void CommCards::Card::setColors(const char *bg, const char *fg)
{
    if (auto *p = title->parentWidget()) {
        p->setStyleSheet(QStringLiteral(
            "QFrame { background-color: %1; border: 1px solid %2; "
            "border-radius: 4px; }")
            .arg(QLatin1String(bg), QLatin1String(fg)));
    }
}

void CommCards::updateFrom(const StatusSnapshot &s, double configuredCycleMs)
{
    // ── 周期 ──
    const bool cycleBad = (s.measuredCycleMs > 0.0 && configuredCycleMs > 0.0
                           && std::fabs(s.measuredCycleMs - configuredCycleMs)
                                  > 0.10 * configuredCycleMs);
    m_cycle.line1->setText(
        QStringLiteral("均值 %1").arg(s.cycleMeanMs, 0, 'f', 2));
    m_cycle.line2->setText(
        QStringLiteral("P99 %1").arg(s.cycleP99Ms, 0, 'f', 2));
    if (!s.connected)
        m_cycle.setColors("#f5f5f5", "#ccc");
    else if (cycleBad)
        m_cycle.setColors("#fff3cd", "#ffc107");
    else
        m_cycle.setColors("#d4edda", "#28a745");

    // ── 回包耗时 ──
    const bool replyBad = s.maxReplyUs > 1000.0;
    m_reply.line1->setText(
        QStringLiteral("当前 %1").arg(s.maxReplyUs, 0, 'f', 0));
    m_reply.line2->setText(
        QStringLiteral("最大 %1").arg(s.maxReplyUs, 0, 'f', 0));
    if (!s.connected)
        m_reply.setColors("#f5f5f5", "#ccc");
    else if (replyBad)
        m_reply.setColors("#fff3cd", "#ffc107");
    else
        m_reply.setColors("#d4edda", "#28a745");

    // ── 丢包 ──
    m_loss.line1->setText(
        QStringLiteral("连续 %1").arg(s.missedCount));
    m_loss.line2->setText(
        QStringLiteral("累计 %1").arg(s.lifetimeLost));
    if (!s.connected)
        m_loss.setColors("#f5f5f5", "#ccc");
    else if (s.missedCount > 0 || s.lifetimeLost > 0)
        m_loss.setColors("#f8d7da", "#dc3545");
    else
        m_loss.setColors("#d4edda", "#28a745");

    // ── IPOC ──
    const bool ipocBad = s.missedCount > 0;  // 丢包即 IPOC 不连续
    m_ipoc.line1->setText(QString::number(s.ipoc));
    m_ipoc.line2->setText(
        ipocBad ? "不连续" : (s.connected ? "连续递增" : "—"));
    if (!s.connected)
        m_ipoc.setColors("#f5f5f5", "#ccc");
    else if (ipocBad)
        m_ipoc.setColors("#f8d7da", "#dc3545");
    else
        m_ipoc.setColors("#d4edda", "#28a745");
}
