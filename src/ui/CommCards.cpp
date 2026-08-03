#include "ui/CommCards.h"
#include <QGridLayout>
#include <QFrame>
#include <QPalette>
#include <QVBoxLayout>

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
        QFont titleF = c.title->font();
        titleF.setBold(true);
        c.title->setFont(titleF);
        v->addWidget(c.title);
        // 两行读数用系统等宽字体，不点名 Consolas：找不到该族时 QFont 不报错，
        // 只会静默换成比例字体，两张卡片上下两行的数字从此不成列。
        c.line1 = new QLabel("--", frame);
        c.line1->setFont(uilogic::monospaceFont());
        v->addWidget(c.line1);
        c.line2 = new QLabel("", frame);
        c.line2->setFont(uilogic::monospaceFont());
        v->addWidget(c.line2);
        grid->addWidget(frame, 0, col);
    };

    makeCard(m_cycle, "周期 ms", 0);
    makeCard(m_reply, "回包 µs", 1);
    makeCard(m_loss,  "丢包",    2);
    makeCard(m_ipoc,  "IPOC",    3);
}

void CommCards::Card::setSeverity(uilogic::Severity sev)
{
    // 只给两行读数上色，标题保持系统默认——标题是「这张卡片在说什么」，
    // 与好坏无关。色值统一走 severityColor：这里原先用的是 Bootstrap 那组
    // （#28a745/#ffc107/#dc3545），而 QSS 用 Tailwind 那组，同一个「正常」
    // 在界面上是两种绿。
    const QColor fg = uilogic::severityColor(sev);
    for (QLabel *l : {line1, line2}) {
        if (l->palette().color(QPalette::WindowText) == fg)
            continue;   // 每 refreshMs 调用一次，颜色没变就别触发重绘
        QPalette p = l->palette();
        p.setColor(QPalette::WindowText, fg);
        l->setPalette(p);
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
        m_cycle.setSeverity(uilogic::Severity::Idle);
    else if (cycleBad)
        m_cycle.setSeverity(uilogic::Severity::Warn);
    else
        m_cycle.setSeverity(uilogic::Severity::Ok);

    // ── 回包耗时 ──
    const bool replyBad = s.maxReplyUs > 1000.0;
    m_reply.line1->setText(
        QStringLiteral("当前 %1").arg(s.maxReplyUs, 0, 'f', 0));
    m_reply.line2->setText(
        QStringLiteral("最大 %1").arg(s.maxReplyUs, 0, 'f', 0));
    if (!s.connected)
        m_reply.setSeverity(uilogic::Severity::Idle);
    else if (replyBad)
        m_reply.setSeverity(uilogic::Severity::Warn);
    else
        m_reply.setSeverity(uilogic::Severity::Ok);

    // ── 丢包 ──
    m_loss.line1->setText(
        QStringLiteral("连续 %1").arg(s.missedCount));
    m_loss.line2->setText(
        QStringLiteral("累计 %1").arg(s.lifetimeLost));
    if (!s.connected)
        m_loss.setSeverity(uilogic::Severity::Idle);
    else if (s.missedCount > 0 || s.lifetimeLost > 0)
        m_loss.setSeverity(uilogic::Severity::Fault);
    else
        m_loss.setSeverity(uilogic::Severity::Ok);

    // ── IPOC ──
    const bool ipocBad = s.missedCount > 0;  // 丢包即 IPOC 不连续
    m_ipoc.line1->setText(QString::number(s.ipoc));
    m_ipoc.line2->setText(
        ipocBad ? "不连续" : (s.connected ? "连续递增" : "—"));
    if (!s.connected)
        m_ipoc.setSeverity(uilogic::Severity::Idle);
    else if (ipocBad)
        m_ipoc.setSeverity(uilogic::Severity::Fault);
    else
        m_ipoc.setSeverity(uilogic::Severity::Ok);
}
