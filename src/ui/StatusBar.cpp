#include "ui/StatusBar.h"
#include <QFontMetrics>
#include <QHBoxLayout>
#include <QPalette>
#include <QVBoxLayout>
#include <algorithm>

StatusBar::StatusBar(QWidget *parent) : QWidget(parent)
{
    auto *v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 4);
    v->setSpacing(4);

    // ═══ 4 状态卡片 ═══
    auto *cards = new QHBoxLayout;
    cards->setSpacing(8);

    auto makeCard = [&](Card &c, const QString &title) {
        c.frame = new QFrame(this);
        // 原生边框代替原来那条 "QFrame { background-color:...; border:... }"：
        // 样式表选择器会级联到卡片内的每个 QLabel（QLabel 继承自 QFrame），
        // 于是标题和状态各自套上白底 + 边框 + 圆角，看起来像两个输入框。
        // setFrameShape 只作用于这一个部件，不会有后代。
        c.frame->setFrameShape(QFrame::StyledPanel);
        auto *cv = new QVBoxLayout(c.frame);
        cv->setContentsMargins(8, 4, 8, 4);
        cv->setSpacing(0);

        auto *top = new QHBoxLayout;
        top->setSpacing(4);
        // 图标不再限死 10x10：那是配合样式表里 8px 字号定的，回到系统字号后
        // 一个 "●" 就装不下，会被裁成半个。宽高交给 sizeHint。
        c.icon = new QLabel(this);
        c.icon->setAlignment(Qt::AlignCenter);
        top->addWidget(c.icon);
        c.label = new QLabel(title, this);
        QFont titleF = c.label->font();
        titleF.setBold(true);
        c.label->setFont(titleF);
        top->addWidget(c.label);
        top->addStretch();
        cv->addLayout(top);

        c.status = new QLabel("--", this);
        QFont statusF = c.status->font();
        statusF.setBold(true);
        c.status->setFont(statusF);
        cv->addWidget(c.status);

        // 宽度按最长状态文案算死，而不是让 sizeHint 跟着文字走：状态每
        // refreshMs 刷一次，宽度随文字变会让四张卡片在运行中左右跳动。
        // 高度反过来不设死——系统字号变大时写死的高度会把第二行裁掉。
        const QFontMetrics fm(statusF);
        int textW = 0;
        for (const QString &s : {QStringLiteral("接近限值 100%"),
                                 QStringLiteral("跟踪(超限)"),
                                 QStringLiteral("丢包 9999")})
            textW = std::max(textW, fm.horizontalAdvance(s));
        c.frame->setFixedWidth(textW + 32);

        cards->addWidget(c.frame);
    };

    makeCard(m_netCard,  "网络");
    makeCard(m_rsiCard,  "RSI通信");
    makeCard(m_ctlCard,  "控制状态");
    makeCard(m_qualCard, "跟踪质量");

    cards->addStretch();
    v->addLayout(cards);

    // 警告条。背景色改成原生边框：写死浅色背景会盖掉操作员在系统层面设的
    // 高对比度主题，而边框在任何主题下都跟着主题走。
    m_warning = new QLabel(this);
    m_warning->setWordWrap(true);
    m_warning->setFrameShape(QFrame::StyledPanel);
    QFont warnF = m_warning->font();
    warnF.setBold(true);
    m_warning->setFont(warnF);
    m_warning->hide();
    v->addWidget(m_warning);
}

void StatusBar::Card::set(const QString &iconText, const QString &statusText,
                          uilogic::Severity sev)
{
    icon->setText(iconText);
    status->setText(statusText);
    // 只改前景色，背景留给系统主题。QPalette 不会级联到后代部件，
    // 所以这里改的确实只是这两个标签的文字色。
    const QColor fg = uilogic::severityColor(sev);
    for (QLabel *l : {icon, status}) {
        if (l->palette().color(QPalette::WindowText) == fg)
            continue;   // 每 refreshMs 调用一次，颜色没变就别触发重绘
        QPalette p = l->palette();
        p.setColor(QPalette::WindowText, fg);
        l->setPalette(p);
    }
}

void StatusBar::updateFrom(const StatusSnapshot &s, bool listening)
{
    using uilogic::Severity;

    // 「监听中」「就绪」「同步中」「等待首帧」这些过渡态原先是蓝色。
    // Severity 没有蓝这一档（理由见 UiLogic.h）：它们与「未监听」同属
    // 「没出问题、还没开始跑」，一律 Idle，区分靠状态文字本身。

    // ── 网络 ──
    if (s.connected) {
        m_netCard.set("●", "已连接", Severity::Ok);
    } else if (listening) {
        m_netCard.set("◐", "监听中", Severity::Idle);
    } else {
        m_netCard.set("○", "未监听", Severity::Idle);
    }

    // ── RSI 通信 ──
    if (!s.connected) {
        m_rsiCard.set("—", "无数据", Severity::Idle);
    } else if (s.missedCount > 0 || s.peerRejected > 0) {
        m_rsiCard.set("⚠", QStringLiteral("丢包 %1").arg(s.missedCount),
                      Severity::Warn);
    } else if (s.state == ControlState::StaleFrame) {
        m_rsiCard.set("⚠", "帧异常", Severity::Warn);
    } else {
        m_rsiCard.set("●", "正常", Severity::Ok);
    }

    // ── 控制状态 ──
    switch (s.state) {
    case ControlState::Fault:
        m_ctlCard.set("✕", "故障锁存", Severity::Fault); break;
    case ControlState::Tracking:
        m_ctlCard.set("▶", s.accumOverLimit ? "跟踪(超限)" : "跟踪中",
                      s.accumOverLimit ? Severity::Fault : Severity::Ok); break;
    case ControlState::Ready:
        m_ctlCard.set("✓", "就绪", Severity::Idle); break;
    case ControlState::Syncing:
        m_ctlCard.set("↻", "同步中", Severity::Idle); break;
    case ControlState::WaitingFirstFrame:
        m_ctlCard.set("…", "等待首帧", Severity::Idle); break;
    case ControlState::StaleFrame:
        m_ctlCard.set("⚠", "帧异常", Severity::Warn); break;
    default:
        m_ctlCard.set("—", "无", Severity::Idle); break;
    }

    // ── 跟踪质量 ──
    switch (s.trackingQuality) {
    case TrackingQuality::Normal:
        m_qualCard.set("●", "正常", Severity::Ok); break;
    case TrackingQuality::LargeError:
        m_qualCard.set("●", QStringLiteral("偏差 %1%").arg(int(s.errorPosPct*100)),
                       Severity::Warn); break;
    case TrackingQuality::NearLimit:
        m_qualCard.set("⚠",
                       QStringLiteral("接近限值 %1%")
                           .arg(int(std::max(s.accumPosPct, s.errorPosPct)*100)),
                       Severity::Warn); break;
    case TrackingQuality::OverLimit:
        m_qualCard.set("✕", "超限", Severity::Fault); break;
    default:
        m_qualCard.set("—", "无数据", Severity::Idle); break;
    }
}

void StatusBar::setWarning(const QString &text, bool isFault)
{
    m_warning->setText(text);
    QPalette p = m_warning->palette();
    p.setColor(QPalette::WindowText,
               uilogic::severityColor(isFault ? uilogic::Severity::Fault
                                              : uilogic::Severity::Idle));
    m_warning->setPalette(p);
    m_warning->setVisible(!text.isEmpty());
}
