#include "ui/StatusBar.h"
#include <QHBoxLayout>
#include <QVBoxLayout>

StatusBar::StatusBar(QWidget *parent) : QWidget(parent)
{
    auto *v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 4);
    v->setSpacing(4);

    // ═══ 上排：4 状态卡片 ═══
    auto *cards = new QHBoxLayout;
    cards->setSpacing(8);

    auto makeCard = [&](Card &c, const QString &title) {
        c.frame = new QFrame(this);
        c.frame->setFixedSize(130, 50);
        c.frame->setStyleSheet(
            "QFrame { background-color: #FFFFFF; border: 1px solid #D9E0E7; "
            "border-radius: 6px; }");
        auto *cv = new QVBoxLayout(c.frame);
        cv->setContentsMargins(8, 4, 8, 4);
        cv->setSpacing(0);

        auto *top = new QHBoxLayout;
        top->setSpacing(4);
        c.icon = new QLabel(this);
        c.icon->setFixedSize(10, 10);
        c.icon->setAlignment(Qt::AlignCenter);
        top->addWidget(c.icon);
        c.label = new QLabel(title, this);
        c.label->setStyleSheet("font-size: 9px; color: #64748B; font-weight: bold;");
        top->addWidget(c.label);
        top->addStretch();
        cv->addLayout(top);

        c.status = new QLabel("--", this);
        c.status->setStyleSheet("font-size: 12px; font-weight: bold; color: #1F2937;");
        cv->addWidget(c.status);
        cards->addWidget(c.frame);
    };

    makeCard(m_netCard,  "网络");
    makeCard(m_rsiCard,  "RSI通信");
    makeCard(m_ctlCard,  "控制状态");
    makeCard(m_qualCard, "跟踪质量");

    cards->addStretch();
    v->addLayout(cards);

    // ═══ 下排：流程步骤条 ═══
    auto *flow = new QHBoxLayout;
    flow->setSpacing(0);
    flow->setContentsMargins(4, 0, 4, 0);
    flow->addStretch();

    const char *stepNames[5] = {"监听", "等待连接", "同步位姿", "使能跟踪", "停止跟踪"};
    for (int i = 0; i < 5; ++i) {
        m_steps[i].icon = new QLabel(this);
        m_steps[i].icon->setFixedSize(20, 20);
        m_steps[i].icon->setAlignment(Qt::AlignCenter);
        m_steps[i].icon->setStyleSheet(
            "background-color: #E5E7EB; color: #9CA3AF; border-radius: 10px; "
            "font-size: 10px; font-weight: bold;");
        m_steps[i].icon->setText(QString::number(i + 1));
        flow->addWidget(m_steps[i].icon);

        m_steps[i].label = new QLabel(stepNames[i], this);
        m_steps[i].label->setStyleSheet("font-size: 9px; color: #9CA3AF; padding: 0 3px;");
        flow->addWidget(m_steps[i].label);

        if (i < 4) {
            auto *arrow = new QLabel("→", this);
            arrow->setStyleSheet("color: #D1D5DB; font-size: 10px; padding: 0 2px;");
            flow->addWidget(arrow);
            m_arrows[i] = arrow;
        }
    }
    flow->addStretch();
    v->addLayout(flow);

    // 警告条
    m_warning = new QLabel(this);
    m_warning->setWordWrap(true);
    m_warning->setStyleSheet(
        "font-size: 9px; padding: 3px 8px; border-radius: 4px;");
    m_warning->hide();
    v->addWidget(m_warning);
}

void StatusBar::Card::set(const QString &iconText, const QString &labelText,
                          const QString &statusText, const QString &bgColor,
                          const QString &fgColor)
{
    icon->setText(iconText);
    icon->setStyleSheet(QStringLiteral(
        "font-size: 8px; font-weight: bold; color: %1; background: transparent;")
        .arg(fgColor));
    if (label) {} // title unchanged
    status->setText(statusText);
    status->setStyleSheet(QStringLiteral(
        "font-size: 12px; font-weight: bold; color: %1;").arg(fgColor));
    frame->setStyleSheet(QStringLiteral(
        "QFrame { background-color: %1; border: 1px solid %2; "
        "border-radius: 6px; }")
        .arg(bgColor, fgColor));
}

void StatusBar::setStepActive(int idx, bool active, bool enabled)
{
    const QString bg = active   ? "#2563EB"
                       : enabled ? "#FFFFFF"
                                 : "#F3F4F6";
    const QString fg = active   ? "#FFFFFF"
                       : enabled ? "#1F2937"
                                 : "#D1D5DB";
    const QString border = active   ? "#2563EB"
                           : enabled ? "#D9E0E7"
                                     : "#E5E7EB";
    m_steps[idx].icon->setStyleSheet(QStringLiteral(
        "background-color: %1; color: %2; border: 2px solid %3; "
        "border-radius: 10px; font-size: 10px; font-weight: bold;")
        .arg(bg, fg, border));
    m_steps[idx].label->setStyleSheet(QStringLiteral(
        "font-size: 9px; color: %1; padding: 0 3px; font-weight: %2;")
        .arg(active ? "#2563EB" : enabled ? "#64748B" : "#D1D5DB",
             active ? "bold" : "normal"));
}

void StatusBar::updateFrom(const StatusSnapshot &s, bool listening)
{
    // ── 网络 ──
    if (s.connected) {
        m_netCard.set("●", "网络", "已连接", "#F0FDF4", "#16A34A");
    } else if (listening) {
        m_netCard.set("◐", "网络", "监听中", "#EFF6FF", "#2563EB");
    } else {
        m_netCard.set("○", "网络", "未监听", "#F9FAFB", "#9CA3AF");
    }

    // ── RSI 通信 ──
    if (!s.connected) {
        m_rsiCard.set("—", "RSI通信", "无数据", "#F9FAFB", "#9CA3AF");
    } else if (s.missedCount > 0 || s.peerRejected > 0) {
        m_rsiCard.set("⚠", "RSI通信",
                      QStringLiteral("丢包 %1").arg(s.missedCount),
                      "#FFFBEB", "#D97706");
    } else if (s.state == ControlState::StaleFrame) {
        m_rsiCard.set("⚠", "RSI通信", "帧异常", "#FFFBEB", "#D97706");
    } else {
        m_rsiCard.set("●", "RSI通信", "正常", "#F0FDF4", "#16A34A");
    }

    // ── 控制状态 ──
    switch (s.state) {
    case ControlState::Fault:
        m_ctlCard.set("✕", "控制状态", "故障锁存", "#FEF2F2", "#DC2626"); break;
    case ControlState::Tracking:
        m_ctlCard.set("▶", "控制状态",
                      s.accumOverLimit ? "跟踪(超限)" : "跟踪中",
                      s.accumOverLimit ? "#FEF2F2" : "#F0FDF4",
                      s.accumOverLimit ? "#DC2626" : "#16A34A"); break;
    case ControlState::Ready:
        m_ctlCard.set("✓", "控制状态", "就绪", "#EFF6FF", "#2563EB"); break;
    case ControlState::Syncing:
        m_ctlCard.set("↻", "控制状态", "同步中", "#EFF6FF", "#2563EB"); break;
    case ControlState::WaitingFirstFrame:
        m_ctlCard.set("…", "控制状态", "等待首帧", "#EFF6FF", "#2563EB"); break;
    case ControlState::StaleFrame:
        m_ctlCard.set("⚠", "控制状态", "帧异常", "#FFFBEB", "#D97706"); break;
    default:
        m_ctlCard.set("—", "控制状态", "无", "#F9FAFB", "#9CA3AF"); break;
    }

    // ── 跟踪质量 ──
    switch (s.trackingQuality) {
    case TrackingQuality::Normal:
        m_qualCard.set("●", "跟踪质量", "正常", "#F0FDF4", "#16A34A"); break;
    case TrackingQuality::LargeError:
        m_qualCard.set("●", "跟踪质量",
                       QStringLiteral("偏差 %1%").arg(int(s.errorPosPct*100)),
                       "#FFFBEB", "#D97706"); break;
    case TrackingQuality::NearLimit:
        m_qualCard.set("⚠", "跟踪质量",
                       QStringLiteral("接近限值 %1%").arg(int(std::max(s.accumPosPct,s.errorPosPct)*100)),
                       "#FFF7ED", "#D97706"); break;
    case TrackingQuality::OverLimit:
        m_qualCard.set("✕", "跟踪质量", "超限", "#FEF2F2", "#DC2626"); break;
    default:
        m_qualCard.set("—", "跟踪质量", "无数据", "#F9FAFB", "#9CA3AF"); break;
    }

    // ── 流程步骤条 ──
    const int activeStep = [&]() -> int {
        switch (s.state) {
        case ControlState::Disconnected:              return listening ? 1 : 0;
        case ControlState::WaitingFirstFrame:
        case ControlState::Syncing:                    return 2;
        case ControlState::Ready:                      return 3;
        case ControlState::Tracking:                   return 4;
        case ControlState::StaleFrame:                 return 2;
        case ControlState::Fault:                      return -1;
        default:                                       return 0;
        }
    }();
    const bool listeningFlag = listening || s.connected;
    setStepActive(0, activeStep == 0, true);
    setStepActive(1, activeStep == 1, listeningFlag);
    setStepActive(2, activeStep == 2, s.connected);
    setStepActive(3, activeStep == 3, s.state == ControlState::Ready || s.state == ControlState::Tracking);
    setStepActive(4, activeStep == 4, s.state == ControlState::Tracking);

    // 故障时特殊处理：所有步骤灰色
    if (s.state == ControlState::Fault) {
        for (int i = 0; i < 5; ++i) setStepActive(i, false, false);
    }
}

void StatusBar::setWarning(const QString &text, bool isFault)
{
    m_warning->setText(text);
    m_warning->setStyleSheet(QStringLiteral(
        "font-size: 9px; padding: 3px 8px; border-radius: 4px; "
        "background-color: %1; color: %2; font-weight: bold;")
        .arg(isFault ? "#FEF2F2" : "#F3F4F6",
             isFault ? "#DC2626" : "#64748B"));
    m_warning->setVisible(!text.isEmpty());
}
