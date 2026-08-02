#pragma once
#include <QLabel>
#include <QFrame>
#include <QWidget>
#include "net/SharedState.h"

// 顶部状态区：上排 4 个状态卡片（网络/RSI/控制/质量），下排流程步骤条。
class StatusBar : public QWidget
{
    Q_OBJECT
public:
    explicit StatusBar(QWidget *parent = nullptr);
    void updateFrom(const StatusSnapshot &s, bool listening);
    void setWarning(const QString &text, bool isFault);

private:
    // 状态卡片
    struct Card {
        QFrame *frame = nullptr;
        QLabel *icon  = nullptr;
        QLabel *label = nullptr;
        QLabel *status = nullptr;
        void set(const QString &iconText, const QString &labelText,
                 const QString &statusText, const QString &bgColor,
                 const QString &fgColor);
    };
    Card m_netCard, m_rsiCard, m_ctlCard, m_qualCard;

    // 流程步骤
    struct Step { QLabel *icon = nullptr; QLabel *label = nullptr; };
    std::array<Step, 5> m_steps;
    std::array<QLabel*, 4> m_arrows; // 箭头标签

    void setStepActive(int idx, bool active, bool enabled);
    QLabel *m_warning = nullptr;
};
