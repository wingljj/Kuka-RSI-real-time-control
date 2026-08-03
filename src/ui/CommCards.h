#pragma once
#include <QLabel>
#include <QWidget>
#include "net/SharedState.h"
#include "ui/UiLogic.h"

// 通信质量四合一卡片：周期 / 回包耗时 / 丢包 / IPOC。
// 每张卡片一个原生边框，阈值超限时读数变色（色块背景会盖掉系统主题）。
class CommCards : public QWidget
{
    Q_OBJECT
public:
    explicit CommCards(QWidget *parent = nullptr);
    void updateFrom(const StatusSnapshot &s, double configuredCycleMs);

private:
    struct Card {
        QLabel *title = nullptr;
        QLabel *line1 = nullptr;
        QLabel *line2 = nullptr;
        // 语义等级施加到两行读数的文字色上。原先是给卡片 QFrame 套背景 +
        // 边框的样式表，那条选择器会级联到卡片内的三个 QLabel。
        void setSeverity(uilogic::Severity sev);
    };
    Card m_cycle;
    Card m_reply;
    Card m_loss;
    Card m_ipoc;
};
