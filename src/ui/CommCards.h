#pragma once
#include <QLabel>
#include <QWidget>
#include "net/SharedState.h"

// 通信质量四合一卡片：周期 / 回包耗时 / 丢包 / IPOC。
// 每张卡片一个背景色块，阈值超限自动变色。
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
        void setColors(const char *bg, const char *fg);
    };
    Card m_cycle;
    Card m_reply;
    Card m_loss;
    Card m_ipoc;
};
