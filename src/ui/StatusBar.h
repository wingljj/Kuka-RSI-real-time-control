#pragma once
#include <QLabel>
#include <QFrame>
#include <QWidget>
#include "net/SharedState.h"
#include "ui/UiLogic.h"

// 顶部状态区：4 个状态卡片（网络/RSI/控制/质量）+ 警告条。
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
        // 卡片标题在构造时就定了，刷新时不会变，所以不再当参数传：
        // 旧签名收下 labelText 却在函数体里原样丢掉（"title unchanged"），
        // 四个调用点每次都重复写一遍标题，看起来像是它能改。
        void set(const QString &iconText, const QString &statusText,
                 uilogic::Severity sev);
    };
    Card m_netCard, m_rsiCard, m_ctlCard, m_qualCard;

    QLabel *m_warning = nullptr;
};
