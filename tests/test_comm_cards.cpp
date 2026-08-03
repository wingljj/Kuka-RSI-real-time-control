// 「回包 µs」卡片的读数与告警色单测。
//
// 锁住的缺陷：这张卡片的两行原先都打印 s.maxReplyUs——
//   1) 「当前」与「最大」两行永远显示同一个数（实测截图 当前 236 / 最大 236）；
//   2) maxReplyUs 单调，一次瞬时尖峰永久污染「当前」读数；
//   3) 告警判据也用 maxReplyUs，于是一旦超过 1000µs，卡片永远锁在告警色——
//      把一条历史告警当成当前状态呈现，这张卡片因此无法用于观察链路恢复。
// 根因是快照里根本没有瞬时回包耗时这个量，只有会话最大值。
//
// 这些事实只有读回真实 QLabel 的文本与前景色才看得见，所以测真实部件。

#include <QtTest>
#include <QLabel>
#include <QPalette>
#include "ui/CommCards.h"
#include "ui/UiLogic.h"

class TestCommCards : public QObject
{
    Q_OBJECT

private:
    // 一个连着的、跟踪中的会话，只有回包耗时两个字段由各条测试指定。
    static StatusSnapshot connected(double replyUs, double maxReplyUs)
    {
        StatusSnapshot s;
        s.connected  = true;
        s.state      = ControlState::Tracking;
        s.frameCount = 100;
        s.replyUs    = replyUs;
        s.maxReplyUs = maxReplyUs;
        return s;
    }

    static QColor fg(QWidget *w, const char *name)
    {
        auto *l = w->findChild<QLabel *>(name);
        Q_ASSERT(l);
        return l->palette().color(QPalette::WindowText);
    }

    static QString text(QWidget *w, const char *name)
    {
        auto *l = w->findChild<QLabel *>(name);
        Q_ASSERT(l);
        return l->text();
    }

private slots:
    // 两行必须显示两个不同的量。这一条在修复前必然失败：两行都 .arg(maxReplyUs)。
    void replyLinesShowInstantaneousAndMax()
    {
        CommCards c;
        c.updateFrom(connected(120.0, 830.0), 12.0);
        QCOMPARE(text(&c, "replyLine1"), QString("当前 120"));
        QCOMPARE(text(&c, "replyLine2"), QString("最大 830"));
    }

    // 尖峰过后不锁存：会话最大值远超门限，但最近一帧已恢复正常，
    // 卡片必须回到正常色。这正是「用它观察链路是否恢复」所要求的行为。
    void spikeInMaxDoesNotLatchWarn()
    {
        CommCards c;
        // 先来一帧真的超门限，卡片应当告警
        c.updateFrom(connected(4200.0, 4200.0), 12.0);
        QCOMPARE(fg(&c, "replyLine1"), uilogic::severityColor(uilogic::Severity::Warn));

        // 链路恢复：瞬时值回到正常，最大值按定义仍是 4200
        c.updateFrom(connected(95.0, 4200.0), 12.0);
        QCOMPARE(fg(&c, "replyLine1"), uilogic::severityColor(uilogic::Severity::Ok));
        // 历史尖峰不该被抹掉，只是不再上色——它仍是排查时要看的数字
        QCOMPARE(text(&c, "replyLine2"), QString("最大 4200"));
    }

    // 门限判的是瞬时值：当前一帧超门限就必须告警，哪怕会话最大值与它相等。
    void currentAboveThresholdWarns()
    {
        CommCards c;
        c.updateFrom(connected(1001.0, 1001.0), 12.0);
        QCOMPARE(fg(&c, "replyLine1"), uilogic::severityColor(uilogic::Severity::Warn));
    }

    // 未连接一律 Idle：断开后残留的耗时是上一次会话的数字，不构成当前告警。
    void disconnectedIsIdleRegardlessOfSpike()
    {
        CommCards c;
        StatusSnapshot s = connected(4200.0, 4200.0);
        s.connected = false;
        s.state     = ControlState::Disconnected;
        c.updateFrom(s, 12.0);
        QCOMPARE(fg(&c, "replyLine1"), uilogic::severityColor(uilogic::Severity::Idle));
    }
};

QTEST_MAIN(TestCommCards)
#include "test_comm_cards.moc"
