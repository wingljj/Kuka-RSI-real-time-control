#include "ui/AlarmLog.h"
#include <QDateTime>
#include <QFile>
#include <QFontMetrics>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QTextStream>
#include <QVBoxLayout>
#include <algorithm>
#include "ui/UiLogic.h"

AlarmLog::AlarmLog(QWidget *parent) : QWidget(parent)
{
    auto *top = new QVBoxLayout(this);
    top->setContentsMargins(4, 4, 4, 4);
    top->setSpacing(4);

    // 工具行。标题不再自己画：QDockWidget 的标题栏已经写着「事件日志」，
    // 再加一遍就是两行同样的字。
    auto *bar = new QHBoxLayout;
    bar->setContentsMargins(0, 0, 0, 0);
    bar->addStretch();
    m_export = new QPushButton("导出 CSV", this);
    connect(m_export, &QPushButton::clicked, this, [this] { exportCsv("alarm_log.csv"); });
    bar->addWidget(m_export);
    m_clear = new QPushButton("清空", this);
    connect(m_clear, &QPushButton::clicked, this, &AlarmLog::clear);
    bar->addWidget(m_clear);
    top->addLayout(bar);

    m_table = new QTableWidget(0, 4, this);
    m_table->setHorizontalHeaderLabels({"时间", "级别", "事件", "建议动作"});
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->verticalHeader()->setVisible(false);
    m_table->setShowGrid(false);
    // 隔行底色交给 setAlternatingRowColors + 系统调色板，不写死 #F9FAFB：
    // 写死的浅灰在高对比度深色主题下会把黑字压成不可读。
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setStretchLastSection(true);
    // 前三列按字体度量算宽：70/48/200 是配合样式表里 9px 字号的，回到系统
    // 字号后时间列放不下 "00:00:00"、级别列放不下「故障」，都会被省略号截断。
    const QFontMetrics fm(m_table->font());
    m_table->setColumnWidth(0, fm.horizontalAdvance("00:00:00") + 16);
    m_table->setColumnWidth(1, fm.horizontalAdvance("故障") + 16);
    m_table->setColumnWidth(2, fm.horizontalAdvance("累计修正超限 位置 100% 姿态 100%") + 16);

    top->addWidget(m_table, 1);
}

void AlarmLog::addEvent(Level level, const QString &event, const QString &action)
{
    const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
    const char *levelStr = (level == Fault) ? "故障"
                           : (level == Warning) ? "警告" : "信息";
    // 级别色与状态卡片同一个来源，两处各写一遍色值就会各走各的。
    const uilogic::Severity sev = (level == Fault)     ? uilogic::Severity::Fault
                                  : (level == Warning) ? uilogic::Severity::Warn
                                                       : uilogic::Severity::Ok;

    // 环形限制
    if (m_count >= kMaxRows) {
        m_table->removeRow(m_table->rowCount() - 1);
        --m_count;
    }

    m_table->insertRow(0);
    auto *t0 = new QTableWidgetItem(ts);
    auto *t1 = new QTableWidgetItem(levelStr);
    t1->setForeground(uilogic::severityColor(sev));
    auto t1f = t1->font(); t1f.setBold(true); t1->setFont(t1f);
    auto *t2 = new QTableWidgetItem(event);
    auto *t3 = new QTableWidgetItem(action);
    // 「建议动作」是次要信息，压一档灰；不写死深灰是为了不与深色主题打架。
    t3->setForeground(uilogic::severityColor(uilogic::Severity::Idle));
    m_table->setItem(0, 0, t0);
    m_table->setItem(0, 1, t1);
    m_table->setItem(0, 2, t2);
    m_table->setItem(0, 3, t3);
    ++m_count;
}

void AlarmLog::clear()
{
    m_table->setRowCount(0);
    m_count = 0;
}

bool AlarmLog::exportCsv(const QString &path)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream ts(&f);
    ts << "时间,级别,事件,建议动作\n";
    for (int r = 0; r < m_table->rowCount(); ++r) {
        ts << m_table->item(r,0)->text() << ","
           << m_table->item(r,1)->text() << ","
           << m_table->item(r,2)->text() << ","
           << m_table->item(r,3)->text() << "\n";
    }
    return true;
}
