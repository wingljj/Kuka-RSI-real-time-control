#include "ui/AlarmLog.h"
#include <QDateTime>
#include <QFile>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QTextStream>
#include <QVBoxLayout>

AlarmLog::AlarmLog(QWidget *parent) : QWidget(parent)
{
    auto *top = new QVBoxLayout(this);
    top->setContentsMargins(0, 0, 0, 0);
    top->setSpacing(0);

    // 标题栏
    auto *bar = new QHBoxLayout;
    bar->setContentsMargins(8, 4, 8, 4);
    m_toggle = new QPushButton("▶ 事件日志", this);
    m_toggle->setFlat(true);
    m_toggle->setStyleSheet(
        "font-weight: bold; font-size: 10px; text-align: left; color: #64748B;");
    connect(m_toggle, &QPushButton::clicked, this, &AlarmLog::toggleCollapse);
    bar->addWidget(m_toggle);
    bar->addStretch();
    m_export = new QPushButton("导出 CSV", this);
    m_export->setStyleSheet("font-size: 9px;");
    connect(m_export, &QPushButton::clicked, this, [this] { exportCsv("alarm_log.csv"); });
    bar->addWidget(m_export);
    m_clear = new QPushButton("清空", this);
    m_clear->setStyleSheet("font-size: 9px;");
    connect(m_clear, &QPushButton::clicked, this, &AlarmLog::clear);
    bar->addWidget(m_clear);
    top->addLayout(bar);

    // 表格内容区
    m_content = new QWidget(this);
    auto *cv = new QVBoxLayout(m_content);
    cv->setContentsMargins(0, 0, 0, 0);

    m_table = new QTableWidget(0, 4, m_content);
    m_table->setHorizontalHeaderLabels({"时间", "级别", "事件", "建议动作"});
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->verticalHeader()->setVisible(false);
    m_table->setShowGrid(false);
    m_table->setAlternatingRowColors(true);
    m_table->setStyleSheet(
        "QTableWidget { border: none; font-size: 9px; } "
        "QTableWidget::item { padding: 2px 6px; } "
        "QTableWidget { alternate-background-color: #F9FAFB; }");
    m_table->horizontalHeader()->setStretchLastSection(true);
    m_table->setColumnWidth(0, 70);
    m_table->setColumnWidth(1, 48);
    m_table->setColumnWidth(2, 200);

    cv->addWidget(m_table);
    top->addWidget(m_content);

    // 默认折叠
    m_content->setVisible(false);
    m_export->setVisible(false);
    m_clear->setVisible(false);
    setMaximumHeight(28);
}

void AlarmLog::addEvent(Level level, const QString &event, const QString &action)
{
    const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
    const char *levelStr = (level == Fault) ? "故障"
                           : (level == Warning) ? "警告" : "信息";
    const char *levelColor = (level == Fault)   ? "#DC2626"
                             : (level == Warning) ? "#D97706"
                                                  : "#16A34A";

    // 环形限制
    if (m_count >= kMaxRows) {
        m_table->removeRow(m_table->rowCount() - 1);
        --m_count;
    }

    m_table->insertRow(0);
    auto *t0 = new QTableWidgetItem(ts);
    auto *t1 = new QTableWidgetItem(levelStr);
    t1->setForeground(QColor(levelColor));
    auto t1f = t1->font(); t1f.setBold(true); t1->setFont(t1f);
    auto *t2 = new QTableWidgetItem(event);
    auto *t3 = new QTableWidgetItem(action);
    t3->setForeground(QColor("#64748B"));
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

void AlarmLog::toggleCollapse()
{
    m_collapsed = !m_collapsed;
    m_content->setVisible(!m_collapsed);
    m_export->setVisible(!m_collapsed);
    m_clear->setVisible(!m_collapsed);
    m_toggle->setText(m_collapsed ? "▶ 事件日志" : "▼ 事件日志");
    setMaximumHeight(m_collapsed ? 28 : 200);
    if (!m_collapsed) setMinimumHeight(140);
    else setMinimumHeight(0);
}
