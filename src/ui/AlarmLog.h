#pragma once
#include <QPushButton>
#include <QTableWidget>
#include <QWidget>

// 可折叠事件日志，浅色表格风格，默认折叠。最大 200 条。
class AlarmLog : public QWidget
{
    Q_OBJECT
public:
    enum Level { Info, Warning, Fault };
    explicit AlarmLog(QWidget *parent = nullptr);

    void addEvent(Level level, const QString &event, const QString &action);
    void clear();
    bool exportCsv(const QString &path);

private slots:
    void toggleCollapse();

private:
    QPushButton  *m_toggle = nullptr;
    QPushButton  *m_export = nullptr;
    QPushButton  *m_clear  = nullptr;
    QTableWidget *m_table  = nullptr;
    QWidget      *m_content = nullptr;
    bool m_collapsed = true;   // 默认折叠
    int  m_count     = 0;
    static constexpr int kMaxRows = 200;
};
