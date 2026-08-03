#pragma once
#include <QPushButton>
#include <QTableWidget>
#include <QWidget>

// 可折叠事件日志，原生表格外观，默认折叠。最大 200 条。
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
    // 折叠后只留标题栏。高度按标题栏实测算而不是写死 28：那个常数是配合
    // 样式表里 9-10px 字号定的，回到系统字号后按钮会被压掉一截。
    int  m_collapsedHeight = 28;
    int  m_count     = 0;
    static constexpr int kMaxRows = 200;
};
