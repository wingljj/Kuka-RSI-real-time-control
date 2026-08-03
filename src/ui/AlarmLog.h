#pragma once
#include <QPushButton>
#include <QTableWidget>
#include <QWidget>

// 事件日志，原生表格外观。最大 200 条。
//
// 自带的折叠机制已删除：这个部件现在装在 QDockWidget 里，显示与隐藏由
// 面板标题栏的关闭按钮和「视图」菜单负责。两套开关并存的后果不只是重复——
// 折叠态会 setMaximumHeight(28)，于是从「视图」菜单调出面板时，面板是开的
// 而内容仍是折的，操作员看到一条空白横条，无从知道该点哪里。
class AlarmLog : public QWidget
{
    Q_OBJECT
public:
    enum Level { Info, Warning, Fault };
    explicit AlarmLog(QWidget *parent = nullptr);

    void addEvent(Level level, const QString &event, const QString &action);
    void clear();
    bool exportCsv(const QString &path);

private:
    QPushButton  *m_export = nullptr;
    QPushButton  *m_clear  = nullptr;
    QTableWidget *m_table  = nullptr;
    int  m_count     = 0;
    static constexpr int kMaxRows = 200;
};
