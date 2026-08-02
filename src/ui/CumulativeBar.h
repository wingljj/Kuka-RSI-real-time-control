#pragma once
#include <QLabel>
#include <QProgressBar>
#include <QWidget>
#include <array>
#include "net/SharedState.h"

// 累积修正进度条：6 轴各一行，标签 + 进度条 + 数值 + 状态指示。
// 绿色(<50%) / 琥珀(50–80%) / 深琥珀(80–100%) / 红色(>100%)。
class CumulativeBar : public QWidget
{
    Q_OBJECT
public:
    explicit CumulativeBar(QWidget *parent = nullptr);

    // 更新累积值与限值
    void updateFrom(const Pose &accum, double limitPosMm, double limitRotDeg);

private:
    struct AxisRow {
        QLabel *label      = nullptr;   // "X"
        QProgressBar *bar  = nullptr;
        QLabel *valueLabel = nullptr;   // "-788.9 mm"
        QLabel *status     = nullptr;   // "超限" / "正常"
    };
    std::array<AxisRow, 6> m_rows;
    QLabel *m_title = nullptr;
};
