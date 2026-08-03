#include "ui/CumulativeBar.h"
#include <QFontMetrics>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QPalette>
#include <algorithm>
#include "ui/UiLogic.h"

CumulativeBar::CumulativeBar(QWidget *parent) : QWidget(parent)
{
    auto *v = new QVBoxLayout(this);
    v->setContentsMargins(4, 4, 4, 4);
    v->setSpacing(2);

    // 标题不再自己画：这个部件装在标题为「累积修正」的 QDockWidget 里，
    // 再写一遍就是紧挨着的两行同样的字。
    auto *grid = new QGridLayout;
    grid->setSpacing(2);
    const char *names[6] = {"X", "Y", "Z", "A", "B", "C"};
    // 数值列与状态列的宽度按字体度量算，不写死 90/32：那两个数字是配合
    // 样式表里 8px 字号定的，回到系统字号后 "-1000.0 mm" 与「超限」都装不下，
    // 会被省略号截断——而这两列存在的理由就是把量值和结论看全。
    const QFontMetrics fm(font());
    const int wValue = fm.horizontalAdvance("-1000.0 mm") + 8;
    int wStatus = 0;
    for (const char *s : {"正常", "注意", "警告", "超限"})
        wStatus = std::max(wStatus, fm.horizontalAdvance(QString::fromUtf8(s)));
    wStatus += 8;

    for (int i = 0; i < 6; ++i) {
        auto &r = m_rows[i];
        r.label = new QLabel(names[i], this);
        r.label->setFixedWidth(fm.horizontalAdvance("W") + 6);
        QFont axisF = r.label->font();
        axisF.setBold(true);
        r.label->setFont(axisF);
        grid->addWidget(r.label, i, 0);

        // 进度条保持原生外观。QProgressBar::chunk 的颜色只有样式表能可靠地
        // 改（原生样式画的是主题位图，QPalette::Highlight 在 Windows 主题下
        // 根本不参与），所以这里不再试图给条本身上色——语义交给右侧的
        // 状态文字 + 文字色，与「颜色永远伴随文字」的既定原则一致。
        r.bar = new QProgressBar(this);
        r.bar->setRange(0, 100);
        r.bar->setValue(0);
        r.bar->setTextVisible(false);
        // 六行必须与上方的位姿表同屏，所以高度仍压到 14px：原生主题的条是
        // 可缩放的，压扁不会画糊。
        r.bar->setFixedHeight(14);
        grid->addWidget(r.bar, i, 1);

        r.valueLabel = new QLabel("--", this);
        r.valueLabel->setFont(uilogic::monospaceFont());
        r.valueLabel->setFixedWidth(wValue);
        r.valueLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        grid->addWidget(r.valueLabel, i, 2);

        r.status = new QLabel("", this);
        r.status->setFixedWidth(wStatus);
        QFont statusF = r.status->font();
        statusF.setBold(true);
        r.status->setFont(statusF);
        r.status->setAlignment(Qt::AlignCenter);
        grid->addWidget(r.status, i, 3);
    }
    v->addLayout(grid);
}

void CumulativeBar::updateFrom(const Pose &accum, double limitPosMm, double limitRotDeg)
{
    const double vals[6] = {accum.x, accum.y, accum.z, accum.a, accum.b, accum.c};
    const char *units[6] = {"mm", "mm", "mm", "°", "°", "°"};
    const double limits[6] = {limitPosMm, limitPosMm, limitPosMm,
                              limitRotDeg, limitRotDeg, limitRotDeg};
    for (int i = 0; i < 6; ++i) {
        auto &r = m_rows[i];
        const double pct = (limits[i] > 0.0)
                               ? (std::fabs(vals[i]) / limits[i]) * 100.0
                               : 0.0;
        r.bar->setValue(int(std::min(pct, 100.0)));
        r.valueLabel->setText(QStringLiteral("%1 %2")
                                  .arg(vals[i], 0, 'f', 1).arg(units[i]));

        // 「注意」与「警告」原先是两种琥珀（#ffc107 / #e07000）。同一档语义
        // 用两个色值，操作员分不出，代码里却要维护两处——档位差别由文字说。
        uilogic::Severity sev;
        const char *statusText;
        if (pct >= 100.0) {
            sev = uilogic::Severity::Fault;
            statusText = "超限";
        } else if (pct >= 80.0) {
            sev = uilogic::Severity::Warn;
            statusText = "警告";
        } else if (pct >= 50.0) {
            sev = uilogic::Severity::Warn;
            statusText = "注意";
        } else {
            sev = uilogic::Severity::Ok;
            statusText = "正常";
        }
        r.status->setText(QString::fromUtf8(statusText));
        const QColor fg = uilogic::severityColor(sev);
        if (r.status->palette().color(QPalette::WindowText) != fg) {
            QPalette p = r.status->palette();
            p.setColor(QPalette::WindowText, fg);
            r.status->setPalette(p);
        }
    }
}
