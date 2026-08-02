#include "ui/CumulativeBar.h"
#include <QGridLayout>
#include <QHBoxLayout>

CumulativeBar::CumulativeBar(QWidget *parent) : QWidget(parent)
{
    auto *v = new QVBoxLayout(this);
    v->setContentsMargins(4, 4, 4, 4);
    v->setSpacing(2);

    m_title = new QLabel("累积修正", this);
    m_title->setStyleSheet("font-weight: bold; font-size: 10px;");
    v->addWidget(m_title);

    auto *grid = new QGridLayout;
    grid->setSpacing(2);
    const char *names[6] = {"X", "Y", "Z", "A", "B", "C"};
    for (int i = 0; i < 6; ++i) {
        auto &r = m_rows[i];
        r.label = new QLabel(names[i], this);
        r.label->setFixedWidth(16);
        r.label->setStyleSheet("font-size: 9px; font-weight: bold;");
        grid->addWidget(r.label, i, 0);

        r.bar = new QProgressBar(this);
        r.bar->setRange(0, 100);
        r.bar->setValue(0);
        r.bar->setTextVisible(false);
        r.bar->setFixedHeight(12);
        grid->addWidget(r.bar, i, 1);

        r.valueLabel = new QLabel("--", this);
        r.valueLabel->setStyleSheet("font-size: 8px; font-family: Consolas, monospace;");
        r.valueLabel->setFixedWidth(90);
        r.valueLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        grid->addWidget(r.valueLabel, i, 2);

        r.status = new QLabel("", this);
        r.status->setFixedWidth(32);
        r.status->setStyleSheet("font-size: 8px; font-weight: bold;");
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

        const char *barColor;
        const char *statusText;
        if (pct >= 100.0) {
            barColor = "#dc3545";  // 红
            statusText = "超限";
        } else if (pct >= 80.0) {
            barColor = "#e07000";  // 深琥珀
            statusText = "警告";
        } else if (pct >= 50.0) {
            barColor = "#ffc107";  // 琥珀
            statusText = "注意";
        } else {
            barColor = "#28a745";  // 绿
            statusText = "正常";
        }
        r.bar->setStyleSheet(QStringLiteral(
            "QProgressBar { border: 1px solid #ccc; border-radius: 2px; "
            "background: #eee; } "
            "QProgressBar::chunk { background-color: %1; border-radius: 2px; }")
            .arg(QLatin1String(barColor)));
        r.status->setText(statusText);
        r.status->setStyleSheet(QStringLiteral(
            "font-size: 8px; font-weight: bold; color: %1;")
            .arg(QLatin1String(barColor)));
    }
}
