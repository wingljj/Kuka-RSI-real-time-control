#include "ui/ForceChart.h"

#include <QChart>
#include <QGridLayout>
#include <QPainter>
#include <QPalette>
#include <algorithm>
#include "ui/UiLogic.h"

ForceChart::ForceChart(int windowSeconds, Mode mode, QWidget *parent)
    : QWidget(parent), m_windowSeconds(std::max(1, windowSeconds)),
      m_mode(mode), m_buf(kMaxDrawPoints)
{
    const bool isForce = m_mode == Mode::Force;
    const QString unit = isForce ? QStringLiteral("N") : QStringLiteral("Nm");

    auto *chart = new QChart;
    chart->setTitle(isForce ? QStringLiteral("力曲线 Fx/Fy/Fz (N)")
                            : QStringLiteral("力矩曲线 Mx/My/Mz (Nm)"));
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->setMargins(QMargins(0, 2, 0, 2));

    // 三通道主系列。先 addSeries 再取 color()：主题色按 addSeries 的顺序
    // 分配（蓝/绿/橙），提前取会让三条线拿到同一个默认色。
    m_seriesX = new QLineSeries;
    m_seriesX->setName(isForce ? QStringLiteral("Fx") : QStringLiteral("Mx"));
    m_seriesY = new QLineSeries;
    m_seriesY->setName(isForce ? QStringLiteral("Fy") : QStringLiteral("My"));
    m_seriesZ = new QLineSeries;
    m_seriesZ->setName(isForce ? QStringLiteral("Fz") : QStringLiteral("Mz"));
    chart->addSeries(m_seriesX);
    chart->addSeries(m_seriesY);
    chart->addSeries(m_seriesZ);
    m_seriesX->setPen(QPen(m_seriesX->color(), 2.0));
    m_seriesY->setPen(QPen(m_seriesY->color(), 2.0));
    m_seriesZ->setPen(QPen(m_seriesZ->color(), 2.0));

    // 零线（力/力矩都以零为基准读数，符号有语义）
    m_zeroLine = new QLineSeries;
    m_zeroLine->setName(QStringLiteral("零线"));
    m_zeroLine->setPen(QPen(QColor(128, 128, 128), 1.0, Qt::DashLine));
    chart->addSeries(m_zeroLine);

    // X 轴
    m_axisX = new QValueAxis;
    m_axisX->setTitleText(QStringLiteral("时间 / s"));
    m_axisX->setLabelFormat("%.0f");
    m_axisX->setTickCount(6);
    chart->addAxis(m_axisX, Qt::AlignBottom);
    m_seriesX->attachAxis(m_axisX);
    m_seriesY->attachAxis(m_axisX);
    m_seriesZ->attachAxis(m_axisX);
    m_zeroLine->attachAxis(m_axisX);

    // Y 轴
    m_axisY = new QValueAxis;
    m_axisY->setTitleText(unit);
    m_axisY->setLabelFormat("%.1f");
    chart->addAxis(m_axisY, Qt::AlignLeft);
    m_seriesX->attachAxis(m_axisY);
    m_seriesY->attachAxis(m_axisY);
    m_seriesZ->attachAxis(m_axisY);
    m_zeroLine->attachAxis(m_axisY);

    m_view = new QChartView(chart, this);
    m_view->setRenderHint(QPainter::Antialiasing);
    m_view->setMinimumSize(QSize(0, 0));
    m_view->setMinimumHeight(150);

    m_placeholder = new QLabel(
        isForce ? QStringLiteral("等待力传感器数据…\n请启动 SRI 数据流")
                : QStringLiteral("等待力矩数据…\n请启动 SRI 数据流"), this);
    m_placeholder->setAlignment(Qt::AlignCenter);
    // 「无数据」正是 Severity::Idle 那一档，色值走同一个函数而不是再写一个灰。
    // 字号在系统字号上加两号：这段字要盖住整张空图，与图内的轴标签同号会
    // 看起来像图的一部分。
    QPalette phPal = m_placeholder->palette();
    phPal.setColor(QPalette::WindowText, uilogic::severityColor(uilogic::Severity::Idle));
    m_placeholder->setPalette(phPal);
    QFont phFont = m_placeholder->font();
    phFont.setPointSize(phFont.pointSize() + 2);
    m_placeholder->setFont(phFont);

    // placeholder 与图表叠在同一格：无数据时盖住空图，有数据时让位给曲线。
    auto *lay = new QGridLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(0);
    lay->addWidget(m_view, 0, 0);
    lay->addWidget(m_placeholder, 0, 0);
}

void ForceChart::setShowFiltered(bool filtered)
{
    m_showFiltered = filtered;
}

void ForceChart::updateFrom(const ForceRing &ring)
{
    const int n = ring.copyOut(m_buf.data(), kMaxDrawPoints);
    if (n == 0) {
        m_seriesX->clear();
        m_seriesY->clear();
        m_seriesZ->clear();
        m_placeholder->setVisible(true);
        m_view->setVisible(false);
        return;
    }

    const double tEnd = m_buf[n - 1].tSec;
    const double tOldest = m_buf[0].tSec;
    const double tStart  = std::max(std::max(0.0, tEnd - double(m_windowSeconds)),
                                    tOldest);

    QList<QPointF> px, py, pz;
    px.reserve(n);
    py.reserve(n);
    pz.reserve(n);
    // 力/力矩可正可负，Y 轴范围取数据包络并让零留在视野内。
    double yMin = 0.0, yMax = 1.0;
    for (int i = 0; i < n; ++i) {
        if (m_buf[i].tSec < tStart) continue;
        const ForceChartSample &s = m_buf[i];
        double x, y, z;
        if (m_mode == Mode::Force) {
            if (m_showFiltered) { x = s.ffx; y = s.ffy; z = s.ffz; }
            else                { x = s.fx;  y = s.fy;  z = s.fz; }
        } else {
            if (m_showFiltered) { x = s.fmx; y = s.fmy; z = s.fmz; }
            else                { x = s.mx;  y = s.my;  z = s.mz; }
        }
        px.append(QPointF(s.tSec, x));
        py.append(QPointF(s.tSec, y));
        pz.append(QPointF(s.tSec, z));
        yMin = std::min(yMin, std::min(x, std::min(y, z)));
        yMax = std::max(yMax, std::max(x, std::max(y, z)));
    }

    m_seriesX->replace(px);
    m_seriesY->replace(py);
    m_seriesZ->replace(pz);

    // 时间轴：确保显示具体 tick 数值
    m_axisX->setRange(tStart, std::max(tStart + 1.0, tEnd));
    m_axisX->setTickCount(std::min(11, int(tEnd - tStart) + 2));

    // Y 轴
    const double pad = (yMax - yMin) * 0.15 + 0.01;
    m_axisY->setRange(yMin - pad, yMax + pad);

    // 零线
    m_zeroLine->replace(QList<QPointF>{{tStart, 0.0}, {std::max(tStart+1.0, tEnd), 0.0}});

    m_placeholder->setVisible(false);
    m_view->setVisible(true);
}
