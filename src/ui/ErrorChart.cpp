#include "ui/ErrorChart.h"

#include <QChart>
#include <QPainter>
#include <QVBoxLayout>
#include <algorithm>

ErrorChart::ErrorChart(int windowSeconds, QWidget *parent)
    : QWidget(parent), m_windowSeconds(std::max(1, windowSeconds)),
      m_buf(kMaxDrawPoints)
{
    auto *chart = new QChart;
    chart->setTitle("跟踪误差");
    chart->legend()->setAlignment(Qt::AlignBottom);

    m_posSeries = new QLineSeries;
    m_posSeries->setName("位置误差 mm");
    m_rotSeries = new QLineSeries;
    m_rotSeries->setName("姿态误差 °");

    chart->addSeries(m_posSeries);
    chart->addSeries(m_rotSeries);

    m_axisX = new QValueAxis;
    m_axisX->setTitleText("时间 s");
    chart->addAxis(m_axisX, Qt::AlignBottom);
    m_posSeries->attachAxis(m_axisX);
    m_rotSeries->attachAxis(m_axisX);

    m_axisPos = new QValueAxis;
    m_axisPos->setTitleText("mm");
    chart->addAxis(m_axisPos, Qt::AlignLeft);
    m_posSeries->attachAxis(m_axisPos);

    m_axisRot = new QValueAxis;
    m_axisRot->setTitleText("°");
    chart->addAxis(m_axisRot, Qt::AlignRight);
    m_rotSeries->attachAxis(m_axisRot);

    m_view = new QChartView(chart, this);
    m_view->setRenderHint(QPainter::Antialiasing);

    auto *lay = new QVBoxLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(m_view);
}

void ErrorChart::updateFrom(const SampleRing &ring)
{
    const int n = ring.copyOut(m_buf.data(), kMaxDrawPoints);
    if (n == 0) {
        m_posSeries->clear();
        m_rotSeries->clear();
        return;
    }

    const double tEnd = m_buf[n - 1].tSec;
    const double tStart = std::max(0.0, tEnd - double(m_windowSeconds));

    QList<QPointF> pos, rot;
    pos.reserve(n);
    rot.reserve(n);
    double posMax = 1.0, rotMax = 1.0;
    for (int i = 0; i < n; ++i) {
        if (m_buf[i].tSec < tStart)
            continue;
        pos.append(QPointF(m_buf[i].tSec, m_buf[i].posErrNorm));
        rot.append(QPointF(m_buf[i].tSec, m_buf[i].rotErrNorm));
        posMax = std::max(posMax, m_buf[i].posErrNorm);
        rotMax = std::max(rotMax, m_buf[i].rotErrNorm);
    }

    m_posSeries->replace(pos);
    m_rotSeries->replace(rot);
    m_axisX->setRange(tStart, std::max(tStart + 1.0, tEnd));
    m_axisPos->setRange(0.0, posMax * 1.2);
    m_axisRot->setRange(0.0, rotMax * 1.2);
}
