#include "ui/ErrorChart.h"

#include <QChart>
#include <QDateTime>
#include <QFile>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QPainter>
#include <QTextStream>
#include <algorithm>

ErrorChart::ErrorChart(int windowSeconds, Mode mode, QWidget *parent)
    : QWidget(parent), m_windowSeconds(std::max(1, windowSeconds)),
      m_mode(mode), m_buf(kMaxDrawPoints)
{
    const bool isPos = m_mode == Mode::Position;

    auto *chart = new QChart;
    chart->setTitle(isPos ? "位置误差（XYZ 合成）mm" : "姿态误差（SO(3) 旋转）°");
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->setMargins(QMargins(0, 2, 0, 2));

    // 主系列
    m_series = new QLineSeries;
    m_series->setName(isPos ? "位置误差" : "姿态误差");
    QPen pen(m_series->color(), 2.0);
    m_series->setPen(pen);
    chart->addSeries(m_series);

    // 零线
    m_zeroLine = new QLineSeries;
    m_zeroLine->setName("零线");
    m_zeroLine->setPen(QPen(QColor(128, 128, 128), 1.0, Qt::DashLine));
    chart->addSeries(m_zeroLine);

    // 阈值线
    m_threshold = new QLineSeries;
    m_threshold->setName("阈值");
    m_threshold->setPen(QPen(QColor(220, 50, 50), 1.5, Qt::DashLine));
    m_threshold->setVisible(false);
    chart->addSeries(m_threshold);

    // X 轴
    m_axisX = new QValueAxis;
    m_axisX->setTitleText("时间 / s");
    m_axisX->setLabelFormat("%.0f");
    m_axisX->setTickCount(6);
    chart->addAxis(m_axisX, Qt::AlignBottom);
    m_series  ->attachAxis(m_axisX);
    m_zeroLine->attachAxis(m_axisX);
    m_threshold->attachAxis(m_axisX);

    // Y 轴
    m_axisY = new QValueAxis;
    m_axisY->setTitleText(isPos ? "mm" : "°");
    m_axisY->setLabelFormat("%.1f");
    chart->addAxis(m_axisY, Qt::AlignLeft);
    m_series  ->attachAxis(m_axisY);
    m_zeroLine->attachAxis(m_axisY);
    m_threshold->attachAxis(m_axisY);

    m_view = new QChartView(chart, this);
    m_view->setRenderHint(QPainter::Antialiasing);
    m_view->setMinimumSize(QSize(0, 0));
    m_view->setMinimumHeight(150);

    m_placeholder = new QLabel(
        isPos ? "等待 RSI 数据…\n请启动 KRL PoseTrack 程序"
              : "等待姿态误差数据…", this);
    m_placeholder->setAlignment(Qt::AlignCenter);
    m_placeholder->setStyleSheet("color: #888; font-size: 14px;");

    // 按钮栏
    m_pauseBtn = new QPushButton("⏸ 暂停", this);
    m_pauseBtn->setMaximumWidth(60);
    m_pauseBtn->setStyleSheet("font-size: 9px;");
    connect(m_pauseBtn, &QPushButton::clicked, this, &ErrorChart::onPauseToggled);

    m_clearBtn = new QPushButton("清空", this);
    m_clearBtn->setMaximumWidth(50);
    m_clearBtn->setStyleSheet("font-size: 9px;");
    connect(m_clearBtn, &QPushButton::clicked, this, &ErrorChart::onClear);

    m_exportBtn = new QPushButton("CSV", this);
    m_exportBtn->setMaximumWidth(50);
    m_exportBtn->setStyleSheet("font-size: 9px;");
    connect(m_exportBtn, &QPushButton::clicked, this, &ErrorChart::onExport);

    auto *btnRow = new QHBoxLayout;
    btnRow->setContentsMargins(0, 0, 0, 0);
    btnRow->addWidget(m_pauseBtn);
    btnRow->addWidget(m_clearBtn);
    btnRow->addWidget(m_exportBtn);
    btnRow->addStretch();

    auto *lay = new QGridLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(0);
    lay->addLayout(btnRow, 0, 0);
    lay->addWidget(m_view, 1, 0);
    lay->addWidget(m_placeholder, 1, 0);
}

void ErrorChart::setThresholdLine(double value)
{
    m_thresholdVal = value;
}

void ErrorChart::setSubMode(int subMode)
{
    // 子模式仅影响标题；实际数据仍为合成误差（需要 per-axis ring 才能拆分）
    const bool isPos = m_mode == Mode::Position;
    const char *labels[4] = {"合成", "X", "Y", "Z"};
    auto *chart = m_view->chart();
    if (isPos)
        chart->setTitle(QStringLiteral("位置误差 (%1) mm").arg(labels[subMode & 3]));
    else
        chart->setTitle(QStringLiteral("姿态误差 (%1) °").arg(labels[subMode & 3]));
}

void ErrorChart::onPauseToggled()
{
    m_paused = !m_paused;
    m_pauseBtn->setText(m_paused ? "▶ 继续" : "⏸ 暂停");
}

void ErrorChart::onClear()
{
    m_series->clear();
    m_threshold->clear();
    m_zeroLine->clear();
}

void ErrorChart::onExport()
{
    const QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString name = (m_mode == Mode::Position)
                             ? QStringLiteral("pos_error_%1.csv").arg(ts)
                             : QStringLiteral("rot_error_%1.csv").arg(ts);
    QFile f(name);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) return;
    QTextStream out(&f);
    out << "t_sec,error\n";
    const auto pts = m_series->points();
    for (const auto &p : pts)
        out << p.x() << "," << p.y() << "\n";
}

void ErrorChart::updateFrom(const SampleRing &ring)
{
    const int n = ring.copyOut(m_buf.data(), kMaxDrawPoints);
    if (n == 0) {
        m_series->clear();
        m_placeholder->setVisible(true);
        m_view->setVisible(false);
        return;
    }

    const double tEnd = m_buf[n - 1].tSec;
    if (m_paused) {
        // 暂停时不更新数据
        m_placeholder->setVisible(true);
        m_view->setVisible(true);
        return;
    }

    const double tOldest = m_buf[0].tSec;
    const double tStart  = std::max(std::max(0.0, tEnd - double(m_windowSeconds)),
                                    tOldest);

    QList<QPointF> pts;
    pts.reserve(n);
    double yMax = 1.0;
    for (int i = 0; i < n; ++i) {
        if (m_buf[i].tSec < tStart) continue;
        const double y = (m_mode == Mode::Position) ? m_buf[i].posErrNorm
                                                    : m_buf[i].rotErrNorm;
        pts.append(QPointF(m_buf[i].tSec, y));
        yMax = std::max(yMax, y);
    }

    m_series->replace(pts);

    // 时间轴：确保显示具体 tick 数值
    m_axisX->setRange(tStart, std::max(tStart + 1.0, tEnd));
    m_axisX->setTickCount(std::min(11, int(tEnd - tStart) + 2));

    // Y 轴
    const double yTop = yMax * 1.2;
    m_axisY->setRange(0.0, yTop);

    // 零线
    m_zeroLine->replace(QList<QPointF>{{tStart, 0.0}, {std::max(tStart+1.0, tEnd), 0.0}});

    // 阈值线
    if (m_thresholdVal > 0.0 && m_thresholdVal < yTop * 2.0) {
        m_threshold->replace(QList<QPointF>{{tStart, m_thresholdVal},
                                            {std::max(tStart+1.0, tEnd), m_thresholdVal}});
        m_threshold->setVisible(true);
    } else {
        m_threshold->setVisible(false);
    }

    m_placeholder->setVisible(false);
    m_view->setVisible(true);
}
