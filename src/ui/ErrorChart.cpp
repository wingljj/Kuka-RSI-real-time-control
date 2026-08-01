#include "ui/ErrorChart.h"

#include <QChart>
#include <QGridLayout>
#include <QPainter>
#include <QSize>
#include <algorithm>

ErrorChart::ErrorChart(int windowSeconds, Mode mode, QWidget *parent)
    : QWidget(parent), m_windowSeconds(std::max(1, windowSeconds)),
      m_mode(mode), m_buf(kMaxDrawPoints)
{
    const bool isPos = m_mode == Mode::Position;

    auto *chart = new QChart;
    chart->setTitle(isPos ? "位置误差 mm" : "姿态误差 °");
    chart->legend()->setAlignment(Qt::AlignBottom);

    // 单系列单 Y 轴：位置图 mm、姿态图 °，量纲不同的两套数据各占一图。
    m_series = new QLineSeries;
    m_series->setName(isPos ? "位置误差 mm" : "姿态误差 °");
    chart->addSeries(m_series);

    m_axisX = new QValueAxis;
    m_axisX->setTitleText("时间 s");
    chart->addAxis(m_axisX, Qt::AlignBottom);
    m_series->attachAxis(m_axisX);

    m_axisY = new QValueAxis;
    m_axisY->setTitleText(isPos ? "mm" : "°");
    chart->addAxis(m_axisY, Qt::AlignLeft);
    m_series->attachAxis(m_axisY);

    m_view = new QChartView(chart, this);
    m_view->setRenderHint(QPainter::Antialiasing);
    // QChartView 的 minimumSizeHint 很大（QGraphicsView 默认约 250×250 再加
    // 边框），它会顶掉 MainWindow 的 resize(980,620)，在小屏笔电上把窗口撑到
    // 超过显示区。显式解除下限，再给一个够看的自有下限。
    m_view->setMinimumSize(QSize(0, 0));
    m_view->setMinimumHeight(150);

    // 空态占位：与 m_view 叠在同一个 grid 单元格里。无数据时显示提示、
    // 隐藏视图；有数据反之（见 updateFrom）。
    m_placeholder = new QLabel(
        isPos ? "等待 RSI 数据…\n请启动 KRL PoseTrack 程序"
              : "等待姿态误差数据…", this);
    m_placeholder->setAlignment(Qt::AlignCenter);
    m_placeholder->setStyleSheet("color: #888; font-size: 14px;");

    auto *lay = new QGridLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(m_view, 0, 0);
    lay->addWidget(m_placeholder, 0, 0);
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

    const double tEnd    = m_buf[n - 1].tSec;
    const double tOldest = m_buf[0].tSec;
    // 轴起点取「名义窗口起点」与「实际最老样本」中较晚的那个。
    // 绘制点数上限 kMaxDrawPoints 在高频周期下会比 chartWindowS 先耗尽，
    // 若仍按名义窗口画轴，左侧会留出一段永久空白，读起来像"前面的数据丢
    // 了"。轴不该承诺它没有的数据。
    const double tStart  = std::max(std::max(0.0, tEnd - double(m_windowSeconds)),
                                    tOldest);

    QList<QPointF> pts;
    pts.reserve(n);
    double yMax = 1.0;
    for (int i = 0; i < n; ++i) {
        if (m_buf[i].tSec < tStart)
            continue;
        const double y = (m_mode == Mode::Position) ? m_buf[i].posErrNorm
                                                    : m_buf[i].rotErrNorm;
        pts.append(QPointF(m_buf[i].tSec, y));
        yMax = std::max(yMax, y);
    }

    m_series->replace(pts);
    m_axisX->setRange(tStart, std::max(tStart + 1.0, tEnd));
    m_axisY->setRange(0.0, yMax * 1.2);
    m_placeholder->setVisible(n == 0);
    m_view->setVisible(n > 0);
}
