#include "ui/ErrorChart.h"

#include <QBrush>
#include <QChart>
#include <QPainter>
#include <QSize>
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

    // 两条曲线共用横轴但各有自己的纵轴（左 mm、右 °），量纲不同、量程独立。
    // 若不做颜色关联，操作员看到一段平台期时无法判断该照哪根轴读数——而这
    // 两个数都是他要据以动手的误差量。把轴标签与轴线染成对应曲线的颜色。
    m_axisPos->setLabelsColor(m_posSeries->color());
    m_axisPos->setLinePenColor(m_posSeries->color());
    m_axisPos->setTitleBrush(QBrush(m_posSeries->color()));
    m_axisRot->setLabelsColor(m_rotSeries->color());
    m_axisRot->setLinePenColor(m_rotSeries->color());
    m_axisRot->setTitleBrush(QBrush(m_rotSeries->color()));

    m_view = new QChartView(chart, this);
    m_view->setRenderHint(QPainter::Antialiasing);
    // QChartView 的 minimumSizeHint 很大（QGraphicsView 默认约 250×250 再加
    // 边框），它会顶掉 MainWindow 的 resize(980,620)，在小屏笔电上把窗口撑到
    // 超过显示区。显式解除下限，再给一个够看的自有下限。
    m_view->setMinimumSize(QSize(0, 0));
    m_view->setMinimumHeight(180);

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

    const double tEnd    = m_buf[n - 1].tSec;
    const double tOldest = m_buf[0].tSec;
    // 轴起点取「名义窗口起点」与「实际最老样本」中较晚的那个。
    // 绘制点数上限 kMaxDrawPoints 在高频周期下会比 chartWindowS 先耗尽
    // （1200 点 × 12ms = 14.4s < 20s 窗口），若仍按名义窗口画轴，左侧会留出
    // 一段永久空白，读起来像"前面的数据丢了"。轴不该承诺它没有的数据。
    const double tStart  = std::max(std::max(0.0, tEnd - double(m_windowSeconds)),
                                    tOldest);

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
