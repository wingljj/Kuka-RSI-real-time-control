#pragma once
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <QWidget>
#include <vector>
#include "net/SharedState.h"

class ErrorChart : public QWidget
{
    Q_OBJECT
public:
    explicit ErrorChart(int windowSeconds, QWidget *parent = nullptr);

    // 从环形缓冲拉取样本重画。由 GUI 线程调用。
    void updateFrom(const SampleRing &ring);

private:
    // 只取绘制真正需要的点数，不要拉满 kCapacity。copyOut 在 ring 互斥内
    // 执行，而 comm 线程每周期的 push() 抢同一把锁；GUI 线程持锁期间若被
    // 抢占，push() 最坏要等一个时间片（~10–15ms），超过 RSI 周期即丢包停机。
    // 1200 点已超过任何常见图表的像素宽度，再多也画不出信息。
    static constexpr int kMaxDrawPoints = 1200;

    int m_windowSeconds = 20;
    QLineSeries *m_posSeries = nullptr;
    QLineSeries *m_rotSeries = nullptr;
    QValueAxis  *m_axisX = nullptr;
    QValueAxis  *m_axisPos = nullptr;
    QValueAxis  *m_axisRot = nullptr;
    QChartView  *m_view = nullptr;

    // 抓取样本的暂存区。做成成员而非函数内 static：static 会被所有
    // ErrorChart 实例共享，而 updateFrom 每秒调用 30 次，容量在构造时
    // 一次分配后就不再进堆。
    std::vector<ChartSample> m_buf;
};
