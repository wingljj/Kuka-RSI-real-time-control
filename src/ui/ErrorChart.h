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
    //
    // 2000 点覆盖 12ms 周期下的完整 20s 窗口（20/0.012 ≈ 1667）。原先取 1200
    // 只够 14.4s，于是 20s 的轴上永远有一段空白。copyOut 已改成两段连续
    // memcpy，2000 × 24B = 48KB 的拷贝仍远快于早先逐元素取模的 4096 次循环。
    // 注意 4ms 周期下 20s 需要 5000 点，超过 SampleRing::kCapacity(4096)，
    // 那时窗口会被环形缓冲本身截短——轴会如实收窄，不会再假装有数据。
    static constexpr int kMaxDrawPoints = 2000;

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
