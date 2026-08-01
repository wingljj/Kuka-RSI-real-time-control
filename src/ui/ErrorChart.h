#pragma once
#include <QLabel>
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
    enum class Mode { Position, Rotation };
    explicit ErrorChart(int windowSeconds, Mode mode, QWidget *parent = nullptr);

    // 从环形缓冲拉取样本重画。由 GUI 线程调用。
    void updateFrom(const SampleRing &ring);

private:
    // 只取绘制真正需要的点数，不要拉满 kCapacity。copyOut 在 ring 互斥内
    // 执行，而 comm 线程每周期的 push() 抢同一把锁；GUI 线程持锁期间若被
    // 抢占，push() 最坏要等一个时间片（~10–15ms），超过 RSI 周期即丢包停机。
    //
    // 2000 点覆盖 12ms 周期下的完整 10s 窗口（10/0.012 ≈ 833）。4ms 周期下
    // 10s 需 2500 点，超过 kMaxDrawPoints 时窗口被截短至 ~8s——轴会如实收窄
    // （tStart = max(tEnd - window, tOldest)），不再假装有数据。
    static constexpr int kMaxDrawPoints = 2000;

    int m_windowSeconds = 10;
    Mode m_mode = Mode::Position;
    QLineSeries *m_series = nullptr;
    QValueAxis  *m_axisX = nullptr;
    QValueAxis  *m_axisY = nullptr;
    QChartView  *m_view = nullptr;
    QLabel      *m_placeholder = nullptr;   // 空态占位，与 m_view 叠放

    // 抓取样本的暂存区。做成成员而非函数内 static：static 会被所有
    // ErrorChart 实例共享，而 updateFrom 每秒调用 30 次，容量在构造时
    // 一次分配后就不再进堆。
    std::vector<ChartSample> m_buf;
};
