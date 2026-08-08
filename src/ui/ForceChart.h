#pragma once
#include <QLabel>
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <QWidget>
#include <vector>
#include "net/SharedState.h"

// 力控曲线：三通道（Fx/Fy/Fz 或 Mx/My/Mz），30s 滚动窗口。
// 与 ErrorChart 同构：QChartView 叠 placeholder，数据来自定容环形缓冲。
class ForceChart : public QWidget
{
    Q_OBJECT
public:
    enum class Mode { Force, Torque };
    explicit ForceChart(int windowSeconds, Mode mode, QWidget *parent = nullptr);

    // 从力控环形缓冲读取最近样本并重绘（无数据时显示 placeholder）
    void updateFrom(const ForceRing &ring);

    // 切换原始/滤波视图（默认滤波）。生效于下一次 updateFrom。
    void setShowFiltered(bool filtered);

private:
    static constexpr int kMaxDrawPoints = 3000;

    int m_windowSeconds = 10;
    Mode m_mode = Mode::Force;
    bool m_showFiltered = true;
    QLineSeries *m_seriesX = nullptr;   // Fx 或 Mx
    QLineSeries *m_seriesY = nullptr;   // Fy 或 My
    QLineSeries *m_seriesZ = nullptr;   // Fz 或 Mz
    QLineSeries *m_zeroLine = nullptr;  // 零线
    QValueAxis  *m_axisX = nullptr;
    QValueAxis  *m_axisY = nullptr;
    QChartView  *m_view  = nullptr;
    QLabel      *m_placeholder = nullptr;

    std::vector<ForceChartSample> m_buf;
};
