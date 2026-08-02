#pragma once
#include <QLabel>
#include <QChartView>
#include <QLineSeries>
#include <QPushButton>
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

    void updateFrom(const SampleRing &ring);

    // 阈值线（水平虚线，单位 mm 或 °）
    void setThresholdLine(double value);
    // 模式切换（位置图：0=合成 1=X 2=Y 3=Z；姿态图：0=合成 1=A 2=B 3=C）
    void setSubMode(int subMode);

private slots:
    void onPauseToggled();
    void onClear();
    void onExport();

private:
    static constexpr int kMaxDrawPoints = 2000;

    int m_windowSeconds = 10;
    Mode m_mode = Mode::Position;
    QLineSeries *m_series   = nullptr;
    QLineSeries *m_threshold = nullptr;   // 阈值线
    QLineSeries *m_zeroLine  = nullptr;   // 零线
    QValueAxis  *m_axisX = nullptr;
    QValueAxis  *m_axisY = nullptr;
    QChartView  *m_view  = nullptr;
    QLabel      *m_placeholder = nullptr;
    QPushButton *m_pauseBtn   = nullptr;
    QPushButton *m_clearBtn   = nullptr;
    QPushButton *m_exportBtn  = nullptr;

    bool m_paused      = false;
    double m_pausedTEnd = 0.0;
    double m_thresholdVal = -1.0;   // < 0 表示未设置

    std::vector<ChartSample> m_buf;
};
