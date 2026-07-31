#pragma once
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMainWindow>
#include <QSlider>
#include <QThread>
#include <QTimer>
#include <array>
#include "core/AppConfig.h"
#include "net/RsiWorker.h"
#include "net/SharedState.h"

class ErrorChart;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(const AppConfig &cfg, QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onRefresh();
    void onTargetEdited();

private:
    QWidget *buildTargetPanel();
    QWidget *buildReadoutPanel();
    QWidget *buildParamPanel();

    AppConfig    m_cfg;
    SharedState  m_state;
    SampleRing   m_ring;
    QThread     *m_commThread = nullptr;
    RsiWorker   *m_worker     = nullptr;
    QTimer      *m_refresh    = nullptr;

    // 目标位姿输入：6 个滑块 + 6 个数值框联动
    std::array<QSlider *, 6>        m_targetSlider{};
    std::array<QDoubleSpinBox *, 6> m_targetSpin{};

    // 读数：当前位姿 / 误差 / 累积
    std::array<QLabel *, 6> m_actualLabel{};
    std::array<QLabel *, 6> m_errorLabel{};
    std::array<QLabel *, 6> m_accumLabel{};

    QLabel *m_statusLabel = nullptr;
    ErrorChart *m_chart   = nullptr;

    bool m_suppressTargetSignal = false;
};
