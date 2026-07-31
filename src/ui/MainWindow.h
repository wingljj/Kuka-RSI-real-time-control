#pragma once
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
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
    void onZeroToActual();
    void onTrackingToggled(bool on);
    void onStopTracking();

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

    // 两段式使能：连接不等于运动，操作员确认数值后才勾选。
    QCheckBox *m_trackCheck = nullptr;
    QLabel    *m_safetyNote = nullptr;

    bool m_suppressTargetSignal = false;
};
