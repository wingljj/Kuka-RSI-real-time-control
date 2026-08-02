#pragma once
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QMainWindow>
#include <QSpinBox>
#include <QThread>
#include <QTimer>
#include <QTableWidgetItem>
#include <array>
#include "core/AppConfig.h"
#include "net/RsiWorker.h"
#include "net/SharedState.h"

class ErrorChart;
class StatusBar;
class CommCards;
class CumulativeBar;
class AlarmLog;
class TcpView3D;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(const AppConfig &cfg, QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onRefresh();
    void onTargetEdited();
    void onApplyTarget();
    void onUndoTarget();
    void onZeroToActual();
    void onReadActualTarget();
    void onPrepareTracking();
    void onResetFault();
    void onStopTracking();
    void onStartListening();
    void onStopListening();
    void onEditParams();

private:
    // 构建方法
    QWidget *buildLeftPanel();
    QWidget *buildMidPanel();
    QWidget *buildRightPanel();

    void saveTargetSnapshot();
    void restoreTargetSnapshot();
    void updateConnControls();

    AppConfig    m_cfg;
    SharedState  m_state;
    SampleRing   m_ring;
    QThread     *m_commThread = nullptr;
    RsiWorker   *m_worker     = nullptr;
    QTimer      *m_refresh    = nullptr;

    // ── 新组件 ──
    StatusBar      *m_statusBar = nullptr;
    CommCards      *m_commCards = nullptr;
    CumulativeBar  *m_cumulBar  = nullptr;
    AlarmLog       *m_alarmLog  = nullptr;
    TcpView3D      *m_tcpView   = nullptr;
    ErrorChart     *m_chartPos  = nullptr;
    ErrorChart     *m_chartRot  = nullptr;

    // ── 左栏：目标位姿表格 ──
    std::array<QDoubleSpinBox *, 6> m_targetSpin{};
    std::array<QPushButton *, 6>    m_stepMinus{};
    std::array<QPushButton *, 6>    m_stepPlus{};
    std::array<QLabel *, 6>         m_liveLabel{};
    QComboBox   *m_stepSel      = nullptr;
    QLabel      *m_deltaPreview  = nullptr;
    double       m_appliedTarget[6] = {0,0,0,0,0,0};
    bool         m_targetApplied = true;

    // ── 中栏：位姿对比表格项 ──
    std::array<QTableWidgetItem *, 6> m_actualItem{};
    std::array<QTableWidgetItem *, 6> m_errorItem{};
    std::array<QTableWidgetItem *, 6> m_targetItem{};

    // ── 控制按钮 ──
    QPushButton *m_enableBtn    = nullptr;
    QPushButton *m_resetFaultBtn = nullptr;
    QPushButton *m_stopBtn      = nullptr;
    QPushButton *m_listenBtn    = nullptr;
    QPushButton *m_unlistenBtn  = nullptr;
    QLabel      *m_interlockLabel = nullptr;

    // ── 连接 ──
    QLineEdit   *m_ipEdit      = nullptr;
    QSpinBox    *m_portSpin    = nullptr;
    bool         m_listening   = false;

    // ── 参数 ──
    QPushButton *m_paramsBtn = nullptr;
    std::array<QLabel *, 6> m_paramVal{};

    bool m_suppressTargetSignal = false;
};
