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
#include "ui/UiLogic.h"

class ErrorChart;
class StatusBar;
class CommCards;
class CumulativeBar;
class AlarmLog;

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

    // 预览与「应用」按钮各自的唯一刷新入口。撤销 / 读取当前值都在抑制信号的
    // 情况下改 spinbox，onTargetEdited 不会被调用——原先只能在每个调用点手写
    // 一句固定文案，于是撤销后预览显示的偏差与实际值不符。
    void refreshDeltaPreview();
    void updateApplyButton();

    // 按行高撑满 rows 行。setFixedHeight(170) 装不下表头 + 6 行（实测仅 4 行
    // 可见），B/C 轴要滚动才看得到，而滚动条在栏边缘并不显眼——
    // 一个只显示四分之三数据的表格比没有表格更危险。
    // 高度按实际行高与表头度量算，而不是写死常数：换 DPI 缩放档或系统字体
    // 变大时表头与行都会长高，写死的总高会重新裁掉 C 行。
    static void fitTableToRows(QTableWidget *tbl, int rows);

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
    ErrorChart     *m_chartPos  = nullptr;
    ErrorChart     *m_chartRot  = nullptr;

    // ── 左栏：目标位姿表格 ──
    std::array<QDoubleSpinBox *, 6> m_targetSpin{};
    std::array<QPushButton *, 6>    m_stepMinus{};
    std::array<QPushButton *, 6>    m_stepPlus{};
    std::array<QLabel *, 6>         m_liveLabel{};
    QComboBox   *m_stepSel      = nullptr;
    QLabel      *m_deltaPreview  = nullptr;
    QPushButton *m_applyBtn     = nullptr;
    double       m_appliedTarget[6] = {0,0,0,0,0,0};
    bool         m_targetApplied = true;

    // 上一帧的告警状态，用于边沿触发。持续为真的告警每帧记一条，
    // 4 秒就能把 200 条上限刷满、挤掉之前的真实事件。
    AlarmEdge m_prevAlarms;
    // 丢包告警的迟滞。missedCount 每个正常帧都被归零，间歇丢包在快照里是
    // 一列脉冲而不是一段电平，单靠边沿触发压不住（实测 --drop 5 记 142 条）。
    LossHold  m_lossHold;

    // ── 中栏：位姿对比表格项 ──
    std::array<QTableWidgetItem *, 6> m_actualItem{};
    std::array<QTableWidgetItem *, 6> m_errorItem{};
    std::array<QTableWidgetItem *, 6> m_targetItem{};
    // RKorr 输出：本帧实际发给 KRC 的增量。数据来自 StatusSnapshot::lastDelta，
    // 通信层早就在填，界面此前从未显示——操作员无从判断「主机到底发了什么」。
    std::array<QTableWidgetItem *, 6> m_rkorrItem{};

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
    // 顺序固定：kpPos, kpRot, vmaxPos, vmaxRot, accumPos, accumRot。
    // 标签在 buildMidPanel 里与数值成对创建，改动其一必须同时改另一个，
    // 否则又会回到「标签说的是这个参数、数值是另一个」的状态。
    std::array<QLabel *, 6> m_paramVal{};

    bool m_suppressTargetSignal = false;

    // 两张表所需的总宽（由各自列宽相加得出，见 buildLeftPanel /
    // buildMidPanel）。面板宽度取它而不是写死常数：写死时表比面板宽就出
    // 横向滚动条，而横向滚动条会从下方吃掉半行，把第六行 C 挤出可视区。
    int m_targetTableWidth = 0;
    int m_poseTableWidth   = 0;
};
