#pragma once
#include <QAction>
#include <QByteArray>
#include <QComboBox>
#include <QDockWidget>
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
    void onResetLayout();
    void onAbout();

protected:
    void showEvent(QShowEvent *e) override;
    void closeEvent(QCloseEvent *e) override;

private:
    void saveLayout();

    // ── 面板构建 ──
    // 一个逻辑区块一个函数，各返回一个可直接 setWidget 进 QDockWidget 的部件。
    // 面板内部不再套 QGroupBox：QDockWidget 自带标题栏，再套一层分组框就是
    // 两条同心边框加两遍标题。
    QWidget *buildListenPanel();    // 监听配置（IP / 端口）
    QWidget *buildTargetPanel();    // 目标位姿编辑
    QWidget *buildComparePanel();   // 位姿对比表（含 RKorr 列）
    QWidget *buildCumulPanel();     // 累积修正
    QWidget *buildParamPanel();     // 控制参数
    QWidget *buildCommPanel();      // 通信指标

    void buildMenus();
    void buildStatusBar();
    void updateStatusBar(const StatusSnapshot &s);

    // 全部停靠面板。「视图」菜单、布局存取都遍历它，避免各处手写一份七元素
    // 列表——漏掉一个的表现是该面板没有显示开关、或它的显示状态不被保存。
    std::array<QDockWidget *, 7> docks() const;

    void saveTargetSnapshot();
    void restoreTargetSnapshot();
    void updateConnControls();
    // ButtonStates 的唯一施加点，构造期与每帧刷新共用。理由见 .cpp。
    void applyButtonStates(const ButtonStates &b);

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

    // ── 部件 ──
    CommCards      *m_commCards = nullptr;
    CumulativeBar  *m_cumulBar  = nullptr;
    AlarmLog       *m_alarmLog  = nullptr;
    ErrorChart     *m_chartPos  = nullptr;
    ErrorChart     *m_chartRot  = nullptr;

    // ── 停靠面板 ──
    // 每个都必须 setObjectName：saveState/restoreState 靠 objectName 认面板，
    // 漏设会导致该面板位置不被恢复，且 Qt 在保存时打警告。
    QDockWidget *m_listenDock  = nullptr;
    QDockWidget *m_targetDock  = nullptr;
    QDockWidget *m_compareDock = nullptr;
    QDockWidget *m_cumulDock   = nullptr;
    QDockWidget *m_paramDock   = nullptr;
    QDockWidget *m_commDock    = nullptr;
    QDockWidget *m_alarmDock   = nullptr;

    // ── 菜单 / 工具栏动作 ──
    // 控制动作全是 QAction 而不再是 QPushButton：菜单项、工具栏按钮与快捷键
    // 必须共享同一个启用状态，三份部件各自 setEnabled 迟早分叉。
    QAction *m_startListenAct = nullptr;
    QAction *m_stopListenAct  = nullptr;
    QAction *m_enableAct      = nullptr;
    QAction *m_stopTrackAct   = nullptr;
    QAction *m_resetFaultAct  = nullptr;

    // 建完默认布局、restoreState 之前存一份，供「恢复默认布局」用。
    // 没有它，一个被拖到屏幕外或全部关掉的布局会被 QSettings 忠实地存下来，
    // 下次启动照样是坏的，操作员只能去删注册表。
    QByteArray m_defaultLayout;
    // showEvent 每次显示都会来一次，恢复只做一次。
    bool m_layoutRestored = false;
    // closeEvent 已经存过布局，析构里就别再存一遍（理由见 closeEvent）。
    bool m_layoutSaved    = false;

    // ── 状态栏 ──
    QLabel *m_noticeLabel = nullptr;   // 左侧常驻安全提示（会被 showMessage 暂时盖住）
    QLabel *m_connLabel   = nullptr;
    QLabel *m_stateLabel  = nullptr;
    QLabel *m_ipocLabel   = nullptr;
    QLabel *m_cycleLabel  = nullptr;

    // ── 目标位姿表格 ──
    std::array<QDoubleSpinBox *, 6> m_targetSpin{};
    std::array<QPushButton *, 6>    m_stepMinus{};
    std::array<QPushButton *, 6>    m_stepPlus{};
    std::array<QLabel *, 6>         m_liveLabel{};
    QComboBox   *m_stepSel      = nullptr;
    QLabel      *m_deltaPreview  = nullptr;
    QPushButton *m_applyBtn     = nullptr;
    QPushButton *m_readActualBtn = nullptr;
    double       m_appliedTarget[6] = {0,0,0,0,0,0};
    bool         m_targetApplied = true;

    // 上一帧的告警状态，用于边沿触发。持续为真的告警每帧记一条，
    // 4 秒就能把 200 条上限刷满、挤掉之前的真实事件。
    AlarmEdge m_prevAlarms;
    // 丢包告警的迟滞。missedCount 每个正常帧都被归零，间歇丢包在快照里是
    // 一列脉冲而不是一段电平，单靠边沿触发压不住（实测 --drop 5 记 142 条）。
    LossHold  m_lossHold;

    // ── 位姿对比表格项 ──
    std::array<QTableWidgetItem *, 6> m_actualItem{};
    std::array<QTableWidgetItem *, 6> m_errorItem{};
    std::array<QTableWidgetItem *, 6> m_targetItem{};
    // RKorr 输出：本帧实际发给 KRC 的增量。数据来自 StatusSnapshot::lastDelta，
    // 通信层早就在填，界面此前从未显示——操作员无从判断「主机到底发了什么」。
    std::array<QTableWidgetItem *, 6> m_rkorrItem{};

    // ── 连接 ──
    QLineEdit   *m_ipEdit      = nullptr;
    QSpinBox    *m_portSpin    = nullptr;
    bool         m_listening   = false;

    // ── 参数 ──
    QPushButton *m_paramsBtn = nullptr;
    // 顺序固定：kpPos, kpRot, vmaxPos, vmaxRot, accumPos, accumRot。
    // 标签与数值靠 kParams 里的成员指针绑定（见 .cpp），不是按位置对应——
    // 后者正是最初「六个参数只有一个标对」的成因。
    std::array<QLabel *, 6> m_paramVal{};

    bool m_suppressTargetSignal = false;
};
