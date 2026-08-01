#pragma once
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QMainWindow>
#include <QSlider>
#include <QSpinBox>
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
    void onStartListening();
    void onStopListening();

private:
    QWidget *buildTargetPanel();
    QWidget *buildReadoutPanel();
    QWidget *buildParamPanel();
    QWidget *buildConnPanel();
    QWidget *buildSingularWarn();

    // 依据当前是否已绑定，切换监听按钮与地址输入的可用状态。
    // 地址与端口只在未绑定时可编辑——运行中改它们毫无意义，而且会让界面
    // 显示的地址与实际绑定的地址不符。
    void updateConnControls();

    AppConfig    m_cfg;
    SharedState  m_state;
    SampleRing   m_ring;
    QThread     *m_commThread = nullptr;
    RsiWorker   *m_worker     = nullptr;
    QTimer      *m_refresh    = nullptr;

    // 目标位姿输入：6 个滑块 + 6 个数值框联动
    std::array<QSlider *, 6>        m_targetSlider{};
    std::array<QDoubleSpinBox *, 6> m_targetSpin{};

    // 紧贴目标数值框右侧的当前位姿。读数面板在曲线下方，窗口一矮就被滚出
    // 视野；而"目标给了多少 / 现在到哪了"是操作时最需要并排看的一对值。
    std::array<QLabel *, 6> m_liveLabel{};

    // 读数：当前位姿 / 误差 / 累积
    std::array<QLabel *, 6> m_actualLabel{};
    std::array<QLabel *, 6> m_errorLabel{};
    std::array<QLabel *, 6> m_accumLabel{};

    QLabel *m_stateCard   = nullptr;   // 大字状态卡（颜色分级）
    QLabel *m_stateDetail = nullptr;   // 次行诊断详情
    ErrorChart *m_chart   = nullptr;

    // 两段式使能：连接不等于运动，操作员确认数值后才勾选。
    QCheckBox *m_trackCheck = nullptr;
    QLabel    *m_safetyNote = nullptr;

    // 联锁拦截原因（红字）。硬拦截：使能不通过时置红字，无覆盖入口。
    QLabel *m_interlockLabel = nullptr;

    // 欧拉奇异区警告（B≈±90° 时姿态控制退化）。黄色提示，不拦截。
    QLabel *m_singularWarnLabel = nullptr;

    // 连接配置与手动监听控制。没有这些的话 bindFailed 是个死局：
    // 弹一次对话框之后应用永久停在未连接，只能改 JSON 再重启。
    QLineEdit  *m_ipEdit    = nullptr;
    QSpinBox   *m_portSpin  = nullptr;
    QPushButton *m_listenBtn = nullptr;
    QPushButton *m_unlistenBtn = nullptr;

    // 是否已成功绑定。区别于 StatusSnapshot::connected（那表示已收到帧）：
    // 「已绑定但一帧未收」和「根本没绑上」对操作员是两件完全不同的事。
    bool m_listening = false;

    bool m_suppressTargetSignal = false;
};
