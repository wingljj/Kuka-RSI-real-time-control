#pragma once
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QWidget>
#include "core/AppConfig.h"

// 力控配置面板：传感器连接、滤波/死区/导纳参数、方向使能、操作按钮。
// 纯 GUI 对象，归属主线程；任何改动都会 emit configChanged()（程序化
// setConfig 除外），由 MainWindow 汇总后经队列连接下发到 RsiWorker。
class ForcePanel : public QWidget
{
    Q_OBJECT
public:
    explicit ForcePanel(QWidget *parent = nullptr);

    // 程序化写入（抑制 configChanged）。
    void setConfig(const ForceControlConfig &cfg);
    ForceControlConfig config() const;

    // 连接状态（RSI 会话）。驱动按钮启用态：连接/断开、清零/使能/停止。
    void setConnected(bool connected);
    // 当前力/力矩矢量模（从 StatusSnapshot 取，每刷新帧喂一次）。
    void setForceNorm(double fNorm, double tNorm);
    // 传感器数据陈旧/超时警告（默认隐藏）。
    void setStaleWarning(bool visible);

signals:
    void configChanged();
    void connectRequested();
    void disconnectRequested();
    void zeroForceRequested();
    void enableForceRequested();
    void stopForceRequested();

private:
    // ── 传感器 ──
    QLineEdit *m_hostEdit = nullptr;
    QSpinBox  *m_portSpin = nullptr;
    QLabel    *m_connStatus = nullptr;
    QLabel    *m_forceNormLabel = nullptr;
    QPushButton *m_connectBtn = nullptr;
    QPushButton *m_disconnectBtn = nullptr;

    // ── 滤波 ──
    QDoubleSpinBox *m_cutoffSpin = nullptr;

    // ── 死区 ──
    QDoubleSpinBox *m_deadForceSpin = nullptr;
    QDoubleSpinBox *m_deadTorqueSpin = nullptr;

    // ── 导纳 ──
    QDoubleSpinBox *m_gainForceSpin = nullptr;
    QDoubleSpinBox *m_gainTorqueSpin = nullptr;
    QDoubleSpinBox *m_vmaxPosSpin = nullptr;
    QDoubleSpinBox *m_vmaxRotSpin = nullptr;

    // ── 方向使能 ──
    QCheckBox *m_enX = nullptr;
    QCheckBox *m_enY = nullptr;
    QCheckBox *m_enZ = nullptr;
    QCheckBox *m_enA = nullptr;
    QCheckBox *m_enB = nullptr;
    QCheckBox *m_enC = nullptr;

    // ── 操作 ──
    QPushButton *m_zeroBtn = nullptr;
    QPushButton *m_enableBtn = nullptr;
    QPushButton *m_stopBtn = nullptr;

    // 安装参数不在面板上编辑，但 config() 必须原样带回：面板是配置的唯一
    // 编辑入口，若 config() 返回全新默认值，config.json 里的非默认 mounting
    //（法兰到传感器/工具的偏移）会在「使能力控」与切入力控页时被静默丢弃。
    MountingConfig m_mounting;

    // ── 实现细节（brief 未列，setConnected 幂等与陈旧提示所需）──
    QLabel *m_staleLabel = nullptr;   // setStaleWarning 的目标
    // 上次施加的连接状态，避免每帧重复 setPalette。初值取 true：让构造期
    // 的 setConnected(false) 能通过「状态未变」守卫，把按钮初始启用态
    //（只能按「连接」）真正施加一遍——初值 false 会让它成为空操作，
    // 其余四个按钮保持 QPushButton 默认的启用态。
    bool    m_connected  = true;
};
