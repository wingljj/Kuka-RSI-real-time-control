#include "ui/ForcePanel.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPalette>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include "ui/UiLogic.h"

// 面板控件范围与 SessionGuard::staticChecks 的力控硬校验一致
//（cutoff ∈ (0,60]；gains > 0；vmax > 0 且 vmax × cycle_ms ≤ 35mm）。
namespace {
constexpr double kCutoffMaxHz  = 60.0;
constexpr double kVmaxPosMax   = 500.0;   // mm/s（×12ms = 6mm/帧 < 35mm）
constexpr double kVmaxRotMax   = 100.0;   // deg/s（×12ms = 1.2deg/帧 < 35deg）
} // namespace

ForcePanel::ForcePanel(QWidget *parent)
    : QWidget(parent)
{
    auto *v = new QVBoxLayout(this);
    v->setContentsMargins(6, 4, 6, 4);
    v->setSpacing(6);

    // ── 传感器 ──
    auto *sensorGroup = new QGroupBox("传感器", this);
    auto *sf = new QFormLayout(sensorGroup);
    sf->setSpacing(4);
    m_hostEdit = new QLineEdit(sensorGroup);
    m_hostEdit->setObjectName("sensorHostEdit");
    m_portSpin = new QSpinBox(sensorGroup);
    m_portSpin->setObjectName("sensorPortSpin");
    m_portSpin->setRange(1, 65535);
    sf->addRow("IP", m_hostEdit);
    sf->addRow("端口", m_portSpin);

    m_connStatus = new QLabel("未连接", sensorGroup);
    m_connStatus->setObjectName("sensorConnStatus");
    sf->addRow("状态", m_connStatus);

    m_forceNormLabel = new QLabel("--", sensorGroup);
    m_forceNormLabel->setObjectName("sensorForceNorm");
    m_forceNormLabel->setFont(uilogic::monospaceFont());
    sf->addRow("当前矢量", m_forceNormLabel);

    m_staleLabel = new QLabel("传感器数据陈旧/超时", sensorGroup);
    m_staleLabel->setObjectName("sensorStaleWarning");
    QPalette stalePal = m_staleLabel->palette();
    stalePal.setColor(QPalette::WindowText,
                      uilogic::severityColor(uilogic::Severity::Warn));
    m_staleLabel->setPalette(stalePal);
    m_staleLabel->setVisible(false);
    sf->addRow(m_staleLabel);

    auto *connRow = new QHBoxLayout;
    m_connectBtn = new QPushButton("连接", sensorGroup);
    m_connectBtn->setObjectName("sensorConnectBtn");
    m_disconnectBtn = new QPushButton("断开", sensorGroup);
    m_disconnectBtn->setObjectName("sensorDisconnectBtn");
    connRow->addWidget(m_connectBtn);
    connRow->addWidget(m_disconnectBtn);
    sf->addRow(connRow);
    v->addWidget(sensorGroup);

    // ── 滤波 ──
    auto *filterGroup = new QGroupBox("滤波", this);
    auto *ff = new QFormLayout(filterGroup);
    ff->setSpacing(4);
    m_cutoffSpin = new QDoubleSpinBox(filterGroup);
    m_cutoffSpin->setObjectName("filterCutoffSpin");
    m_cutoffSpin->setRange(0.1, kCutoffMaxHz);
    m_cutoffSpin->setDecimals(1);
    m_cutoffSpin->setSingleStep(1.0);
    m_cutoffSpin->setSuffix(" Hz");
    ff->addRow("截止频率", m_cutoffSpin);
    v->addWidget(filterGroup);

    // ── 死区 ──
    auto *deadGroup = new QGroupBox("死区", this);
    auto *df = new QFormLayout(deadGroup);
    df->setSpacing(4);
    m_deadForceSpin = new QDoubleSpinBox(deadGroup);
    m_deadForceSpin->setObjectName("deadzoneForceSpin");
    m_deadForceSpin->setRange(0.0, 10000.0);
    m_deadForceSpin->setDecimals(1);
    m_deadForceSpin->setSingleStep(0.5);
    m_deadForceSpin->setSuffix(" N");
    df->addRow("力", m_deadForceSpin);
    m_deadTorqueSpin = new QDoubleSpinBox(deadGroup);
    m_deadTorqueSpin->setObjectName("deadzoneTorqueSpin");
    m_deadTorqueSpin->setRange(0.0, 10000.0);
    m_deadTorqueSpin->setDecimals(1);
    m_deadTorqueSpin->setSingleStep(0.5);
    m_deadTorqueSpin->setSuffix(" Nm");
    df->addRow("力矩", m_deadTorqueSpin);
    v->addWidget(deadGroup);

    // ── 导纳 ──
    auto *admGroup = new QGroupBox("导纳", this);
    auto *af = new QFormLayout(admGroup);
    af->setSpacing(4);
    m_gainForceSpin = new QDoubleSpinBox(admGroup);
    m_gainForceSpin->setObjectName("admittanceGainForceSpin");
    m_gainForceSpin->setRange(0.001, 1.0);
    m_gainForceSpin->setDecimals(3);
    m_gainForceSpin->setSingleStep(0.005);
    af->addRow("力增益", m_gainForceSpin);
    m_gainTorqueSpin = new QDoubleSpinBox(admGroup);
    m_gainTorqueSpin->setObjectName("admittanceGainTorqueSpin");
    m_gainTorqueSpin->setRange(0.001, 1.0);
    m_gainTorqueSpin->setDecimals(3);
    m_gainTorqueSpin->setSingleStep(0.005);
    af->addRow("力矩增益", m_gainTorqueSpin);
    m_vmaxPosSpin = new QDoubleSpinBox(admGroup);
    m_vmaxPosSpin->setObjectName("admittanceVmaxPosSpin");
    m_vmaxPosSpin->setRange(0.1, kVmaxPosMax);
    m_vmaxPosSpin->setDecimals(1);
    m_vmaxPosSpin->setSingleStep(0.5);
    m_vmaxPosSpin->setSuffix(" mm/s");
    af->addRow("限速位置", m_vmaxPosSpin);
    m_vmaxRotSpin = new QDoubleSpinBox(admGroup);
    m_vmaxRotSpin->setObjectName("admittanceVmaxRotSpin");
    m_vmaxRotSpin->setRange(0.1, kVmaxRotMax);
    m_vmaxRotSpin->setDecimals(1);
    m_vmaxRotSpin->setSingleStep(0.5);
    m_vmaxRotSpin->setSuffix(" deg/s");
    af->addRow("限速姿态", m_vmaxRotSpin);
    v->addWidget(admGroup);

    // ── 方向使能 ──
    auto *axesGroup = new QGroupBox("方向使能", this);
    auto *ag = new QGridLayout(axesGroup);
    ag->setSpacing(4);
    m_enX = new QCheckBox("X", axesGroup);
    m_enY = new QCheckBox("Y", axesGroup);
    m_enZ = new QCheckBox("Z", axesGroup);
    m_enA = new QCheckBox("A", axesGroup);
    m_enB = new QCheckBox("B", axesGroup);
    m_enC = new QCheckBox("C", axesGroup);
    // 三列两行排布：位置轴 X/Y/Z 一行、姿态轴 A/B/C 一行，比六个竖排省一半高度。
    QCheckBox *axes6[6] = {m_enX, m_enY, m_enZ, m_enA, m_enB, m_enC};
    for (int i = 0; i < 6; ++i) {
        QCheckBox *cb = axes6[i];
        cb->setObjectName(QStringLiteral("axisEn%1").arg(cb->text()));
        ag->addWidget(cb, i / 3, i % 3);
    }
    v->addWidget(axesGroup);

    // ── 操作 ──
    auto *actGroup = new QGroupBox("操作", this);
    auto *ar = new QHBoxLayout(actGroup);
    ar->setSpacing(4);
    m_zeroBtn = new QPushButton("力清零", actGroup);
    m_zeroBtn->setObjectName("zeroForceBtn");
    m_enableBtn = new QPushButton("使能力控", actGroup);
    m_enableBtn->setObjectName("enableForceBtn");
    m_stopBtn = new QPushButton("停止", actGroup);
    m_stopBtn->setObjectName("stopForceBtn");
    ar->addWidget(m_zeroBtn);
    ar->addWidget(m_enableBtn);
    ar->addWidget(m_stopBtn);
    v->addWidget(actGroup);

    v->addStretch();

    // ── 信号：任何参数改动 → configChanged（setConfig 走抑制路径，不触发）──
    auto onChanged = [this] { emit configChanged(); };
    connect(m_hostEdit, &QLineEdit::textEdited, this, onChanged);
    connect(m_portSpin, &QSpinBox::valueChanged, this, onChanged);
    connect(m_cutoffSpin, &QDoubleSpinBox::valueChanged, this, onChanged);
    connect(m_deadForceSpin, &QDoubleSpinBox::valueChanged, this, onChanged);
    connect(m_deadTorqueSpin, &QDoubleSpinBox::valueChanged, this, onChanged);
    connect(m_gainForceSpin, &QDoubleSpinBox::valueChanged, this, onChanged);
    connect(m_gainTorqueSpin, &QDoubleSpinBox::valueChanged, this, onChanged);
    connect(m_vmaxPosSpin, &QDoubleSpinBox::valueChanged, this, onChanged);
    connect(m_vmaxRotSpin, &QDoubleSpinBox::valueChanged, this, onChanged);
    for (QCheckBox *cb : {m_enX, m_enY, m_enZ, m_enA, m_enB, m_enC})
        connect(cb, &QCheckBox::toggled, this, onChanged);

    // ── 操作按钮 → 请求信号（由 MainWindow 跨线程转发给 RsiWorker）──
    connect(m_connectBtn, &QPushButton::clicked,
            this, &ForcePanel::connectRequested);
    connect(m_disconnectBtn, &QPushButton::clicked,
            this, &ForcePanel::disconnectRequested);
    connect(m_zeroBtn, &QPushButton::clicked,
            this, &ForcePanel::zeroForceRequested);
    connect(m_enableBtn, &QPushButton::clicked,
            this, &ForcePanel::enableForceRequested);
    connect(m_stopBtn, &QPushButton::clicked,
            this, &ForcePanel::stopForceRequested);

    // 初始状态：未连接 → 只能按「连接」
    setConnected(false);
}

void ForcePanel::setConfig(const ForceControlConfig &cfg)
{
    // 程序化写入不产生 configChanged：信号语义是「操作员改了参数」。
    const QSignalBlocker hb(m_hostEdit), pb(m_portSpin), cb(m_cutoffSpin),
        dfb(m_deadForceSpin), dtb(m_deadTorqueSpin), gfb(m_gainForceSpin),
        gtb(m_gainTorqueSpin), vpb(m_vmaxPosSpin), vrb(m_vmaxRotSpin),
        xb(m_enX), yb(m_enY), zb(m_enZ), ab(m_enA), bb(m_enB), cc(m_enC);

    m_hostEdit->setText(cfg.sensor.host);
    m_portSpin->setValue(int(cfg.sensor.port));
    m_cutoffSpin->setValue(cfg.params.cutoffHz);
    m_deadForceSpin->setValue(cfg.params.deadzoneForceN);
    m_deadTorqueSpin->setValue(cfg.params.deadzoneTorqueNm);
    m_gainForceSpin->setValue(cfg.params.gainForce);
    m_gainTorqueSpin->setValue(cfg.params.gainTorque);
    m_vmaxPosSpin->setValue(cfg.params.vmaxPosMmS);
    m_vmaxRotSpin->setValue(cfg.params.vmaxRotDegS);
    m_enX->setChecked(cfg.axes.enX);
    m_enY->setChecked(cfg.axes.enY);
    m_enZ->setChecked(cfg.axes.enZ);
    m_enA->setChecked(cfg.axes.enA);
    m_enB->setChecked(cfg.axes.enB);
    m_enC->setChecked(cfg.axes.enC);
}

ForceControlConfig ForcePanel::config() const
{
    ForceControlConfig c;
    c.sensor.host = m_hostEdit->text().trimmed();
    c.sensor.port = quint16(m_portSpin->value());
    c.params.cutoffHz = m_cutoffSpin->value();
    c.params.deadzoneForceN = m_deadForceSpin->value();
    c.params.deadzoneTorqueNm = m_deadTorqueSpin->value();
    c.params.gainForce = m_gainForceSpin->value();
    c.params.gainTorque = m_gainTorqueSpin->value();
    c.params.vmaxPosMmS = m_vmaxPosSpin->value();
    c.params.vmaxRotDegS = m_vmaxRotSpin->value();
    c.axes.enX = m_enX->isChecked();
    c.axes.enY = m_enY->isChecked();
    c.axes.enZ = m_enZ->isChecked();
    c.axes.enA = m_enA->isChecked();
    c.axes.enB = m_enB->isChecked();
    c.axes.enC = m_enC->isChecked();
    // 安装配置（mounting）不在面板编辑范围内，保留默认值。
    return c;
}

void ForcePanel::setConnected(bool connected)
{
    if (connected == m_connected)
        return;   // 每刷新帧都来：连接状态稳定时不该反复 setText/setPalette

    m_connected = connected;
    const QString txt = connected ? "已连接" : "未连接";
    if (m_connStatus->text() != txt)
        m_connStatus->setText(txt);
    const QColor fg = uilogic::severityColor(
        connected ? uilogic::Severity::Ok : uilogic::Severity::Idle);
    if (m_connStatus->palette().color(QPalette::WindowText) != fg) {
        QPalette p = m_connStatus->palette();
        p.setColor(QPalette::WindowText, fg);
        m_connStatus->setPalette(p);
    }

    // 按钮启用态随连接状态：断开状态下清零/使能/停止都没有对象可作用。
    m_connectBtn->setEnabled(!connected);
    m_disconnectBtn->setEnabled(connected);
    m_zeroBtn->setEnabled(connected);
    m_enableBtn->setEnabled(connected);
    m_stopBtn->setEnabled(connected);
}

void ForcePanel::setForceNorm(double fNorm, double tNorm)
{
    const QString txt = QStringLiteral("力 %1 N  力矩 %2 Nm")
                            .arg(fNorm, 0, 'f', 1)
                            .arg(tNorm, 0, 'f', 1);
    if (m_forceNormLabel->text() != txt)
        m_forceNormLabel->setText(txt);
}

void ForcePanel::setStaleWarning(bool visible)
{
    if (m_staleLabel->isVisible() != visible)
        m_staleLabel->setVisible(visible);
}
