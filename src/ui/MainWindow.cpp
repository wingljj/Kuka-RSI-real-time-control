#include "ui/MainWindow.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QHostAddress>
#include <QMessageBox>
#include <QScreen>
#include <QScrollArea>
#include <QSplitter>
#include <QTableWidget>
#include <QVBoxLayout>
#include <cmath>
#include "core/SessionGuard.h"
#include "ui/AlarmLog.h"
#include "ui/CommCards.h"
#include "ui/CumulativeBar.h"
#include "ui/ErrorChart.h"
#include "ui/StatusBar.h"

namespace {

const char *kAxisName[6] = {"X", "Y", "Z", "A", "B", "C"};
const char *kAxisUnit(int i) { return i < 3 ? " mm" : " deg"; }
double axisMin(int i) { return i < 3 ? -4000.0 : -180.0; }
double axisMax(int i) { return i < 3 ?  4000.0 :  180.0; }

} // namespace

// ═══════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════

MainWindow::MainWindow(const AppConfig &cfg, QWidget *parent)
    : QMainWindow(parent), m_cfg(cfg)
{
    setWindowTitle("KUKA RSI POSCORR 位姿跟踪");

    auto *central = new QWidget(this);
    auto *outer = new QVBoxLayout(central);
    outer->setContentsMargins(12, 8, 12, 8);
    outer->setSpacing(8);

    // ── 顶部：StatusBar（4 卡片 + 警告） ──
    m_statusBar = new StatusBar(this);
    m_statusBar->setWarning(
        "软停止：RKorr=0，RSI 回包保持。不是急停。紧急情况请按示教器物理急停。", false);
    outer->addWidget(m_statusBar);

    // ── 按钮栏 ──
    auto *btnBar = new QHBoxLayout;
    btnBar->setSpacing(6);

    m_listenBtn = new QPushButton("开始监听", this);
    m_listenBtn->setProperty("cssClass", "primary");
    connect(m_listenBtn, &QPushButton::clicked, this, &MainWindow::onStartListening);
    btnBar->addWidget(m_listenBtn);

    m_unlistenBtn = new QPushButton("停止监听", this);
    connect(m_unlistenBtn, &QPushButton::clicked, this, &MainWindow::onStopListening);
    btnBar->addWidget(m_unlistenBtn);

    auto *sep1 = new QLabel("→", this);
    sep1->setStyleSheet("color: #D1D5DB; font-size: 14px;");
    btnBar->addWidget(sep1);

    m_resetFaultBtn = new QPushButton("复位故障", this);
    m_resetFaultBtn->setProperty("cssClass", "warning");
    m_resetFaultBtn->setEnabled(false);
    connect(m_resetFaultBtn, &QPushButton::clicked, this, &MainWindow::onResetFault);
    btnBar->addWidget(m_resetFaultBtn);

    m_enableBtn = new QPushButton("使能跟踪", this);
    m_enableBtn->setEnabled(false);
    m_enableBtn->setProperty("cssClass", "primary");
    connect(m_enableBtn, &QPushButton::clicked, this, &MainWindow::onPrepareTracking);
    btnBar->addWidget(m_enableBtn);

    m_stopBtn = new QPushButton("停止跟踪", this);
    m_stopBtn->setProperty("cssClass", "danger");
    connect(m_stopBtn, &QPushButton::clicked, this, &MainWindow::onStopTracking);
    btnBar->addWidget(m_stopBtn);

    btnBar->addStretch();

    outer->addLayout(btnBar);

    // 联锁拦截
    m_interlockLabel = new QLabel(this);
    m_interlockLabel->setStyleSheet(
        "color: #DC2626; font-weight: bold; padding: 4px 10px; "
        "background-color: #FEF2F2; border: 1px solid #FECACA; border-radius: 4px;");
    m_interlockLabel->setWordWrap(true);
    m_interlockLabel->hide();
    outer->addWidget(m_interlockLabel);

    // ── 三栏主体 ──
    auto *body = new QHBoxLayout;
    body->setSpacing(12);

    auto *leftPanel  = buildLeftPanel();
    leftPanel->setFixedWidth(360);
    body->addWidget(leftPanel);

    auto *midPanel   = buildMidPanel();
    midPanel->setFixedWidth(420);
    body->addWidget(midPanel);

    auto *rightPanel = buildRightPanel();
    body->addWidget(rightPanel, 1);

    outer->addLayout(body, 1);

    // ── 底部：AlarmLog（默认折叠） ──
    m_alarmLog = new AlarmLog(this);
    outer->addWidget(m_alarmLog);

    setCentralWidget(central);

    // ── 通信线程 ──
    m_commThread = new QThread(this);
    m_worker = new RsiWorker(m_cfg, &m_state, &m_ring);
    m_worker->moveToThread(m_commThread);
    connect(m_commThread, &QThread::started, m_worker, &RsiWorker::start);
    connect(m_commThread, &QThread::finished, m_worker, &QObject::deleteLater);
    connect(m_worker, &RsiWorker::listening, this, [this] {
        m_listening = true; updateConnControls(); });
    connect(m_worker, &RsiWorker::bindFailed, this, [this](const QString &why) {
        m_listening = false; updateConnControls();
        QMessageBox::critical(this, "监听失败", why); });
    connect(m_worker, &RsiWorker::firstFrameReceived, this, [this] {
        const Pose a = m_state.snapshot().actual;
        m_suppressTargetSignal = true;
        for (int i = 0; i < 6; ++i)
            m_targetSpin[i]->setValue((&a.x)[i]);
        m_suppressTargetSignal = false;
        saveTargetSnapshot();
    });
    m_commThread->start();

    m_refresh = new QTimer(this);
    m_refresh->setInterval(m_cfg.refreshMs);
    connect(m_refresh, &QTimer::timeout, this, &MainWindow::onRefresh);
    m_refresh->start();

    const QRect wa = QGuiApplication::primaryScreen()->availableGeometry();
    resize(qMin(1400, wa.width() - 80), qMin(860, wa.height() - 80));
    updateConnControls();
    saveTargetSnapshot();
}

MainWindow::~MainWindow()
{
    if (m_commThread && m_commThread->isRunning()) {
        QMetaObject::invokeMethod(m_worker, "stop", Qt::BlockingQueuedConnection);
        m_commThread->quit();
        m_commThread->wait(2000);
    }
}

// ═══════════════════════════════════════════════
// 左栏：连接配置 + 目标位姿表格
// ═══════════════════════════════════════════════

QWidget *MainWindow::buildLeftPanel()
{
    auto *w = new QWidget(this);
    auto *v = new QVBoxLayout(w);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(12);

    // ── 连接配置 ──
    auto *connBox = new QGroupBox("监听配置", w);
    auto *connLay = new QHBoxLayout(connBox);
    connLay->addWidget(new QLabel("IP", connBox));
    m_ipEdit = new QLineEdit(m_cfg.listenIp, connBox);
    m_ipEdit->setMaximumWidth(120);
    connLay->addWidget(m_ipEdit);
    connLay->addWidget(new QLabel(":", connBox));
    m_portSpin = new QSpinBox(connBox);
    m_portSpin->setRange(1, 65535);
    m_portSpin->setValue(int(m_cfg.listenPort));
    m_portSpin->setMaximumWidth(80);
    connLay->addWidget(m_portSpin);
    connLay->addStretch();
    v->addWidget(connBox);

    // ── 目标位姿表格 ──
    auto *tgtBox = new QGroupBox("目标位姿 (BASE)", w);
    auto *tgtV = new QVBoxLayout(tgtBox);
    tgtV->setSpacing(4);

    auto *tbl = new QTableWidget(6, 4, tgtBox);
    tbl->setHorizontalHeaderLabels({"轴", "目标值", "当前值（只读）", "调整"});
    tbl->verticalHeader()->setVisible(false);
    tbl->setShowGrid(false);
    tbl->setSelectionMode(QAbstractItemView::NoSelection);
    tbl->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tbl->setStyleSheet(
        "QTableWidget { border: none; font-size: 10px; } "
        "QTableWidget::item { padding: 0px 4px; }");
    tbl->horizontalHeader()->setStretchLastSection(true);
    tbl->setColumnWidth(0, 28);
    tbl->setColumnWidth(1, 110);
    tbl->setColumnWidth(2, 110);
    tbl->setFixedHeight(170);

    for (int i = 0; i < 6; ++i) {
        // 轴名
        auto *ax = new QTableWidgetItem(kAxisName[i]);
        ax->setFlags(Qt::NoItemFlags);
        auto axF = ax->font(); axF.setBold(true); ax->setFont(axF);
        tbl->setItem(i, 0, ax);

        // 目标值 spinbox
        auto *sp = new QDoubleSpinBox(tbl);
        sp->setRange(axisMin(i), axisMax(i));
        sp->setDecimals(2);
        sp->setSingleStep(0.5);
        sp->setSuffix(kAxisUnit(i));
        sp->setKeyboardTracking(false);
        sp->setStyleSheet("QDoubleSpinBox { border: 1px solid #D9E0E7; border-radius: 3px; "
                          "font-family: Consolas, monospace; padding: 1px 4px; } "
                          "QDoubleSpinBox:focus { border-color: #2563EB; }");
        tbl->setCellWidget(i, 1, sp);
        m_targetSpin[i] = sp;

        // 当前值 label
        auto *live = new QLabel("--", tbl);
        live->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        live->setStyleSheet(
            "font-family: Consolas, monospace; font-size: 10px; "
            "color: #1F2937; background: #F3F4F6; border-radius: 2px; padding: 2px 6px;");
        tbl->setCellWidget(i, 2, live);
        m_liveLabel[i] = live;

        // +/- 按钮
        auto *btnW = new QWidget(tbl);
        auto *btnL = new QHBoxLayout(btnW);
        btnL->setContentsMargins(0, 0, 0, 0);
        btnL->setSpacing(2);
        auto *minus = new QPushButton("−", btnW);
        minus->setFixedSize(22, 22);
        minus->setStyleSheet("QPushButton { font-size: 12px; padding: 0; }");
        btnL->addWidget(minus);
        m_stepMinus[i] = minus;
        auto *plus = new QPushButton("+", btnW);
        plus->setFixedSize(22, 22);
        plus->setStyleSheet("QPushButton { font-size: 12px; padding: 0; }");
        btnL->addWidget(plus);
        m_stepPlus[i] = plus;
        btnL->addStretch();
        tbl->setCellWidget(i, 3, btnW);

        const int axis = i;
        connect(minus, &QPushButton::clicked, this, [this, axis] {
            const double step = m_stepSel->currentText().toDouble();
            m_targetSpin[axis]->setValue(m_targetSpin[axis]->value() - step);
        });
        connect(plus, &QPushButton::clicked, this, [this, axis] {
            const double step = m_stepSel->currentText().toDouble();
            m_targetSpin[axis]->setValue(m_targetSpin[axis]->value() + step);
        });
        connect(sp, &QDoubleSpinBox::valueChanged, this, [this](double) {
            onTargetEdited();
        });
    }
    tgtV->addWidget(tbl);

    // 步长选择
    auto *stepRow = new QHBoxLayout;
    stepRow->addWidget(new QLabel("步长", tgtBox));
    m_stepSel = new QComboBox(tgtBox);
    m_stepSel->addItems({"0.01", "0.1", "0.5", "1", "5", "10"});
    m_stepSel->setCurrentText("0.5");
    stepRow->addWidget(m_stepSel);
    stepRow->addStretch();
    tgtV->addLayout(stepRow);

    // 差值预览（浅黄提示条）
    m_deltaPreview = new QLabel("差值预览：编辑目标值后此处显示当前偏差", tgtBox);
    m_deltaPreview->setStyleSheet(
        "font-family: Consolas, monospace; font-size: 9px; color: #92400E; "
        "padding: 4px 8px; background-color: #FEF3C7; border-radius: 4px;");
    m_deltaPreview->setWordWrap(true);
    tgtV->addWidget(m_deltaPreview);

    // 操作按钮
    auto *actRow = new QHBoxLayout;
    auto *setCur = new QPushButton("读取当前值", tgtBox);
    setCur->setProperty("cssClass", "secondary");
    connect(setCur, &QPushButton::clicked, this, &MainWindow::onReadActualTarget);
    actRow->addWidget(setCur);
    auto *apply = new QPushButton("应用目标", tgtBox);
    apply->setStyleSheet(
        "QPushButton { background-color: #2563EB; color: #FFF; font-weight: bold; }");
    connect(apply, &QPushButton::clicked, this, &MainWindow::onApplyTarget);
    actRow->addWidget(apply);
    auto *undo = new QPushButton("撤销修改", tgtBox);
    undo->setProperty("cssClass", "secondary");
    connect(undo, &QPushButton::clicked, this, &MainWindow::onUndoTarget);
    actRow->addWidget(undo);
    actRow->addStretch();
    tgtV->addLayout(actRow);

    v->addWidget(tgtBox);
    v->addStretch();
    return w;
}

// ═══════════════════════════════════════════════
// 中栏：位姿对比 + 安全限制
// ═══════════════════════════════════════════════

QWidget *MainWindow::buildMidPanel()
{
    auto *w = new QWidget(this);
    auto *v = new QVBoxLayout(w);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(12);

    // ── 位姿对比 ──
    auto *poseBox = new QGroupBox("位姿对比", w);
    // 轴名占第 0 列而不是垂直表头：表头下一行就被 setVisible(false) 隐藏，
    // 写进去的 X/Y/Z/A/B/C 永远显示不出来，六行数字谁也不知道是哪个轴。
    // 第 4 列 RKorr 输出留空，Task 5 填充。
    auto *tbl = new QTableWidget(6, 5, poseBox);
    tbl->setHorizontalHeaderLabels(
        {"轴", "当前实际", "实时误差", "目标位姿", "RKorr 输出"});
    tbl->verticalHeader()->setVisible(false);
    tbl->setShowGrid(false);
    tbl->setSelectionMode(QAbstractItemView::NoSelection);
    tbl->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tbl->setStyleSheet(
        "QTableWidget { border: none; font-size: 10px; } "
        "QTableWidget::item { padding: 0px 6px; }");
    tbl->horizontalHeader()->setStretchLastSection(true);
    tbl->setColumnWidth(0, 28);
    tbl->setColumnWidth(1, 92);
    tbl->setColumnWidth(2, 92);
    tbl->setColumnWidth(3, 92);
    // 六行必须一屏放下：默认行高下只露出 X/Y/Z/A，B 与 C 被挤到滚动条以外，
    // 轴名从「看不见」变成「要滚动才看得见」，对操作员是同一个问题。
    // 高度 = 表头 + 6×22 行 + 余量，宁可留白也不要出现纵向滚动条。
    tbl->verticalHeader()->setDefaultSectionSize(22);
    tbl->setFixedHeight(190);

    for (int i = 0; i < 6; ++i) {
        auto *ax = new QTableWidgetItem(kAxisName[i]);
        ax->setFlags(Qt::NoItemFlags);
        ax->setTextAlignment(Qt::AlignCenter);
        auto axF = ax->font(); axF.setBold(true); ax->setFont(axF);
        tbl->setItem(i, 0, ax);

        // 当前实际
        auto *act = new QTableWidgetItem("--");
        act->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        act->setFlags(Qt::NoItemFlags);
        auto actF = act->font(); actF.setFamily("Consolas"); actF.setPointSize(10);
        actF.setBold(true); act->setFont(actF);
        act->setForeground(QColor("#1F2937"));
        tbl->setItem(i, 1, act);
        m_actualItem[i] = act;

        // 误差
        auto *err = new QTableWidgetItem("--");
        err->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        err->setFlags(Qt::NoItemFlags);
        auto errF = err->font(); errF.setFamily("Consolas"); errF.setPointSize(10);
        errF.setBold(true); err->setFont(errF);
        err->setForeground(QColor("#64748B"));
        tbl->setItem(i, 2, err);
        m_errorItem[i] = err;

        // 目标（只显示，不编辑——编辑在左栏）
        auto *tgt = new QTableWidgetItem("--");
        tgt->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        tgt->setFlags(Qt::NoItemFlags);
        auto tgtF2 = tgt->font(); tgtF2.setFamily("Consolas"); tgtF2.setPointSize(10);
        tgt->setFont(tgtF2);
        tgt->setForeground(QColor("#2563EB"));
        tbl->setItem(i, 3, tgt);
        m_targetItem[i] = tgt;
    }
    auto *poseLay = new QVBoxLayout(poseBox);
    poseLay->setContentsMargins(4, 4, 4, 4);
    poseLay->addWidget(tbl);
    v->addWidget(poseBox);

    // ── 安全限制 ──
    auto *safeBox = new QGroupBox("安全限制", w);
    auto *safeV = new QVBoxLayout(safeBox);
    safeV->setSpacing(6);

    m_cumulBar = new CumulativeBar(safeBox);
    safeV->addWidget(m_cumulBar);

    // 控制参数只读行
    // 标签与数值在同一次循环里成对创建。原实现把标签按
    // (1,0)(2,0)(1,2)(2,2)(1,4)(2,4) 摆放、数值按 (r+1, c*2+1) 摆放，
    // 六个参数里只有第一个对得上——操作员照标签读到的是别的参数。
    // 成对创建让这种错位在结构上不可能发生。
    struct ParamRow { const char *label; };
    const ParamRow kParams[6] = {
        {"Kp 位置"},   {"Kp 姿态"},
        {"限速位置"},  {"限速姿态"},
        {"累积上限位置"}, {"累积上限姿态"},
    };

    auto *paramForm = new QGridLayout;
    paramForm->setHorizontalSpacing(12);
    paramForm->setVerticalSpacing(4);
    for (int i = 0; i < 6; ++i) {
        const int row = i / 2;
        const int col = (i % 2) * 2;
        auto *name = new QLabel(kParams[i].label, safeBox);
        name->setProperty("cssClass", "fieldLabel");
        paramForm->addWidget(name, row, col);

        auto *val = new QLabel("--", safeBox);
        val->setProperty("cssClass", "readout");
        val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        paramForm->addWidget(val, row, col + 1);
        m_paramVal[i] = val;
    }
    paramForm->setColumnStretch(1, 1);
    paramForm->setColumnStretch(3, 1);
    safeV->addLayout(paramForm);

    m_paramsBtn = new QPushButton("编辑控制参数…", safeBox);
    connect(m_paramsBtn, &QPushButton::clicked, this, &MainWindow::onEditParams);
    safeV->addWidget(m_paramsBtn);

    v->addWidget(safeBox);
    v->addStretch();
    return w;
}

// ═══════════════════════════════════════════════
// 右栏：图表 + 通信卡片
// ═══════════════════════════════════════════════

QWidget *MainWindow::buildRightPanel()
{
    auto *w = new QWidget(this);
    auto *v = new QVBoxLayout(w);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(8);

    m_chartPos = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Position, w);
    m_chartRot = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Rotation, w);
    v->addWidget(m_chartPos, 2);
    v->addWidget(m_chartRot, 2);

    m_commCards = new CommCards(w);
    v->addWidget(m_commCards);

    return w;
}

// ═══════════════════════════════════════════════
// 连接控制
// ═══════════════════════════════════════════════

void MainWindow::updateConnControls()
{
    if (m_ipEdit)      m_ipEdit->setEnabled(!m_listening);
    if (m_portSpin)    m_portSpin->setEnabled(!m_listening);
    if (m_listenBtn)   m_listenBtn->setEnabled(!m_listening);
    if (m_unlistenBtn) m_unlistenBtn->setEnabled(m_listening);
}

void MainWindow::onStartListening()
{
    if (!m_worker || m_listening) return;
    m_cfg.listenIp   = m_ipEdit->text().trimmed();
    m_cfg.listenPort = quint16(m_portSpin->value());
    if (m_interlockLabel) m_interlockLabel->hide();
    QMetaObject::invokeMethod(m_worker, "applyConfig", Qt::QueuedConnection,
                              Q_ARG(AppConfig, m_cfg));
    QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection);
}

void MainWindow::onStopListening()
{
    if (!m_worker) return;
    QMetaObject::invokeMethod(m_worker, "stop", Qt::QueuedConnection);
    m_listening = false;
    updateConnControls();
}

// ═══════════════════════════════════════════════
// 控制参数对话框
// ═══════════════════════════════════════════════

void MainWindow::onEditParams()
{
    QDialog dlg(this);
    dlg.setWindowTitle("编辑控制参数");
    dlg.setMinimumWidth(420);

    auto *form = new QFormLayout(&dlg);
    form->setSpacing(8);
    auto *kpP  = new QDoubleSpinBox; kpP->setRange(0.0, 100.0);  kpP->setDecimals(3);
    auto *kpR  = new QDoubleSpinBox; kpR->setRange(0.0, 100.0);  kpR->setDecimals(3);
    auto *vP   = new QDoubleSpinBox; vP->setRange(0.0, 10000.0); vP->setSuffix(" mm/s");
    auto *vR   = new QDoubleSpinBox; vR->setRange(0.0, 10000.0); vR->setSuffix(" deg/s");
    auto *alP  = new QDoubleSpinBox; alP->setRange(0.0, 10000.0); alP->setSuffix(" mm");
    auto *alR  = new QDoubleSpinBox; alR->setRange(0.0, 10000.0); alR->setSuffix(" deg");
    kpP->setValue(m_cfg.kpPos);  kpR->setValue(m_cfg.kpRot);
    vP->setValue(m_cfg.vmaxPosMmS);  vR->setValue(m_cfg.vmaxRotDegS);
    alP->setValue(m_cfg.accumLimitPosMm);  alR->setValue(m_cfg.accumLimitRotDeg);

    form->addRow("Kp 位置 (0.01–100)", kpP);
    form->addRow("Kp 姿态 (0.01–100)", kpR);
    form->addRow("限速位置 (1–10000 mm/s)", vP);
    form->addRow("限速姿态 (1–10000 deg/s)", vR);
    form->addRow("累积上限位置 (1–10000 mm)", alP);
    form->addRow("累积上限姿态 (1–10000 deg)", alR);

    const bool locked = m_state.snapshot().state == ControlState::Tracking;
    const std::array<QWidget *, 6> fields = {kpP, kpR, vP, vR, alP, alR};
    for (QWidget *f : fields) f->setEnabled(!locked);
    if (locked) {
        auto *note = new QLabel("运行中：参数已锁定。停止跟踪后可修改。", &dlg);
        note->setStyleSheet("color: #D97706; font-weight: bold;");
        form->addRow(note);
    }

    auto *btns = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel | QDialogButtonBox::RestoreDefaults, &dlg);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    auto *resetBtn = btns->button(QDialogButtonBox::RestoreDefaults);
    connect(resetBtn, &QPushButton::clicked, &dlg, [&] {
        AppConfig def = AppConfig::defaults();
        kpP->setValue(def.kpPos); kpR->setValue(def.kpRot);
        vP->setValue(def.vmaxPosMmS); vR->setValue(def.vmaxRotDegS);
        alP->setValue(def.accumLimitPosMm); alR->setValue(def.accumLimitRotDeg);
    });
    form->addRow(btns);

    if (dlg.exec() != QDialog::Accepted) return;
    AppConfig c2 = m_cfg;
    c2.kpPos = kpP->value();  c2.kpRot = kpR->value();
    c2.vmaxPosMmS = vP->value();  c2.vmaxRotDegS = vR->value();
    c2.accumLimitPosMm = alP->value();  c2.accumLimitRotDeg = alR->value();
    m_cfg = c2;
    QMetaObject::invokeMethod(m_worker, "applyConfig", Qt::QueuedConnection,
                              Q_ARG(AppConfig, c2));
}

// ═══════════════════════════════════════════════
// 目标快照（撤销用）
// ═══════════════════════════════════════════════

void MainWindow::saveTargetSnapshot()
{
    for (int i = 0; i < 6; ++i)
        m_appliedTarget[i] = m_targetSpin[i]->value();
    m_targetApplied = true;
}

void MainWindow::restoreTargetSnapshot()
{
    m_suppressTargetSignal = true;
    for (int i = 0; i < 6; ++i)
        m_targetSpin[i]->setValue(m_appliedTarget[i]);
    m_suppressTargetSignal = false;
    m_targetApplied = true;
}

// ═══════════════════════════════════════════════
// 目标编辑
// ═══════════════════════════════════════════════

void MainWindow::onTargetEdited()
{
    m_targetApplied = false;
    const StatusSnapshot s = m_state.snapshot();
    // 差值预览
    QString delta;
    QTextStream ds(&delta);
    ds << "差值预览（目标 − 当前）：";
    for (int i = 0; i < 6; ++i) {
        const double d = m_targetSpin[i]->value() - *(&s.actual.x + i);
        if (std::fabs(d) > 0.005) {
            ds << kAxisName[i] << "=" << QString::number(d, 'f', (i < 3) ? 1 : 2)
               << kAxisUnit(i) << "  ";
        }
    }
    m_deltaPreview->setText(delta.isEmpty() ? "差值预览：无偏差" : delta);
}

void MainWindow::onApplyTarget()
{
    if (!m_worker) return;
    Pose t;
    t.x = m_targetSpin[0]->value(); t.y = m_targetSpin[1]->value();
    t.z = m_targetSpin[2]->value(); t.a = m_targetSpin[3]->value();
    t.b = m_targetSpin[4]->value(); t.c = m_targetSpin[5]->value();
    QMetaObject::invokeMethod(m_worker, "applyTarget", Qt::QueuedConnection,
                              Q_ARG(Pose, t));
    saveTargetSnapshot();
    m_deltaPreview->setStyleSheet(
        "font-family: Consolas, monospace; font-size: 9px; color: #16A34A; "
        "padding: 4px 8px; background-color: #F0FDF4; border-radius: 4px;");
    m_deltaPreview->setText("✓ 目标已应用");
    m_alarmLog->addEvent(AlarmLog::Info, "目标位姿已更新", "");
}

void MainWindow::onUndoTarget()
{
    restoreTargetSnapshot();
    m_deltaPreview->setStyleSheet(
        "font-family: Consolas, monospace; font-size: 9px; color: #64748B; "
        "padding: 4px 8px; background-color: #F3F4F6; border-radius: 4px;");
    m_deltaPreview->setText("已撤销未应用的修改");
}

// ═══════════════════════════════════════════════
// 跟踪控制
// ═══════════════════════════════════════════════

void MainWindow::onZeroToActual()
{
    if (!m_worker) return;
    QMetaObject::invokeMethod(m_worker, "resetToActual", Qt::QueuedConnection);
    const Pose a = m_state.snapshot().actual;
    m_suppressTargetSignal = true;
    const double *v = &a.x;
    for (int i = 0; i < 6; ++i) m_targetSpin[i]->setValue(v[i]);
    m_suppressTargetSignal = false;
    onApplyTarget();
}

void MainWindow::onReadActualTarget()
{
    const Pose a = m_state.snapshot().actual;
    m_suppressTargetSignal = true;
    const double *v = &a.x;
    for (int i = 0; i < 6; ++i) m_targetSpin[i]->setValue(v[i]);
    m_suppressTargetSignal = false;
    saveTargetSnapshot();
    m_deltaPreview->setText("已读取当前实际位姿；尚未发送新的跟踪目标");
}

void MainWindow::onPrepareTracking()
{
    if (!m_worker) return;
    const StatusSnapshot s = m_state.snapshot();

    if (s.state == ControlState::Fault) return;

    // 周期检查：至少 5 帧后才上报实测值，避免前几帧噪声误拦。
    // 用均值（cycleMeanMs）而非单次快照（measuredCycleMs），更稳定。
    const int    kMinSamples = 5;
    const double cycleMs    = (s.frameCount >= kMinSamples) ? s.cycleMeanMs : 0.0;
    const QStringList blocked = SessionGuard::enableChecks(m_cfg, cycleMs);
    if (!blocked.isEmpty()) {
        m_interlockLabel->setText("使能被拦截：" + blocked.join("; "));
        m_interlockLabel->show();
        return;
    }
    m_interlockLabel->hide();

    const Pose t = s.target;
    const QMessageBox::StandardButton r = QMessageBox::question(
        this, "确认使能跟踪",
        QStringLiteral(
            "请确认以下全部项：\n\n"
            "1. 示教器 BASE / TOOL 设置正确\n"
            "2. 目标位姿: X%1 Y%2 Z%3 A%4 B%5 C%6\n"
            "3. 最大速度: 位置 %7 mm/s  姿态 %8 deg/s\n"
            "4. 累积余量: 位置 %9/%10 mm  姿态 %11/%12 deg\n\n"
            "继续使能跟踪？")
            .arg(t.x, 0, 'f', 1).arg(t.y, 0, 'f', 1).arg(t.z, 0, 'f', 1)
            .arg(t.a, 0, 'f', 2).arg(t.b, 0, 'f', 2).arg(t.c, 0, 'f', 2)
            .arg(m_cfg.vmaxPosMmS, 0, 'f', 1).arg(m_cfg.vmaxRotDegS, 0, 'f', 1)
            .arg(std::max({std::fabs(s.accum.x),std::fabs(s.accum.y),std::fabs(s.accum.z)}), 0, 'f', 1)
            .arg(m_cfg.accumLimitPosMm, 0, 'f', 1)
            .arg(std::max({std::fabs(s.accum.a),std::fabs(s.accum.b),std::fabs(s.accum.c)}), 0, 'f', 1)
            .arg(m_cfg.accumLimitRotDeg, 0, 'f', 1),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (r != QMessageBox::Yes) return;

    QMetaObject::invokeMethod(m_worker, "setTracking", Qt::QueuedConnection,
                              Q_ARG(bool, true));
    m_alarmLog->addEvent(AlarmLog::Info, "跟踪已使能", "");
}

void MainWindow::onResetFault()
{
    if (!m_worker) return;
    QMetaObject::invokeMethod(m_worker, "resetToActual", Qt::QueuedConnection);
    m_alarmLog->addEvent(AlarmLog::Warning, "故障复位请求已发送", "请确认通信与累计修正均恢复正常");
}

void MainWindow::onStopTracking()
{
    onZeroToActual();
    m_alarmLog->addEvent(AlarmLog::Warning, "软停止跟踪", "RKorr=0，RSI 回包保持");
}

// ═══════════════════════════════════════════════
// 定时刷新
// ═══════════════════════════════════════════════

void MainWindow::onRefresh()
{
    const StatusSnapshot s = m_state.snapshot();

    // ── 状态栏 ──
    m_statusBar->updateFrom(s, m_listening);
    const bool isFault = (s.state == ControlState::Fault) || s.accumOverLimit;
    m_statusBar->setWarning(
        isFault ? "故障：跟踪已停止！检查累计修正或通信状态。紧急情况请按示教器物理急停。"
                : "软停止：RKorr=0，RSI 回包保持。不是急停。紧急情况请按示教器物理急停。",
        isFault);

    // ── 中栏位姿对比 ──
    const double act[6] = {s.actual.x, s.actual.y, s.actual.z,
                           s.actual.a, s.actual.b, s.actual.c};
    const double err[6] = {s.error.x, s.error.y, s.error.z,
                           s.error.a, s.error.b, s.error.c};
    const double tgt[6] = {s.target.x, s.target.y, s.target.z,
                           s.target.a, s.target.b, s.target.c};
    const char *units[6] = {" mm", " mm", " mm", " deg", " deg", " deg"};

    for (int i = 0; i < 6; ++i) {
        // 当前实际
        m_actualItem[i]->setText(QStringLiteral("%1%2").arg(act[i], 0, 'f', 3).arg(units[i]));
        // 实时误差（着色）
        const double ep = (i < 3) ? s.errorPosPct : s.errorRotPct;
        const QColor errColor = (ep >= 1.0)   ? QColor("#DC2626")
                                : (ep >= 0.8) ? QColor("#D97706")
                                : (ep >= 0.5) ? QColor("#D97706")
                                :               QColor("#16A34A");
        m_errorItem[i]->setText(QStringLiteral("%1%2").arg(err[i], 0, 'f', 3).arg(units[i]));
        m_errorItem[i]->setForeground(errColor);
        // 目标值
        m_targetItem[i]->setText(QStringLiteral("%1%2").arg(tgt[i], 0, 'f', 3).arg(units[i]));
        // 左栏当前值
        m_liveLabel[i]->setText(QStringLiteral("%1%2").arg(act[i], 0, 'f', 3).arg(units[i]));
    }

    // ── 安全限制 ──
    m_cumulBar->updateFrom(s.accum, m_cfg.accumLimitPosMm, m_cfg.accumLimitRotDeg);

    // ── 通信卡片 ──
    m_commCards->updateFrom(s, m_cfg.cycleMs);

    // ── 控制参数只读 ──
    m_paramVal[0]->setText(QString::number(m_cfg.kpPos, 'f', 3));
    m_paramVal[1]->setText(QString::number(m_cfg.kpRot, 'f', 3));
    m_paramVal[2]->setText(QStringLiteral("%1 mm/s").arg(m_cfg.vmaxPosMmS, 0, 'f', 1));
    m_paramVal[3]->setText(QStringLiteral("%1 deg/s").arg(m_cfg.vmaxRotDegS, 0, 'f', 1));
    m_paramVal[4]->setText(QStringLiteral("%1 mm").arg(m_cfg.accumLimitPosMm, 0, 'f', 1));
    m_paramVal[5]->setText(QStringLiteral("%1 deg").arg(m_cfg.accumLimitRotDeg, 0, 'f', 1));

    // ── 使能按钮状态机 ──
    if (s.state == ControlState::Fault) {
        m_resetFaultBtn->setEnabled(true);
        m_enableBtn->setEnabled(false);
    } else if (s.state == ControlState::Tracking) {
        m_resetFaultBtn->setEnabled(false);
        m_enableBtn->setText("已使能跟踪");
        m_enableBtn->setEnabled(false);
    } else if (s.state == ControlState::Ready) {
        m_resetFaultBtn->setEnabled(false);
        m_enableBtn->setText("使能跟踪");
        m_enableBtn->setEnabled(true);
    } else {
        m_resetFaultBtn->setEnabled(false);
        m_enableBtn->setText("使能跟踪");
        m_enableBtn->setEnabled(false);
    }
    m_stopBtn->setEnabled(s.state == ControlState::Tracking);

    // ── 事件日志 ──
    if (s.accumOverLimit && s.state == ControlState::Tracking) {
        m_alarmLog->addEvent(AlarmLog::Fault,
            QStringLiteral("累计修正超限 位置 %1% 姿态 %2%")
                .arg(int(s.accumPosPct*100)).arg(int(s.accumRotPct*100)),
            "请停止跟踪并归零复位");
    }
    if (s.missedCount > 0 && s.connected) {
        m_alarmLog->addEvent(AlarmLog::Warning,
            QStringLiteral("连续丢包 %1").arg(s.missedCount),
            "检查网络或 KRC 周期");
    }

    // ── 图表 ──
    m_chartPos->updateFrom(m_ring);
    m_chartRot->updateFrom(m_ring);
}
