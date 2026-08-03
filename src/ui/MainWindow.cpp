#include "ui/MainWindow.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QFontMetrics>
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
#include <algorithm>
#include <cmath>
#include "core/SessionGuard.h"
#include "ui/AlarmLog.h"
#include "ui/CommCards.h"
#include "ui/CumulativeBar.h"
#include "ui/ErrorChart.h"
#include "ui/StatusBar.h"
#include "ui/UiLogic.h"

namespace {

const char *kAxisName[6] = {"X", "Y", "Z", "A", "B", "C"};
const char *kAxisUnit(int i) { return i < 3 ? " mm" : " deg"; }
double axisMin(int i) { return i < 3 ? -4000.0 : -180.0; }
double axisMax(int i) { return i < 3 ?  4000.0 :  180.0; }

// 控制参数只读行的唯一真源。标签、字段、单位、小数位写在同一行，
// 建面板与刷新数值都从这里取——原缺陷是标签坐标和数值坐标各写一套，
// 六个里错了五个。仅把标签与数值成对创建只堵住一半：只要 onRefresh
// 里「第 i 个数值读哪个字段」还是手写六行，重排本表就会静默复现同一
// 个错位，而且这次隔着两个函数更难看出来。成员指针让绑定跨函数成立。
struct ParamRow {
    const char        *label;
    double AppConfig::*field;
    const char        *unit;
    int                decimals;
};
const ParamRow kParams[6] = {
    {"Kp 位置",       &AppConfig::kpPos,            "",       3},
    {"Kp 姿态",       &AppConfig::kpRot,            "",       3},
    {"限速位置",      &AppConfig::vmaxPosMmS,       " mm/s",  1},
    {"限速姿态",      &AppConfig::vmaxRotDegS,      " deg/s", 1},
    {"累积上限位置",  &AppConfig::accumLimitPosMm,  " mm",    1},
    {"累积上限姿态",  &AppConfig::accumLimitRotDeg, " deg",   1},
};

// 把表格宽度钉死在「列宽之和 + 边框」上，使 viewport 正好等于列宽之和。
// 和必须读回实际列宽，不能拿打算设的那几个数相加：QHeaderView 会把过窄的列
// 抬到 minimumSectionSize，实测左表因此比算出来的多 1px。横向滚动条是关掉的
//（见 fitTableToRows），多出来的这 1px 不会以滚动条示警，只会静默裁掉末列
// 右对齐数值的尾巴——而尾巴上正是单位。
void pinTableWidth(QTableWidget *tbl)
{
    int sum = 0;
    for (int c = 0; c < tbl->columnCount(); ++c)
        sum += tbl->columnWidth(c);
    tbl->setFixedWidth(sum + 2 * tbl->frameWidth());
}

} // namespace

void MainWindow::fitTableToRows(QTableWidget *tbl, int rows)
{
    tbl->resizeRowsToContents();
    int h = tbl->horizontalHeader()->height() + 2 * tbl->frameWidth();
    for (int r = 0; r < rows; ++r)
        h += tbl->rowHeight(r);
    tbl->setFixedHeight(h);
    // 高度已按内容算准，两条滚动条都不该出现，而且必须都关掉。只关纵向的
    // 那条不够：列宽之和一旦超出 viewport（左栏实测 348 对 316），横向滚动条
    // 会从下方吃掉 17px——正好半行——末行 C 又被裁掉，缺陷 D 换个面目复现。
    tbl->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    tbl->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}

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

    // 按钮一律原生外观。原先靠动态属性 + QSS 属性选择器染成蓝/橙/红，
    // 那些属性只为选择器存在，样式表一删就是死代码。危险动作的区分改由
    // 文字（「停止跟踪」）和启用状态承担——底色在高对比度主题下本就不可信。
    m_listenBtn = new QPushButton("开始监听", this);
    connect(m_listenBtn, &QPushButton::clicked, this, &MainWindow::onStartListening);
    btnBar->addWidget(m_listenBtn);

    m_unlistenBtn = new QPushButton("停止监听", this);
    connect(m_unlistenBtn, &QPushButton::clicked, this, &MainWindow::onStopListening);
    btnBar->addWidget(m_unlistenBtn);

    btnBar->addWidget(new QLabel("→", this));

    m_resetFaultBtn = new QPushButton("复位故障", this);
    m_resetFaultBtn->setEnabled(false);
    connect(m_resetFaultBtn, &QPushButton::clicked, this, &MainWindow::onResetFault);
    btnBar->addWidget(m_resetFaultBtn);

    m_enableBtn = new QPushButton("使能跟踪", this);
    m_enableBtn->setEnabled(false);
    connect(m_enableBtn, &QPushButton::clicked, this, &MainWindow::onPrepareTracking);
    btnBar->addWidget(m_enableBtn);

    m_stopBtn = new QPushButton("停止跟踪", this);
    connect(m_stopBtn, &QPushButton::clicked, this, &MainWindow::onStopTracking);
    btnBar->addWidget(m_stopBtn);

    btnBar->addStretch();

    outer->addLayout(btnBar);

    // 联锁拦截。红字 + 加粗 + 原生边框：文字色用 severityColor(Fault)，
    // 背景不动——写死的浅红底会盖掉操作员设的高对比度主题，而这条恰恰是
    // 最不该看不清的一行。
    m_interlockLabel = new QLabel(this);
    {
        QPalette p = m_interlockLabel->palette();
        p.setColor(QPalette::WindowText,
                   uilogic::severityColor(uilogic::Severity::Fault));
        m_interlockLabel->setPalette(p);
        QFont f = m_interlockLabel->font();
        f.setBold(true);
        m_interlockLabel->setFont(f);
    }
    m_interlockLabel->setFrameShape(QFrame::StyledPanel);
    m_interlockLabel->setWordWrap(true);
    m_interlockLabel->hide();
    outer->addWidget(m_interlockLabel);

    // ── 三栏主体 ──
    auto *body = new QHBoxLayout;
    body->setSpacing(12);

    // 两栏宽度都由各自表格的列宽反推，而不是先定 360/420 再硬塞列进去：
    // 后者只能靠缩字号或裁尾巴收场，而这两张表存在的理由就是把数字看全。
    // 两张表在 build* 里已按列宽之和 setFixedWidth，所以这里取 sizeHint 即可，
    // 不再写「+34」那种实测常数：那 34 是 QSS 给 QGroupBox 的 padding 加边框，
    // 样式表一删就不再成立，而算少一像素就是末列被裁（横向滚动条已关掉，
    // 超宽不会以滚动条示警，直接吃掉尾巴上的单位）。
    auto *leftPanel  = buildLeftPanel();
    leftPanel->setFixedWidth(leftPanel->sizeHint().width());
    body->addWidget(leftPanel);

    auto *midPanel   = buildMidPanel();
    midPanel->setFixedWidth(midPanel->sizeHint().width());
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
        refreshDeltaPreview();
        updateApplyButton();
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
    // 初始 m_targetApplied 为真，「应用目标」应当一开始就是灰的：还没改过
    // 任何值就允许点，等于允许把开机默认的全零目标发给机器人。
    updateApplyButton();
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
    // 与中栏 poseBox 用同一组边距，两栏的「面板宽 − 表宽」之差才是同一个
    // 常数（kPanelChrome）。默认的 9px 边距会让左栏比中栏多吃 10px，
    // 表就比 viewport 宽 10px，横向滚动条随之出现。
    tgtV->setContentsMargins(4, 4, 4, 4);
    tgtV->setSpacing(4);

    auto *tbl = new QTableWidget(6, 4, tgtBox);
    tbl->setHorizontalHeaderLabels({"轴", "目标值", "当前值（只读）", "调整"});
    tbl->verticalHeader()->setVisible(false);
    tbl->setShowGrid(false);
    tbl->setSelectionMode(QAbstractItemView::NoSelection);
    tbl->setEditTriggers(QAbstractItemView::NoEditTriggers);
    // 列宽全部显式给出，不靠 stretchLastSection：实测它并没有把末列收到
    // 剩余宽度（末列停在 defaultSectionSize=100，四列合计 338 > viewport 316），
    // 于是横向滚动条常驻并从下方吃掉半行，把第六行 C 压出可视区。
    // 「调整」列只装两个 22px 按钮加 2px 间距，60px 够用且留出余量。
    // 字号不再点名 10：删掉 QSS 后系统字号说了算，写死 10 会让读数比周围的
    // 标签小一号，而这几列正是要最先被看清的。
    const QFont tgtNumF = uilogic::monospaceFont();
    const QFontMetrics tgtFm(tgtNumF);
    // 当前值列要放得下最宽的合法读数 "-4000.000 mm"；目标值列是 QDoubleSpinBox，
    // 还要额外留出上下箭头的宽度（spinbox 有自己的边框，比表格单元多吃 8px）。
    const int wLive = tgtFm.horizontalAdvance("-4000.000 mm") + 14;
    const int wSpin = wLive + 30;
    const int wAxis = tgtFm.horizontalAdvance("W") + 16;
    // +/- 是方形小按钮。边长跟着字高走：写死 22 是配合 QSS 里 12px 字号的，
    // 回到系统字号后 "+" 会顶到边框上。
    const int kStepBtn = std::max(22, tgtFm.height() + 6);
    const int wStep = 2 * kStepBtn + 10;
    tbl->horizontalHeader()->setStretchLastSection(false);
    tbl->setColumnWidth(0, wAxis);
    tbl->setColumnWidth(1, wSpin);
    tbl->setColumnWidth(2, wLive);
    tbl->setColumnWidth(3, wStep);
    // 表格自己钉住宽度，面板宽度再由它反推（见构造函数）。宽度不再存成成员：
    // 面板已经能从 sizeHint 拿到，存一份只会多一个「写了没人读」的字段。
    pinTableWidth(tbl);

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
        // 等宽字体让六个输入框里的小数点成列；边框和聚焦高亮交回给系统主题，
        // 原来那两条选择器画的浅灰边框在深色主题下会消失。
        sp->setFont(tgtNumF);
        tbl->setCellWidget(i, 1, sp);
        m_targetSpin[i] = sp;

        // 当前值 label
        // 只读读数：等宽 + 右对齐就够了。原先那块浅灰底 + 圆角让它看起来像
        // 一个可以输入的框，而它恰恰是不可编辑的那一列。
        auto *live = new QLabel("--", tbl);
        live->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        live->setFont(tgtNumF);
        tbl->setCellWidget(i, 2, live);
        m_liveLabel[i] = live;

        // +/- 按钮
        auto *btnW = new QWidget(tbl);
        auto *btnL = new QHBoxLayout(btnW);
        btnL->setContentsMargins(0, 0, 0, 0);
        btnL->setSpacing(2);
        auto *minus = new QPushButton("−", btnW);
        minus->setFixedSize(kStepBtn, kStepBtn);
        btnL->addWidget(minus);
        m_stepMinus[i] = minus;
        auto *plus = new QPushButton("+", btnW);
        plus->setFixedSize(kStepBtn, kStepBtn);
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
            // m_suppressTargetSignal 此前只写不读（四处置位全部无效），于是
            // 「程序改值」与「操作员改值」走同一条路径。四个置位点末尾都会
            // 自己把 m_targetApplied 摆正，所以旧代码结果上没错——但这个
            // 保护栏形同虚设，下一个依赖它的改动就会踩空。读它一次，让
            // 「抑制」真的抑制。
            if (m_suppressTargetSignal) return;
            onTargetEdited();
        });
    }
    fitTableToRows(tbl, 6);
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

    // 差值预览。原生边框代替浅黄底：它列的是「目标 − 当前」的逐轴数字，
    // 等宽字体让这些数字成列比底色更要紧；而黄底暗示「警告」，实际上
    // 有偏差是编辑目标后的常态，不是异常。
    m_deltaPreview = new QLabel("差值预览：编辑目标值后此处显示当前偏差", tgtBox);
    m_deltaPreview->setFont(uilogic::monospaceFont());
    m_deltaPreview->setFrameShape(QFrame::StyledPanel);
    m_deltaPreview->setWordWrap(true);
    tgtV->addWidget(m_deltaPreview);

    // 操作按钮
    auto *actRow = new QHBoxLayout;
    auto *setCur = new QPushButton("读取当前值", tgtBox);
    connect(setCur, &QPushButton::clicked, this, &MainWindow::onReadActualTarget);
    actRow->addWidget(setCur);
    // 「应用目标」不再涂成蓝底白字：强调交给下面的 setDefault，那是平台自带的
    // 默认按钮外观，操作员在别的 Windows 程序里已经认得它。
    auto *apply = new QPushButton("应用目标", tgtBox);
    // setDefault 只在按钮的 autoDefault 打开时生效（QPushButton 在非对话框
    // 父窗口里默认关闭）。updateApplyButton 靠 setDefault 表达「改了没发」，
    // 少了这行它就是一次静默的空操作。
    apply->setAutoDefault(true);
    connect(apply, &QPushButton::clicked, this, &MainWindow::onApplyTarget);
    actRow->addWidget(apply);
    m_applyBtn = apply;
    auto *undo = new QPushButton("撤销修改", tgtBox);
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
    // 误差列的表头不叫「实时误差」：那三个字暗示它与左右两列同构，而后三行
    // 装的是 SO(3) 最短旋转在世界坐标轴上的分量，不是 A/B/C 三个欧拉角之差。
    // 现场已经因此报过一次假 bug（详见 uilogic::errorColumnTooltip）。
    tbl->setHorizontalHeaderLabels(
        {"轴", "当前实际", "误差（旋转向量）", "目标位姿", "RKorr 输出"});
    tbl->horizontalHeaderItem(2)->setToolTip(uilogic::errorColumnTooltip());
    tbl->verticalHeader()->setVisible(false);
    tbl->setShowGrid(false);
    tbl->setSelectionMode(QAbstractItemView::NoSelection);
    tbl->setEditTriggers(QAbstractItemView::NoEditTriggers);
    // 列宽从字体度量算出来，不写死像素：数值列必须放得下最宽的合法读数
    // "-4000.000 mm"（轴行程上限，实测 94px），写死 86 时 1804.000 这种
    // 四位坐标会换行成两行，把整表撑高又顶出滚动条。
    // 同时 stretchLastSection 必须关掉——开着时最后一列的 setColumnWidth
    // 不生效。约束只有一条：五列之和等于 viewport，多一像素就出横向滚动条，
    // 而右对齐数值的尾巴（单位）正好落在被截掉的那一段里。
    // 注：col4 原先的 100 不是被表头文字「RKorr 输出」顶出来的（实测
    // minimumSectionSize=28，表头文字只需 47px），它只是 Qt 的
    // defaultSectionSize，改小完全生效——别去找那个不存在的约束。
    const QFont numF = uilogic::monospaceFont();
    const QFontMetrics numFm(numF);
    // 单元格左右留白。原来是样式表里的 padding 0px 6px，删掉样式表后由这个
    // 余量顶上——原生 item 只有一两像素的边距，不留出来数字会贴着列线。
    const int kCellPad = 14;
    const int kAxisCol = numFm.horizontalAdvance("W") + 16;
    // 位姿 / 目标：最宽读数是行程上限；RKorr 每帧增量受限速约束，
    // 量级远小（vmax 13mm/s × 12ms ≈ 0.16mm），按自己的最宽串单独算。
    const int wData  = numFm.horizontalAdvance("-4000.000 mm") + kCellPad;
    const int wRkorr = numFm.horizontalAdvance("-0.1560 deg") + kCellPad;
    // 误差列比数值列宽：姿态三行带 Rx/Ry/Rz 前缀（见 uilogic::formatError），
    // 最宽串是旋转分量取满量程的 "Rz -180.000 deg"，比位置行的行程上限还长。
    // 不按前缀重算这一列，尾巴上的 "deg" 会被裁掉——而单位正是这列要说清的
    // 事情之一。表头文字也一并纳入：它比原来长，虽然目前仍窄于数据串，
    // 但改字号时不该靠人记得回来手算。
    const QFontMetrics hdrFm(tbl->horizontalHeader()->font());
    const int wError = std::max({
        numFm.horizontalAdvance("-4000.000 mm") + kCellPad,
        numFm.horizontalAdvance("Rz -180.000 deg") + kCellPad,
        hdrFm.horizontalAdvance("误差（旋转向量）") + kCellPad});
    tbl->horizontalHeader()->setStretchLastSection(false);
    tbl->setColumnWidth(0, kAxisCol);
    tbl->setColumnWidth(1, wData);
    tbl->setColumnWidth(2, wError);
    tbl->setColumnWidth(3, wData);
    tbl->setColumnWidth(4, wRkorr);
    // 面板宽度反过来由列宽决定，而不是先定 420 再硬塞五列进去：
    // 后者只能靠缩字号或裁尾巴收场，而这张表存在的理由就是把数字看全。
    // 表格自己钉住宽度，面板宽度再由它反推（见构造函数）。
    pinTableWidth(tbl);
    // 六行必须一屏放下：默认行高下只露出 X/Y/Z/A，B 与 C 被挤到滚动条以外，
    // 轴名从「看不见」变成「要滚动才看得见」，对操作员是同一个问题。
    // 高度交给 fitTableToRows 按实测行高算（见其声明处的理由）。
    tbl->verticalHeader()->setDefaultSectionSize(22);

    for (int i = 0; i < 6; ++i) {
        auto *ax = new QTableWidgetItem(kAxisName[i]);
        ax->setFlags(Qt::NoItemFlags);
        ax->setTextAlignment(Qt::AlignCenter);
        auto axF = ax->font(); axF.setBold(true); ax->setFont(axF);
        tbl->setItem(i, 0, ax);

        // 读数列一律用系统等宽字体（numF，见上方列宽计算）。原先四列各写一次
        // setFamily("Consolas")：QFont 找不到该族不会报错，只会静默回退成比例
        // 字体，右对齐还在但小数点不再成列——而成列正是这几列存在的理由。

        // 当前实际
        auto *act = new QTableWidgetItem("--");
        act->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        act->setFlags(Qt::NoItemFlags);
        auto actF = numF; actF.setBold(true); act->setFont(actF);
        // 不设前景色：这一列没有语义，写死深灰会在深色主题下变成黑底黑字。
        tbl->setItem(i, 1, act);
        m_actualItem[i] = act;

        // 误差
        auto *err = new QTableWidgetItem("--");
        err->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        err->setFlags(Qt::NoItemFlags);
        auto errF = numF; errF.setBold(true); err->setFont(errF);
        // 首帧到来前是 "--"，属于「无数据」这一档；有数据后由 onRefresh 按
        // 误差占比改成 Ok/Warn/Fault。
        err->setForeground(uilogic::severityColor(uilogic::Severity::Idle));
        // 姿态三行额外挂 tooltip：单元格里的 Rx/Ry/Rz 前缀足以拦住误读，
        // 但拦住之后操作员会问「那这到底是什么」，答案要在原地拿得到。
        // 位置三行的 tooltip 是空串（见 errorCellTooltip），不设。
        const QString errTip = uilogic::errorCellTooltip(i);
        if (!errTip.isEmpty())
            err->setToolTip(errTip);
        tbl->setItem(i, 2, err);
        m_errorItem[i] = err;

        // 目标（只显示，不编辑——编辑在左栏）
        auto *tgt = new QTableWidgetItem("--");
        tgt->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        tgt->setFlags(Qt::NoItemFlags);
        tgt->setFont(numF);
        tbl->setItem(i, 3, tgt);
        m_targetItem[i] = tgt;

        // RKorr 输出：本帧实际发给 KRC 的增量，与左边三列并排才有意义——
        // 「误差这么大，主机到底发了多少出去」是同一眼要回答的问题。
        auto *rk = new QTableWidgetItem("--");
        rk->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        rk->setFlags(Qt::NoItemFlags);
        rk->setFont(numF);
        tbl->setItem(i, 4, rk);
        m_rkorrItem[i] = rk;
    }
    fitTableToRows(tbl, 6);
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

    // 控制参数只读行：标签与数值在同一次循环里成对创建，
    // 行列由同一个下标算出，错位在结构上不可能发生（表见文件顶部 kParams）。
    auto *paramForm = new QGridLayout;
    paramForm->setHorizontalSpacing(12);
    paramForm->setVerticalSpacing(4);
    for (int i = 0; i < 6; ++i) {
        const int row = i / 2;
        const int col = (i % 2) * 2;
        auto *name = new QLabel(kParams[i].label, safeBox);
        paramForm->addWidget(name, row, col);

        // 只读数值：等宽 + 右对齐，六个参数的小数点成列才能一眼看出量级差异。
        // 原先靠动态属性让 QSS 的 readout 规则挑中它，属性一删就得自己带字体。
        auto *val = new QLabel("--", safeBox);
        val->setFont(uilogic::monospaceFont());
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
    // 与 onRefresh 用同一个 buttonStates，避免两处各写一套判定后逐渐分叉。
    // 这个函数仍然需要：构造期还没有第一帧快照、bindFailed 回调也要在
    // 下一次刷新到来之前立刻把按钮改回去。
    const ButtonStates b = uilogic::buttonStates(m_state.snapshot(), m_listening);
    if (m_ipEdit)      m_ipEdit->setEnabled(b.connEditable);
    if (m_portSpin)    m_portSpin->setEnabled(b.connEditable);
    if (m_listenBtn)   m_listenBtn->setEnabled(b.startListen);
    if (m_unlistenBtn) m_unlistenBtn->setEnabled(b.stopListen);
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
    // socket 一关就不可能再收发帧，跟踪也就无从谈起。不取消的话控制器
    // 状态会停在 Tracking，状态卡片持续显示「跟踪中」——覆盖一台已经
    // 断开的机器，这是最坏的一类显示错误。
    QMetaObject::invokeMethod(m_worker, "setTracking", Qt::QueuedConnection,
                              Q_ARG(bool, false));
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
        QPalette notePal = note->palette();
        notePal.setColor(QPalette::WindowText,
                         uilogic::severityColor(uilogic::Severity::Warn));
        note->setPalette(notePal);
        QFont noteF = note->font();
        noteF.setBold(true);
        note->setFont(noteF);
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

void MainWindow::refreshDeltaPreview()
{
    double tgt[6];
    for (int i = 0; i < 6; ++i)
        tgt[i] = m_targetSpin[i]->value();
    m_deltaPreview->setText(
        uilogic::deltaPreview(tgt, m_state.snapshot().actual));
}

void MainWindow::updateApplyButton()
{
    // m_targetApplied 原先只写不读：目标改了没发，界面毫无提示。
    // 用它驱动按钮的默认态，「改了没发」一眼可见。
    // 用 setDefault 而不是配色：原生按钮的「默认按钮」有平台自带的
    // 视觉强调（Windows 上是高亮边框），且回车键会触发它——正是
    // 「刚改完值，按回车发出去」这个动作。
    m_applyBtn->setDefault(!m_targetApplied);
    m_applyBtn->setEnabled(!m_targetApplied);
}

void MainWindow::onTargetEdited()
{
    m_targetApplied = false;
    refreshDeltaPreview();
    updateApplyButton();
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
    m_alarmLog->addEvent(AlarmLog::Info, "目标位姿已更新", "");
    // 预览按实际值重算，而不是写一句「✓ 目标已应用」：目标发出去之后偏差
    // 并不归零（机器人要走一段才到位），写死的文案会盖掉这个信息。
    refreshDeltaPreview();
    updateApplyButton();
}

void MainWindow::onUndoTarget()
{
    restoreTargetSnapshot();
    // restoreTargetSnapshot 抑制了 valueChanged，onTargetEdited 不会被调用，
    // 所以撤销之后必须显式重算——否则预览停留在撤销前那组值算出的偏差。
    refreshDeltaPreview();
    updateApplyButton();
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
    refreshDeltaPreview();
    updateApplyButton();
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

    for (int i = 0; i < 6; ++i) {
        // 当前实际
        m_actualItem[i]->setText(uilogic::formatValue(act[i], i));
        // 实时误差（着色）
        const double ep = (i < 3) ? s.errorPosPct : s.errorRotPct;
        // 色值来自 uilogic::severityColor 而不是就地写十六进制：原先内联样式
        // 与 QSS 各带一套色板，同一个「正常」在界面上是两种绿。
        const QColor errColor = uilogic::severityColor(
            (ep >= 1.0)   ? uilogic::Severity::Fault
            : (ep >= 0.5) ? uilogic::Severity::Warn
                          : uilogic::Severity::Ok);
        // formatError 而非 formatValue：姿态三行要带 Rx/Ry/Rz 前缀，
        // 否则它们会被行首的 A/B/C 认领（理由见 UiLogic.h 误差列一节）。
        m_errorItem[i]->setText(uilogic::formatError(err[i], i));
        m_errorItem[i]->setForeground(errColor);
        // 目标值
        m_targetItem[i]->setText(uilogic::formatValue(tgt[i], i));
        // RKorr：零值压成灰（Idle），非零恢复系统默认文字色——一眼看出哪个轴
        // 在动。非零那档刻意不再涂蓝：蓝在 Severity 里没有对应档，再引入一个
        // 色值就又是一套独立色板。零值判定与 formatRkorr 共用
        // uilogic::isRkorrZero，两处各写一个 5e-5 时改一处漏一处就会出现
        // 「显示 0.0000 却不是灰的」这种自相矛盾的格。
        const double rk = (&s.lastDelta.x)[i];
        m_rkorrItem[i]->setText(uilogic::formatRkorr(rk, i));
        if (uilogic::isRkorrZero(rk))
            m_rkorrItem[i]->setForeground(
                uilogic::severityColor(uilogic::Severity::Idle));
        else
            m_rkorrItem[i]->setData(Qt::ForegroundRole, QVariant());
        // 左栏当前值
        m_liveLabel[i]->setText(uilogic::formatValue(act[i], i));
    }

    // ── 安全限制 ──
    m_cumulBar->updateFrom(s.accum, m_cfg.accumLimitPosMm, m_cfg.accumLimitRotDeg);

    // ── 通信卡片 ──
    m_commCards->updateFrom(s, m_cfg.cycleMs);

    // ── 控制参数只读 ──
    // 字段、单位、小数位都来自建面板时用的同一张表，不再手写六行按位置对应。
    for (int i = 0; i < 6; ++i)
        m_paramVal[i]->setText(QStringLiteral("%1%2")
                                   .arg(m_cfg.*(kParams[i].field), 0, 'f', kParams[i].decimals)
                                   .arg(kParams[i].unit));

    // ── 使能按钮状态机 ──
    // 全部交给 uilogic::buttonStates，界面这边不再自己判状态。原先手写的
    // 那条 `m_stopBtn->setEnabled(s.state == Tracking)` 已整段删除：留着它
    // 会在 StaleFrame / Syncing 下继续把停止按钮置灰，而那正是最需要能停的
    // 时刻（反馈已异常、PoseController 仍在发增量）。
    const ButtonStates btn = uilogic::buttonStates(s, m_listening);
    m_resetFaultBtn->setEnabled(btn.resetFault);
    m_enableBtn->setEnabled(btn.enableTrack);
    m_enableBtn->setText(s.state == ControlState::Tracking ? "已使能跟踪"
                                                           : "使能跟踪");
    m_stopBtn->setEnabled(btn.stopTrack);
    m_listenBtn->setEnabled(btn.startListen);
    m_unlistenBtn->setEnabled(btn.stopListen);
    m_ipEdit->setEnabled(btn.connEditable);
    m_portSpin->setEnabled(btn.connEditable);

    // ── 事件日志 ──
    // 边沿触发：只在告警「发生」时记一条，而不是在它「持续」的每一帧。
    // 丢包那一路还要先经迟滞拉平：missedCount 在每个正常帧被 RsiWorker 归零，
    // 间歇丢包在快照里是 0/1/0/1 的脉冲串，每个脉冲都是货真价实的上升沿，
    // 只做边沿触发挡不住（--drop 5 实测 8.5 秒 142 条）。
    // 恢复门限取 1 秒的刷新帧数：短于一次 KRC 重传的间隔就判恢复，等于没有
    // 迟滞；长到几十秒又会把两次独立的网络故障并成一条。
    const int kLossClearFrames =
        std::max(1, int(1000.0 / std::max(1, m_cfg.refreshMs)));
    const AlarmEdge now  = uilogic::currentAlarmsHeld(m_lossHold, s, kLossClearFrames);
    const AlarmEdge edge = uilogic::edgesBetween(m_prevAlarms, now);
    if (edge.accumOverLimit)
        m_alarmLog->addEvent(AlarmLog::Fault,
            QStringLiteral("累计修正超限 位置 %1% 姿态 %2%")
                .arg(int(s.accumPosPct * 100)).arg(int(s.accumRotPct * 100)),
            "请停止跟踪并复位故障");
    if (edge.packetLoss)
        m_alarmLog->addEvent(AlarmLog::Warning,
            QStringLiteral("出现丢包，连续 %1 帧").arg(s.missedCount),
            "检查网络或 KRC 周期");
    // 存的必须是「本帧的告警电平」，不是 edgesBetween 的返回值——后者是
    // 「本帧是否为上升沿」，持续告警时它恒为 false，下一帧就又成了上升沿，
    // 刷屏原样复现。
    m_prevAlarms = now;

    // ── 图表 ──
    m_chartPos->updateFrom(m_ring);
    m_chartRot->updateFrom(m_ring);
}
