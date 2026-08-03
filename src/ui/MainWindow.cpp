#include "ui/MainWindow.h"

#include <QApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QFontMetrics>
#include <QFrame>
#include <QGridLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QHostAddress>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPalette>
#include <QScreen>
#include <QSettings>
#include <QStatusBar>
#include <QTableWidget>
#include <QToolBar>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <vector>
#include "core/SessionGuard.h"
#include "ui/AlarmLog.h"
#include "ui/CommCards.h"
#include "ui/CumulativeBar.h"
#include "ui/ErrorChart.h"
#include "ui/UiLogic.h"

namespace {

const char *kAxisName[6] = {"X", "Y", "Z", "A", "B", "C"};
const char *kAxisUnit(int i) { return i < 3 ? " mm" : " deg"; }
double axisMin(int i) { return i < 3 ? -4000.0 : -180.0; }
double axisMax(int i) { return i < 3 ?  4000.0 :  180.0; }

// QSettings 的组织名/应用名。两处（构造与析构）必须一致，写成常量而不是
// 各写一遍字面量——拼错一个字符的表现是「布局每次都不被恢复」，无任何报错。
const char *kSettingsOrg = "kuka_rsi_win";
const char *kSettingsApp = "rsi_host";

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

// 表格列宽的唯一设置入口。
//
// 为什么不再是「按列宽之和 setFixedWidth，面板宽度再由它反推」：那套做法
// 成立的前提是面板宽度由程序决定。改成 QDockWidget 之后宽度由操作员拖动，
// 前提消失——拖宽会在表格右侧留一条死白，拖窄则把列挤到 viewport 之外，
// 而横向滚动条是关掉的（见 fitTableToRows），挤出去的部分不会以滚动条示警，
// 只会静默裁掉右对齐数值尾巴上的单位。
//
// 改成两条约束一起给：
//   1) 整表一个 minimumWidth = 各列所需宽度之和 + 边框。面板拖不到比这更窄，
//      于是「六行全见、单位不裁」在任何宽度下都成立，不必再靠调数字维持。
//   2) 指定一列为 Stretch，多余宽度全归它。拖宽时空白落在这一列内部，
//      而不是落在表格外面。
//
// 求和必须读回实际列宽，不能拿打算设的那几个数相加：QHeaderView 会把过窄的
// 列抬到 minimumSectionSize（实测左表因此比算出来的多 1px），少算的那一像素
// 正是被裁掉的单位后缀。所以先设完全部列宽、读回求和，最后才切 Stretch——
// 切了之后 columnWidth(stretchCol) 返回的是拉伸后的值，求和就白算了。
void setTableColumns(QTableWidget *tbl, const std::vector<int> &want, int stretchCol)
{
    QHeaderView *h = tbl->horizontalHeader();
    // stretchLastSection 必须关掉：开着时最后一列的 setColumnWidth 不生效，
    // 而这里要按度量给每一列宽度、再自己选哪一列吃多余宽度。
    h->setStretchLastSection(false);
    for (int c = 0; c < int(want.size()); ++c)
        tbl->setColumnWidth(c, want[c]);

    int sum = 0;
    for (int c = 0; c < tbl->columnCount(); ++c)
        sum += tbl->columnWidth(c);
    tbl->setMinimumWidth(sum + 2 * tbl->frameWidth());

    h->setSectionResizeMode(stretchCol, QHeaderView::Stretch);
}

// 状态栏标签：文字 + 语义色。只在真的变了才写回，因为它每 refreshMs 调用一次，
// 无条件 setPalette 会让状态栏每 20ms 重绘一次。
void setSeverityText(QLabel *l, const QString &text, uilogic::Severity sev)
{
    if (l->text() != text)
        l->setText(text);
    const QColor fg = uilogic::severityColor(sev);
    if (l->palette().color(QPalette::WindowText) != fg) {
        QPalette p = l->palette();
        p.setColor(QPalette::WindowText, fg);
        l->setPalette(p);
    }
}

// 状态栏常驻标签的最小宽度按「最长可能文案」钉住。不钉的话读数每变一位
// 宽度就跟着变，四个标签在运行中左右跳动，而它们恰恰是要被瞥一眼的东西。
void pinLabelWidth(QLabel *l, const QString &widest)
{
    l->setMinimumWidth(QFontMetrics(l->font()).horizontalAdvance(widest) + 8);
}

// 软停止不是急停——这句必须常驻可见，不能只在故障时弹出来。
const char *kNoticeNormal =
    "软停止：RKorr=0，RSI 回包保持。不是急停。紧急情况请按示教器物理急停。";
const char *kNoticeFault =
    "故障：跟踪已停止！检查累计修正或通信状态。紧急情况请按示教器物理急停。";

} // namespace

void MainWindow::fitTableToRows(QTableWidget *tbl, int rows)
{
    tbl->resizeRowsToContents();
    int h = tbl->horizontalHeader()->height() + 2 * tbl->frameWidth();
    for (int r = 0; r < rows; ++r)
        h += tbl->rowHeight(r);
    tbl->setFixedHeight(h);
    // 高度已按内容算准，两条滚动条都不该出现，而且必须都关掉。只关纵向的
    // 那条不够：列宽之和一旦超出 viewport，横向滚动条会从下方吃掉 17px——
    // 正好半行——末行 C 又被裁掉，缺陷 D 换个面目复现。
    // 关掉横向滚动条之后，「不超出 viewport」就没有任何示警了，只能靠
    // setTableColumns 给出的 minimumWidth 从结构上保证。
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

    // ── 中央部件：误差图表 ──
    // 图表是唯一需要大面积且持续观察的东西——趋势要看一段时间才有意义。
    // 其余面板都是「看一眼确认数值」的性质，适合停靠、按需调出。
    // 原先的三栏定宽布局把图表挤在最右侧一条，1400px 以下还会把控件推出屏幕。
    auto *charts = new QWidget(this);
    auto *cv = new QVBoxLayout(charts);
    cv->setContentsMargins(0, 0, 0, 0);
    m_chartPos = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Position, charts);
    m_chartRot = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Rotation, charts);
    cv->addWidget(m_chartPos, 1);
    cv->addWidget(m_chartRot, 1);
    setCentralWidget(charts);

    m_alarmLog = new AlarmLog(this);

    // objectName 用 ASCII 而不是直接拿标题当名字：它是 saveState 二进制块里的
    // 键，改一次面板标题（比如给「目标位姿」加上 BASE 字样）就会让已保存的
    // 布局对不上号、静默丢掉该面板的位置。标题给人看，objectName 给程序认。
    auto addDock = [this](const QString &title, const char *objName, QWidget *w,
                          Qt::DockWidgetArea area) -> QDockWidget * {
        auto *d = new QDockWidget(title, this);
        d->setObjectName(QString::fromLatin1(objName));
        d->setWidget(w);
        addDockWidget(area, d);
        return d;
    };

    m_listenDock  = addDock("监听配置", "listenDock",  buildListenPanel(),  Qt::LeftDockWidgetArea);
    m_targetDock  = addDock("目标位姿 (BASE)", "targetDock", buildTargetPanel(), Qt::LeftDockWidgetArea);
    m_compareDock = addDock("位姿对比", "compareDock", buildComparePanel(), Qt::LeftDockWidgetArea);
    m_cumulDock   = addDock("累积修正", "cumulDock",   buildCumulPanel(),   Qt::RightDockWidgetArea);
    m_paramDock   = addDock("控制参数", "paramDock",   buildParamPanel(),   Qt::RightDockWidgetArea);
    m_commDock    = addDock("通信指标", "commDock",    buildCommPanel(),    Qt::RightDockWidgetArea);
    m_alarmDock   = addDock("事件日志", "alarmDock",   m_alarmLog,          Qt::BottomDockWidgetArea);
    m_alarmDock->hide();   // 默认隐藏，从「视图」菜单调出

    buildMenus();
    buildStatusBar();

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

    // 默认布局要在 restoreState 之前存：一份被拖坏的布局（面板拖到屏幕外、
    // 全部关掉）会被 QSettings 忠实保存，下次启动照样是坏的。有了这份快照，
    // 「视图 → 恢复默认布局」就能把它救回来。
    m_defaultLayout = saveState();

    // 窗口几何在构造期恢复：这一步必须早于显示，否则窗口会先按默认尺寸画一帧
    // 再跳到上次的尺寸。面板布局（restoreState）不在这里，见 showEvent。
    QSettings st(kSettingsOrg, kSettingsApp);
    restoreGeometry(st.value("geometry").toByteArray());

    updateConnControls();
    saveTargetSnapshot();
    // 初始 m_targetApplied 为真，「应用目标」应当一开始就是灰的：还没改过
    // 任何值就允许点，等于允许把开机默认的全零目标发给机器人。
    updateApplyButton();
}

std::array<QDockWidget *, 7> MainWindow::docks() const
{
    return {m_listenDock, m_targetDock, m_compareDock,
            m_cumulDock, m_paramDock, m_commDock, m_alarmDock};
}

void MainWindow::showEvent(QShowEvent *e)
{
    QMainWindow::showEvent(e);
    if (m_layoutRestored)
        return;
    m_layoutRestored = true;

    // 面板布局在首次显示时恢复，而不是在构造函数里。showEvent 早于第一次绘制，
    // 所以看不到「先按默认布局画一帧再跳」的闪动，但窗口此时已经存在——
    // 浮动面板是顶层窗口，在主窗口还不存在时摆放它并不可靠。
    // restoreState 依赖每个 dock 与 toolbar 的 objectName，都已在构造期设好。
    QSettings st(kSettingsOrg, kSettingsApp);
    restoreState(st.value("windowState").toByteArray());

    // 面板的显示/隐藏另存一份，不使用 restoreState 里的那一位。实测
    // restoreState 能把浮动面板的区域与几何恢复得分毫不差，却一律把它恢复成
    // 隐藏（float=1 而 vis=0）：操作员上次把某个面板拖出窗口，下次启动它就
    // 不见了，得自己想到去「视图」菜单里再勾一次。停靠着的面板不受影响，
    // 所以这个缺陷只在拖出过浮动窗口的机器上出现，更不容易被发现。
    //
    // 这一步必须排到事件循环的下一轮，不能就地做：浮动面板是主窗口的子顶层
    // 窗口，而 showEvent 期间主窗口本身还没真正映射出来，此时对子窗口
    // setVisible(true) 会被 Qt 压住不生效（实测 vis 仍为 0）。
    QTimer::singleShot(0, this, [this] {
        QSettings s(kSettingsOrg, kSettingsApp);
        for (QDockWidget *d : docks()) {
            // 键不存在（首次运行、或新增的面板）时不动，让默认值说话。
            const QVariant v = s.value("dockVisible/" + d->objectName());
            if (v.isValid())
                d->setVisible(v.toBool());
        }
    });
}

void MainWindow::saveLayout()
{
    m_layoutSaved = true;
    QSettings st(kSettingsOrg, kSettingsApp);
    st.setValue("geometry", saveGeometry());
    st.setValue("windowState", saveState());
    for (QDockWidget *d : docks())
        st.setValue("dockVisible/" + d->objectName(), !d->isHidden());
}

void MainWindow::closeEvent(QCloseEvent *e)
{
    // 布局必须在这里存，不能只在析构里存。关窗口的顺序是
    // closeEvent → 隐藏主窗口 → 析构，而浮动面板是主窗口的子顶层窗口，
    // 主窗口一隐藏它们就跟着被显式隐藏。到析构时再 saveState，存下来的就是
    // 「这个面板是关着的」——操作员上次把面板拖出窗口，下次启动它不见了，
    // 而且是被如实记录的，怎么调 restoreState 都救不回来。
    saveLayout();
    // 显式退进程。单靠 quitOnLastWindowClosed 是间接的：若析构函数里
    // wait(2000) 超时，通信线程可能赶不上，QUdpSocket 来不及 close，
    // 端口还被占着而新实例已经启动——两个 socket 共享同一端口不报错，
    // UDP 包被 OS 随机分配，表现就是间歇性断连重连。真正的防线是
    // main.cpp 里的单实例锁，这里是一层兜底。
    QApplication::quit();
    QMainWindow::closeEvent(e);
}

MainWindow::~MainWindow()
{
    // 兜底：程序也可能不经关窗口就退出（QApplication::quit、会话注销）。
    // closeEvent 走过就不再存，否则会用「已隐藏」的状态盖掉正确的那一份。
    if (!m_layoutSaved)
        saveLayout();

    if (m_commThread && m_commThread->isRunning()) {
        QMetaObject::invokeMethod(m_worker, "stop", Qt::BlockingQueuedConnection);
        m_commThread->quit();
        m_commThread->wait(2000);
    }
}

// ═══════════════════════════════════════════════
// 菜单 / 工具栏 / 状态栏
// ═══════════════════════════════════════════════

void MainWindow::buildMenus()
{
    // ── 监听 ──
    QMenu *listenMenu = menuBar()->addMenu("监听(&L)");
    m_startListenAct = listenMenu->addAction("开始监听");
    m_startListenAct->setObjectName("startListenAct");
    m_startListenAct->setShortcut(QKeySequence("F5"));
    connect(m_startListenAct, &QAction::triggered, this, &MainWindow::onStartListening);
    m_stopListenAct = listenMenu->addAction("停止监听");
    m_stopListenAct->setObjectName("stopListenAct");
    m_stopListenAct->setShortcut(QKeySequence("Shift+F5"));
    connect(m_stopListenAct, &QAction::triggered, this, &MainWindow::onStopListening);

    // ── 控制 ──
    // 三个控制动作都设 objectName：单测靠 findChild 取到它们，验证「构造完成、
    // 第一次 onRefresh 之前」它们就是禁用的。按标题取会随文案改动而失效，
    // 而 m_enableAct 的标题本身就会在跟踪时变成「已使能跟踪」。
    QMenu *ctlMenu = menuBar()->addMenu("控制(&C)");
    m_enableAct = ctlMenu->addAction("使能跟踪");
    m_enableAct->setObjectName("enableTrackAct");
    m_enableAct->setShortcut(QKeySequence("F9"));
    connect(m_enableAct, &QAction::triggered, this, &MainWindow::onPrepareTracking);
    m_stopTrackAct = ctlMenu->addAction("停止跟踪");
    m_stopTrackAct->setObjectName("stopTrackAct");
    m_stopTrackAct->setShortcut(QKeySequence("Esc"));
    // 提示文案挂在动作本身：软停止与急停的区别是关于「按下这个会发生什么」
    // 的说明，属于这个动作，而不属于界面上某个固定位置。
    m_stopTrackAct->setToolTip(kNoticeNormal);
    connect(m_stopTrackAct, &QAction::triggered, this, &MainWindow::onStopTracking);
    ctlMenu->addSeparator();
    m_resetFaultAct = ctlMenu->addAction("复位故障");
    m_resetFaultAct->setObjectName("resetFaultAct");
    connect(m_resetFaultAct, &QAction::triggered, this, &MainWindow::onResetFault);
    ctlMenu->addSeparator();
    QAction *paramsAct = ctlMenu->addAction("编辑控制参数…");
    connect(paramsAct, &QAction::triggered, this, &MainWindow::onEditParams);

    // 刻意不像 rlPlanDemo 那样对每个动作再调一次 this->addAction()：那一步是
    // 为「不在菜单栏里的菜单」准备的。挂在菜单栏下的 QAction 的快捷键已经是
    // 窗口作用域（菜单未展开时同样生效），再把同一个动作加到窗口上会让
    // 同一组按键在 QShortcutMap 里注册两次，运行时打 "Ambiguous shortcut
    // overload" 并且什么都不触发——比没有快捷键更糟。

    // ── 视图：各面板的显示开关 ──
    // toggleViewAction() 直接给出带勾选状态的 QAction，显示状态与菜单勾选
    // 自动同步，不必自己维护。
    QMenu *viewMenu = menuBar()->addMenu("视图(&V)");
    for (QDockWidget *d : docks())
        viewMenu->addAction(d->toggleViewAction());

    // ── 工具栏：安全关键动作 ──
    // 使能与停止同时留在工具栏。菜单里的动作要两次点击才触发（先展开菜单、
    // 再点条目），停止跟踪不该有这个延迟。
    QToolBar *tb = addToolBar("控制");
    tb->setObjectName("controlToolBar");
    // 动作没有图标。默认的 ToolButtonIconOnly 下无图标动作只剩一个空按钮，
    // 必须显式要求显示文字。
    tb->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    tb->addAction(m_startListenAct);
    tb->addAction(m_stopListenAct);
    tb->addSeparator();
    tb->addAction(m_resetFaultAct);
    tb->addAction(m_enableAct);
    tb->addAction(m_stopTrackAct);

    viewMenu->addSeparator();
    viewMenu->addAction(tb->toggleViewAction());
    viewMenu->addSeparator();
    QAction *resetLayoutAct = viewMenu->addAction("恢复默认布局");
    connect(resetLayoutAct, &QAction::triggered, this, &MainWindow::onResetLayout);

    // ── 帮助 ──
    QMenu *helpMenu = menuBar()->addMenu("帮助(&H)");
    QAction *aboutAct = helpMenu->addAction("关于");
    connect(aboutAct, &QAction::triggered, this, &MainWindow::onAbout);
    QAction *aboutQtAct = helpMenu->addAction("关于 Qt");
    connect(aboutQtAct, &QAction::triggered, qApp, &QApplication::aboutQt);
}

void MainWindow::buildStatusBar()
{
    // 常驻标签用 addPermanentWidget（右侧），瞬时消息用 showMessage（左侧）。
    // 连接/控制状态是「随时想瞥一眼」的信息，占一整块面板不值得——原先四张
    // 状态卡片吃掉窗口顶部一整条，而它们合起来只有八个汉字的信息量。

    // 安全提示用 addWidget 而不是 addPermanentWidget：addWidget 放在左侧的
    // 消息区，showMessage 期间被暂时盖住、消息过期后自动露出来。这正是要的
    // 语义——联锁拦截原因这类瞬时消息应当优先，但它一过去，「软停止不是急停」
    // 必须自己回来。
    m_noticeLabel = new QLabel(kNoticeNormal, this);
    QFont noticeF = m_noticeLabel->font();
    noticeF.setBold(true);
    m_noticeLabel->setFont(noticeF);
    statusBar()->addWidget(m_noticeLabel);

    m_connLabel  = new QLabel(this);
    m_stateLabel = new QLabel(this);
    m_ipocLabel  = new QLabel(this);
    m_cycleLabel = new QLabel(this);
    for (QLabel *l : {m_connLabel, m_stateLabel, m_ipocLabel, m_cycleLabel}) {
        QFont f = l->font();
        f.setBold(true);
        l->setFont(f);
        statusBar()->addPermanentWidget(l);
    }
    // 读数用等宽字体，理由与表格各列相同：数字每帧都在变，等宽让位数不跳。
    m_ipocLabel->setFont(uilogic::monospaceFont());
    m_cycleLabel->setFont(uilogic::monospaceFont());

    pinLabelWidth(m_connLabel,  "已连接 丢包 9999");
    pinLabelWidth(m_stateLabel, "跟踪中 接近限值 100%");
    pinLabelWidth(m_ipocLabel,  "IPOC 9999999999");
    pinLabelWidth(m_cycleLabel, "周期 99.99 ms");

    // 首帧到来前先摆成「未监听 / 无」，否则四个标签是空的，看起来像界面没起来。
    updateStatusBar(m_state.snapshot());
}

void MainWindow::updateStatusBar(const StatusSnapshot &s)
{
    using uilogic::Severity;

    // 「监听中」「就绪」「同步中」「等待首帧」这些过渡态不涂蓝：Severity 里
    // 没有蓝这一档（理由见 UiLogic.h），它们与「未监听」同属「没出问题、
    // 还没开始跑」，一律 Idle，区分靠状态文字本身。

    // ── 连接 ──
    // 丢包并进这一格而不是另开一格：丢包只在已连接时有意义，而状态栏每多一格
    // 常驻标签就少一截 showMessage 可用的宽度。详细计数在「通信指标」面板里。
    if (!s.connected)
        setSeverityText(m_connLabel, m_listening ? "监听中" : "未监听", Severity::Idle);
    else if (s.missedCount > 0 || s.peerRejected > 0)
        setSeverityText(m_connLabel,
                        QStringLiteral("已连接 丢包 %1").arg(s.missedCount),
                        Severity::Warn);
    else
        setSeverityText(m_connLabel, "已连接", Severity::Ok);

    // ── 控制状态 ──
    switch (s.state) {
    case ControlState::Fault:
        setSeverityText(m_stateLabel, "故障锁存", Severity::Fault);
        break;
    case ControlState::Tracking: {
        // 跟踪质量并进控制状态这一格：它只在 Tracking 下有意义，单独占一格的话
        // 其余状态下永远显示「无数据」——一格常驻宽度换一句废话。
        QString  txt = "跟踪中";
        Severity sev = Severity::Ok;
        if (s.accumOverLimit || s.trackingQuality == TrackingQuality::OverLimit) {
            txt = "跟踪(超限)";
            sev = Severity::Fault;
        } else if (s.trackingQuality == TrackingQuality::NearLimit) {
            txt = QStringLiteral("跟踪中 接近限值 %1%")
                      .arg(int(std::max(s.accumPosPct, s.errorPosPct) * 100));
            sev = Severity::Warn;
        } else if (s.trackingQuality == TrackingQuality::LargeError) {
            txt = QStringLiteral("跟踪中 偏差 %1%").arg(int(s.errorPosPct * 100));
            sev = Severity::Warn;
        }
        setSeverityText(m_stateLabel, txt, sev);
        break;
    }
    case ControlState::Ready:
        setSeverityText(m_stateLabel, "就绪", Severity::Idle); break;
    case ControlState::Syncing:
        setSeverityText(m_stateLabel, "同步中", Severity::Idle); break;
    case ControlState::WaitingFirstFrame:
        setSeverityText(m_stateLabel, "等待首帧", Severity::Idle); break;
    case ControlState::StaleFrame:
        setSeverityText(m_stateLabel, "帧异常", Severity::Warn); break;
    default:
        setSeverityText(m_stateLabel, "未连接", Severity::Idle); break;
    }

    // ── IPOC ──
    // 丢包即 IPOC 不连续，与「通信指标」面板同一个判据。
    setSeverityText(m_ipocLabel, QStringLiteral("IPOC %1").arg(s.ipoc),
                    !s.connected      ? Severity::Idle
                    : s.missedCount > 0 ? Severity::Fault
                                        : Severity::Ok);

    // ── 周期 ──
    // 显示均值而非单帧值：单帧值每 20ms 抖一位，读不出趋势。判坏的门限与
    // 「通信指标」面板一致（偏离配置值 10%）——同一个量两处两个门限，
    // 操作员会看到一格红一格绿。
    const bool cycleBad = (s.measuredCycleMs > 0.0 && m_cfg.cycleMs > 0.0
                           && std::fabs(s.measuredCycleMs - m_cfg.cycleMs)
                                  > 0.10 * m_cfg.cycleMs);
    setSeverityText(m_cycleLabel,
                    QStringLiteral("周期 %1 ms").arg(s.cycleMeanMs, 0, 'f', 2),
                    !s.connected ? Severity::Idle
                    : cycleBad   ? Severity::Warn
                                 : Severity::Ok);

    // ── 安全提示 ──
    const bool isFault = (s.state == ControlState::Fault) || s.accumOverLimit;
    setSeverityText(m_noticeLabel, isFault ? kNoticeFault : kNoticeNormal,
                    isFault ? Severity::Fault : Severity::Idle);
}

void MainWindow::onResetLayout()
{
    restoreState(m_defaultLayout);
    // 显示状态与 restoreState 分开维护（见 showEvent），所以这里也要分开复位。
    // 少了这一步，「恢复默认布局」救不回被全部关掉的面板——而那正是最需要
    // 这个菜单项的处境。
    for (QDockWidget *d : docks())
        d->setVisible(d != m_alarmDock);   // 事件日志默认隐藏
}

void MainWindow::onAbout()
{
    QMessageBox::about(this, "关于",
        QStringLiteral(
            "<b>KUKA RSI POSCORR 位姿跟踪</b><br><br>"
            "以 RSI 的 POSCORR 通道对机器人做位姿闭环修正。<br><br>"
            "<b>%1</b><br><br>"
            "快捷键：<br>"
            "F5 开始监听　Shift+F5 停止监听<br>"
            "F9 使能跟踪　Esc 停止跟踪")
            .arg(QString::fromUtf8(kNoticeNormal)));
}

// ═══════════════════════════════════════════════
// 面板：监听配置
// ═══════════════════════════════════════════════

QWidget *MainWindow::buildListenPanel()
{
    auto *w = new QWidget(this);
    auto *lay = new QHBoxLayout(w);
    lay->setContentsMargins(6, 4, 6, 4);
    lay->addWidget(new QLabel("IP", w));
    // objectName 供单测用 findChild 取控件，验证初始启用态确实被施加过。
    m_ipEdit = new QLineEdit(m_cfg.listenIp, w);
    m_ipEdit->setObjectName("listenIpEdit");
    m_ipEdit->setMaximumWidth(120);
    lay->addWidget(m_ipEdit);
    lay->addWidget(new QLabel(":", w));
    m_portSpin = new QSpinBox(w);
    m_portSpin->setObjectName("listenPortSpin");
    m_portSpin->setRange(1, 65535);
    m_portSpin->setValue(int(m_cfg.listenPort));
    m_portSpin->setMaximumWidth(80);
    lay->addWidget(m_portSpin);
    lay->addStretch();
    return w;
}

// ═══════════════════════════════════════════════
// 面板：目标位姿
// ═══════════════════════════════════════════════

QWidget *MainWindow::buildTargetPanel()
{
    auto *w = new QWidget(this);
    auto *v = new QVBoxLayout(w);
    v->setContentsMargins(4, 4, 4, 4);
    v->setSpacing(4);

    auto *tbl = new QTableWidget(6, 4, w);
    tbl->setHorizontalHeaderLabels({"轴", "目标值", "当前值（只读）", "调整"});
    tbl->verticalHeader()->setVisible(false);
    tbl->setShowGrid(false);
    tbl->setSelectionMode(QAbstractItemView::NoSelection);
    tbl->setEditTriggers(QAbstractItemView::NoEditTriggers);
    // 列宽全部按字体度量显式给出（见 setTableColumns 的注释）。
    // 「调整」列只装两个方形小按钮加 2px 间距。
    // 字号不点名 10：删掉 QSS 后系统字号说了算，写死 10 会让读数比周围的
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
    // 多余宽度交给「调整」列：那一列里两个按钮是靠左的定尺寸部件，
    // 空白落在它们右边不影响任何读数。数值列拉宽反而会让小数点离开视线焦点。
    setTableColumns(tbl, {wAxis, wSpin, wLive, wStep}, 3);

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
    v->addWidget(tbl);

    // 步长选择
    auto *stepRow = new QHBoxLayout;
    stepRow->addWidget(new QLabel("步长", w));
    m_stepSel = new QComboBox(w);
    m_stepSel->addItems({"0.01", "0.1", "0.5", "1", "5", "10"});
    m_stepSel->setCurrentText("0.5");
    stepRow->addWidget(m_stepSel);
    stepRow->addStretch();
    v->addLayout(stepRow);

    // 差值预览。原生边框代替浅黄底：它列的是「目标 − 当前」的逐轴数字，
    // 等宽字体让这些数字成列比底色更要紧；而黄底暗示「警告」，实际上
    // 有偏差是编辑目标后的常态，不是异常。
    m_deltaPreview = new QLabel("差值预览：编辑目标值后此处显示当前偏差", w);
    m_deltaPreview->setFont(uilogic::monospaceFont());
    m_deltaPreview->setFrameShape(QFrame::StyledPanel);
    m_deltaPreview->setWordWrap(true);
    v->addWidget(m_deltaPreview);

    // 操作按钮
    auto *actRow = new QHBoxLayout;
    // 「读取当前值」把 actual 抄进目标输入框，所以它必须和其它动作一样有守卫：
    // 未收到有效帧时 actual 是全零，读进去再点「应用目标」就把目标设成了原点
    // ——机器人会朝 BASE 原点走。启用条件由 uilogic::buttonStates 统一给出。
    m_readActualBtn = new QPushButton("读取当前值", w);
    m_readActualBtn->setObjectName("readActualBtn");
    m_readActualBtn->setEnabled(false);
    connect(m_readActualBtn, &QPushButton::clicked, this, &MainWindow::onReadActualTarget);
    actRow->addWidget(m_readActualBtn);
    // 「应用目标」不再涂成蓝底白字：强调交给下面的 setDefault，那是平台自带的
    // 默认按钮外观，操作员在别的 Windows 程序里已经认得它。
    auto *apply = new QPushButton("应用目标", w);
    // setDefault 只在按钮的 autoDefault 打开时生效（QPushButton 在非对话框
    // 父窗口里默认关闭）。updateApplyButton 靠 setDefault 表达「改了没发」，
    // 少了这行它就是一次静默的空操作。
    apply->setAutoDefault(true);
    connect(apply, &QPushButton::clicked, this, &MainWindow::onApplyTarget);
    actRow->addWidget(apply);
    m_applyBtn = apply;
    auto *undo = new QPushButton("撤销修改", w);
    connect(undo, &QPushButton::clicked, this, &MainWindow::onUndoTarget);
    actRow->addWidget(undo);
    actRow->addStretch();
    v->addLayout(actRow);

    v->addStretch();
    return w;
}

// ═══════════════════════════════════════════════
// 面板：位姿对比
// ═══════════════════════════════════════════════

QWidget *MainWindow::buildComparePanel()
{
    auto *w = new QWidget(this);
    auto *v = new QVBoxLayout(w);
    v->setContentsMargins(4, 4, 4, 4);

    // 轴名占第 0 列而不是垂直表头：表头下一行就被 setVisible(false) 隐藏，
    // 写进去的 X/Y/Z/A/B/C 永远显示不出来，六行数字谁也不知道是哪个轴。
    auto *tbl = new QTableWidget(6, 5, w);
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
    // 多余宽度交给误差列：它是这张表里最需要被读全的一列（带前缀、最长），
    // 而且拉宽它不会把任何一列的单位后缀推出可视区。
    setTableColumns(tbl, {kAxisCol, wData, wError, wData, wRkorr}, 2);
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

        // 目标（只显示，不编辑——编辑在「目标位姿」面板）
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
    v->addWidget(tbl);
    v->addStretch();
    return w;
}

// ═══════════════════════════════════════════════
// 面板：累积修正 / 控制参数 / 通信指标
// ═══════════════════════════════════════════════

QWidget *MainWindow::buildCumulPanel()
{
    // 同 buildCommPanel：六行进度条不该被多余高度拉开，行距变了就不像一组了。
    auto *w = new QWidget(this);
    auto *v = new QVBoxLayout(w);
    v->setContentsMargins(0, 0, 0, 0);
    m_cumulBar = new CumulativeBar(w);
    v->addWidget(m_cumulBar);
    v->addStretch();
    return w;
}

QWidget *MainWindow::buildParamPanel()
{
    auto *w = new QWidget(this);
    auto *v = new QVBoxLayout(w);
    v->setContentsMargins(6, 4, 6, 4);
    v->setSpacing(6);

    // 控制参数只读行：标签与数值在同一次循环里成对创建，
    // 行列由同一个下标算出，错位在结构上不可能发生（表见文件顶部 kParams）。
    auto *paramForm = new QGridLayout;
    paramForm->setHorizontalSpacing(12);
    paramForm->setVerticalSpacing(4);
    for (int i = 0; i < 6; ++i) {
        const int row = i / 2;
        const int col = (i % 2) * 2;
        auto *name = new QLabel(kParams[i].label, w);
        paramForm->addWidget(name, row, col);

        // 只读数值：等宽 + 右对齐，六个参数的小数点成列才能一眼看出量级差异。
        // 原先靠动态属性让 QSS 的 readout 规则挑中它，属性一删就得自己带字体。
        auto *val = new QLabel("--", w);
        val->setFont(uilogic::monospaceFont());
        val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        paramForm->addWidget(val, row, col + 1);
        m_paramVal[i] = val;
    }
    paramForm->setColumnStretch(1, 1);
    paramForm->setColumnStretch(3, 1);
    v->addLayout(paramForm);

    m_paramsBtn = new QPushButton("编辑控制参数…", w);
    connect(m_paramsBtn, &QPushButton::clicked, this, &MainWindow::onEditParams);
    v->addWidget(m_paramsBtn);
    v->addStretch();
    return w;
}

QWidget *MainWindow::buildCommPanel()
{
    // 卡片外面要一层带 addStretch 的容器。直接把 CommCards 交给 QDockWidget 的
    // 话，面板分到的多余高度会全部灌进那一行卡片，四张卡片被拉成一人多高、
    // 上下两行读数隔开半屏——同一张卡片上的「均值」与「P99」本该并读。
    auto *w = new QWidget(this);
    auto *v = new QVBoxLayout(w);
    v->setContentsMargins(4, 4, 4, 4);
    m_commCards = new CommCards(w);
    v->addWidget(m_commCards);
    v->addStretch();
    return w;
}

// ═══════════════════════════════════════════════
// 连接控制
// ═══════════════════════════════════════════════

void MainWindow::applyButtonStates(const ButtonStates &b)
{
    // ButtonStates 全部字段的唯一施加点。
    //
    // 为什么必须只有一处：原先构造期的 updateConnControls 与每帧的 onRefresh
    // 各自手写一份 setEnabled 清单，而前者漏了 resetFault / enableTrack /
    // stopTrack 三项。这三个动作从按钮改成 QAction 之后再没有初始禁用语句，
    // 而 QAction 默认是启用的——于是从构造到第一次 onRefresh 之间（一个
    // refresh_ms），「未监听/未连接」状态下「复位故障 / 使能跟踪 / 停止跟踪」
    // 连同 F9 / Esc 快捷键全都可点。手写清单少一行时编译器不会报错，所以
    // 靠「记得两边都改」是不成立的，两份清单必须收敛成一份。
    if (m_resetFaultAct)  m_resetFaultAct->setEnabled(b.resetFault);
    if (m_enableAct)      m_enableAct->setEnabled(b.enableTrack);
    if (m_stopTrackAct)   m_stopTrackAct->setEnabled(b.stopTrack);
    if (m_startListenAct) m_startListenAct->setEnabled(b.startListen);
    if (m_stopListenAct)  m_stopListenAct->setEnabled(b.stopListen);
    if (m_ipEdit)         m_ipEdit->setEnabled(b.connEditable);
    if (m_portSpin)       m_portSpin->setEnabled(b.connEditable);
    if (m_readActualBtn)  m_readActualBtn->setEnabled(b.readActual);

    // 让编译器参与「新增字段必须接线」这件事：ButtonStates 全是 bool，
    // sizeof 就是字段数。加一个字段而忘了在上面接线，这行就编译不过——
    // 否则新字段会重演同一个缺陷（判定算出来了，但没人施加）。
    static_assert(sizeof(ButtonStates) == 7 * sizeof(bool),
                  "ButtonStates 字段有增减，请同步上面的接线清单");
}

void MainWindow::updateConnControls()
{
    // 这个入口不能省：构造期还没有第一帧快照，bindFailed / 停止监听也要在
    // 下一次刷新到来之前立刻把控件改回去。判定与施加都与 onRefresh 共用。
    applyButtonStates(uilogic::buttonStates(m_state.snapshot(), m_listening));
}

void MainWindow::onStartListening()
{
    if (!m_worker || m_listening) return;
    m_cfg.listenIp   = m_ipEdit->text().trimmed();
    m_cfg.listenPort = quint16(m_portSpin->value());
    // 上一次的联锁拦截原因已经过时，别让它留在状态栏里误导下一次尝试。
    statusBar()->clearMessage();
    QMetaObject::invokeMethod(m_worker, "applyConfig", Qt::QueuedConnection,
                              Q_ARG(AppConfig, m_cfg));
    QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection);
}

void MainWindow::onStopListening()
{
    if (!m_worker) return;
    QMetaObject::invokeMethod(m_worker, "stop", Qt::QueuedConnection);
    // socket 一关就不可能再收发帧，跟踪也就无从谈起。不取消的话控制器
    // 状态会停在 Tracking，状态栏持续显示「跟踪中」——覆盖一台已经
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
    // 按钮的禁用状态不是唯一入口：回车、辅助工具、以后可能加的快捷键都能
    // 走到这里。守卫复用同一个 buttonStates，不在这里另写一套判定。
    if (!uilogic::buttonStates(m_state.snapshot(), m_listening).readActual)
        return;
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
        // 拦截原因走状态栏的瞬时消息，不再占一条常驻的红字横幅：它只在
        // 「刚点了使能却没反应」那一刻需要被读到。10 秒足够读完，之后
        // 让位给常驻的安全提示，而不是一直留在界面上冒充当前故障。
        statusBar()->showMessage("使能被拦截：" + blocked.join("; "), 10000);
        return;
    }
    statusBar()->clearMessage();

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
    updateStatusBar(s);

    // ── 位姿对比 ──
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
        // 目标位姿面板的当前值列
        m_liveLabel[i]->setText(uilogic::formatValue(act[i], i));
    }

    // ── 累积修正 ──
    m_cumulBar->updateFrom(s.accum, m_cfg.accumLimitPosMm, m_cfg.accumLimitRotDeg);

    // ── 通信指标 ──
    m_commCards->updateFrom(s, m_cfg.cycleMs);

    // ── 控制参数只读 ──
    // 字段、单位、小数位都来自建面板时用的同一张表，不再手写六行按位置对应。
    for (int i = 0; i < 6; ++i)
        m_paramVal[i]->setText(QStringLiteral("%1%2")
                                   .arg(m_cfg.*(kParams[i].field), 0, 'f', kParams[i].decimals)
                                   .arg(kParams[i].unit));

    // ── 动作启用状态机 ──
    // 全部交给 uilogic::buttonStates，界面这边不再自己判状态。原先手写的
    // 那条 `m_stopBtn->setEnabled(s.state == Tracking)` 已整段删除：留着它
    // 会在 StaleFrame / Syncing 下继续把停止按钮置灰，而那正是最需要能停的
    // 时刻（反馈已异常、PoseController 仍在发增量）。
    applyButtonStates(uilogic::buttonStates(s, m_listening));
    // 文案不属于启用状态，留在这里：applyButtonStates 只做 ButtonStates 的施加。
    m_enableAct->setText(s.state == ControlState::Tracking ? "已使能跟踪"
                                                          : "使能跟踪");

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
