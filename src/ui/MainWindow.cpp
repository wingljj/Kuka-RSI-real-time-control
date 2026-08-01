#include "ui/MainWindow.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QMessageBox>
#include <QPushButton>
#include <QScreen>
#include <QScrollArea>
#include <QVBoxLayout>
#include <cmath>
#include "core/SessionGuard.h"
#include "ui/ErrorChart.h"

namespace {

const char *kAxisName[6] = {"X", "Y", "Z", "A", "B", "C"};

// 位置 ±4000mm，姿态 ±180°
double axisMin(int i) { return i < 3 ? -4000.0 : -180.0; }
double axisMax(int i) { return i < 3 ?  4000.0 :  180.0; }
const char *axisUnit(int i) { return i < 3 ? " mm" : " °"; }

} // namespace

MainWindow::MainWindow(const AppConfig &cfg, QWidget *parent)
    : QMainWindow(parent), m_cfg(cfg)
{
    setWindowTitle("KUKA RSI POSCORR 位姿跟踪");

    m_stateCard = new QLabel("未连接", this);
    m_stateCard->setStyleSheet(
        "font-size: 18px; font-weight: bold; color: #888;");
    m_stateDetail = new QLabel("", this);
    m_stateDetail->setStyleSheet("color: #555;");

    auto *central = new QWidget(this);
    auto *outer = new QVBoxLayout(central);
    outer->addWidget(m_stateCard);
    outer->addWidget(m_stateDetail);
    outer->addWidget(buildConnPanel());

    // 按钮栏紧随状态栏，并且刻意放在下面那个滚动区之外。
    // 它原先位于整个布局的最底部，而目标面板、控制参数、曲线、读数四者的
    // minimumSizeHint 之和会把窗口撑到 900px 以上——于是在 1440 高的屏幕上
    // 按钮栏被直接推出屏幕，操作员既看不到也点不到「使能跟踪」「停止跟踪」，
    // 以及那条说明"软停止不是急停"的提示。安全关键控件的可见性不能依赖
    // "窗口恰好够大"：把它们放在不可压缩的位置，结构上就不可能被截断。
    auto *bar = new QHBoxLayout;

    auto *zeroBtn = new QPushButton("归零到当前位姿", this);
    connect(zeroBtn, &QPushButton::clicked,
            this, &MainWindow::onZeroToActual);
    bar->addWidget(zeroBtn);

    // 两阶段使能：构造时按钮置灰，连接本身不会启动运动。操作员先看读数是否
    // 合理，再点「准备跟踪」经确认框后启用。任何"连上就自动使能"的便利都
    // 不可接受。
    m_enableBtn = new QPushButton("准备跟踪", this);
    m_enableBtn->setEnabled(false);   // 首帧前不可用
    connect(m_enableBtn, &QPushButton::clicked,
            this, &MainWindow::onPrepareTracking);
    bar->addWidget(m_enableBtn);

    auto *stopBtn = new QPushButton("停止跟踪", this);
    connect(stopBtn, &QPushButton::clicked,
            this, &MainWindow::onStopTracking);
    bar->addWidget(stopBtn);

    bar->addStretch();
    m_safetyNote = new QLabel(
        "⚠ 软件停止 = 目标归零并继续回包。急停只有示教器上的物理急停按钮。",
        this);
    m_safetyNote->setStyleSheet(
        "background-color: #fdd; color: #900; font-weight: bold; "
        "padding: 4px 8px; border: 1px solid #c00;");
    // 刻意不开 setWordWrap：关闭自动换行时 QLabel::minimumSizeHint() 等于整段
    // 文本的宽度，布局因此不可能把这条提示截断——窗口只会拒绝再变窄。开了
    // 换行反而允许标签被压成一行高度而把后半句藏掉，那是安全提示最坏的失效
    // 方式。
    bar->addWidget(m_safetyNote);

    outer->addLayout(bar);

    m_interlockLabel = new QLabel(this);
    m_interlockLabel->setStyleSheet("color: #b00; font-weight: bold;");
    m_interlockLabel->setWordWrap(true);
    m_interlockLabel->hide();
    outer->addWidget(m_interlockLabel);

    // 可压缩的内容放进滚动区：窗口再小也能滚到，而顶部的状态栏与控制栏始终
    // 可见。这样"看得到状态、按得到停止"就不再是一个取决于屏幕尺寸的巧合。
    auto *row = new QHBoxLayout;
    auto *left = new QVBoxLayout;
    left->addWidget(buildTargetPanel());
    left->addWidget(buildSingularWarn());
    left->addWidget(buildParamPanel());
    left->addStretch();
    row->addLayout(left, 1);
    auto *right = new QVBoxLayout;
    // 位置图在上、姿态图在下，各占一个 ErrorChart。原先单图双 Y 轴把量纲
    // 不同的 mm 与 ° 压在同一张图上，量程互相挤压、读数难分；拆开后每图
    // 只画一条线、只配一个 Y 轴，比例与空态都由各自处理。
    m_chartPos = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Position, this);
    m_chartRot = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Rotation, this);
    auto *chartCol = new QVBoxLayout;
    // 位置/姿态两图各占较高高度（stretch 2），图表区整体 3:1 于读数——曲线
    // 太小是"看不清"的直接原因，宁可给图更多空间。
    chartCol->addWidget(m_chartPos, 2);
    chartCol->addWidget(m_chartRot, 2);
    right->addLayout(chartCol, 3);
    right->addWidget(buildReadoutPanel(), 1);
    row->addLayout(right, 1);

    auto *scrollInner = new QWidget;
    scrollInner->setLayout(row);
    auto *scroll = new QScrollArea(this);
    scroll->setWidget(scrollInner);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    outer->addWidget(scroll, 1);

    setCentralWidget(central);

    // 通信线程
    m_commThread = new QThread(this);
    m_worker = new RsiWorker(m_cfg, &m_state, &m_ring);
    m_worker->moveToThread(m_commThread);
    // start() 必须在通信线程里执行，它 new 出来的 QUdpSocket/QTimer 才会带上
    // 该线程的亲和性；因此只经由 QThread::started 触发，绝不直接调用。
    connect(m_commThread, &QThread::started,
            m_worker, &RsiWorker::start);
    // worker 无 parent（moveToThread 要求），随线程结束自毁。
    connect(m_commThread, &QThread::finished,
            m_worker, &QObject::deleteLater);
    // 两个 lambda 都显式传 this 作为 context：否则它们会在发射线程（通信线程）
    // 上执行，而它们都要碰窗口部件。
    connect(m_worker, &RsiWorker::listening,
            this, [this] {
                m_listening = true;
                updateConnControls();
            });
    connect(m_worker, &RsiWorker::bindFailed,
            this, [this](const QString &why) {
                // 绑定失败不再是死局：回到未绑定态，地址与端口重新可改、
                // 「开始监听」重新可用。端口被占用是最常见的现场情形
                //（例如上一次的模拟器还挂着），不该逼操作员改 JSON 重启。
                m_listening = false;
                updateConnControls();
                QMessageBox::critical(this, "监听失败", why);
            });
    connect(m_worker, &RsiWorker::firstFrameReceived,
            this, [this] {
                // 首帧到达：把界面目标同步为机器人实际位姿
                const Pose a = m_state.snapshot().actual;
                m_suppressTargetSignal = true;
                const double v[6] = {a.x, a.y, a.z, a.a, a.b, a.c};
                for (int i = 0; i < 6; ++i) {
                    m_targetSpin[i]->setValue(v[i]);
                    m_targetSlider[i]->setValue(int(v[i] * 10.0));
                }
                m_suppressTargetSignal = false;
            });
    m_commThread->start();

    m_refresh = new QTimer(this);
    m_refresh->setInterval(m_cfg.refreshMs);
    connect(m_refresh, &QTimer::timeout, this, &MainWindow::onRefresh);
    m_refresh->start();

    // 初始尺寸夹到工作区之内。滚动区已经保证按钮栏不会被挤出屏幕，但一个
    // 开局就比屏幕还高的窗口仍然会让操作员第一眼看不到读数，得先去拖窗口。
    const QRect wa = QGuiApplication::primaryScreen()->availableGeometry();
    resize(qMin(1180, wa.width() - 80), qMin(760, wa.height() - 80));

    // 初始按未绑定渲染。线程启动会自动尝试用配置文件里的地址绑定，成功后
    // listening() 会把状态翻过来；失败则停在这个状态，地址可改、可重试。
    updateConnControls();
}

MainWindow::~MainWindow()
{
    // isRunning() 守卫：BlockingQueuedConnection 只有在目标线程还在跑事件循环
    // 时才会返回；若线程从未启动或已退出，这一行会永久挂住 GUI 线程。
    if (m_commThread && m_commThread->isRunning()) {
        QMetaObject::invokeMethod(m_worker, "stop",
                                  Qt::BlockingQueuedConnection);
        m_commThread->quit();
        m_commThread->wait(2000);
    }
}

QWidget *MainWindow::buildConnPanel()
{
    auto *w = new QWidget(this);
    auto *lay = new QHBoxLayout(w);
    lay->setContentsMargins(0, 0, 0, 0);

    lay->addWidget(new QLabel("监听地址", w));
    m_ipEdit = new QLineEdit(m_cfg.listenIp, w);
    m_ipEdit->setMaximumWidth(150);
    lay->addWidget(m_ipEdit);

    lay->addWidget(new QLabel(":", w));
    m_portSpin = new QSpinBox(w);
    m_portSpin->setRange(1, 65535);
    m_portSpin->setValue(int(m_cfg.listenPort));
    m_portSpin->setMaximumWidth(95);
    lay->addWidget(m_portSpin);

    m_listenBtn = new QPushButton("开始监听", w);
    connect(m_listenBtn, &QPushButton::clicked,
            this, &MainWindow::onStartListening);
    lay->addWidget(m_listenBtn);

    m_unlistenBtn = new QPushButton("停止监听", w);
    connect(m_unlistenBtn, &QPushButton::clicked,
            this, &MainWindow::onStopListening);
    lay->addWidget(m_unlistenBtn);

    lay->addStretch();
    return w;
}

void MainWindow::updateConnControls()
{
    // 地址与端口只在未绑定时可编辑。运行中改它们不会生效，只会让界面显示的
    // 地址与实际绑定的地址不符——那比不给改更糟。
    if (m_ipEdit)      m_ipEdit->setEnabled(!m_listening);
    if (m_portSpin)    m_portSpin->setEnabled(!m_listening);
    if (m_listenBtn)   m_listenBtn->setEnabled(!m_listening);
    if (m_unlistenBtn) m_unlistenBtn->setEnabled(m_listening);
}

void MainWindow::onStartListening()
{
    if (!m_worker || m_listening)
        return;
    m_cfg.listenIp   = m_ipEdit->text().trimmed();
    m_cfg.listenPort = quint16(m_portSpin->value());
    // 先把新地址推给通信线程，再让它绑定。两者都走同一条队列，投递顺序
    // 因此有保证——不必担心 start() 抢在 applyConfig() 之前拿到旧地址。
    if (m_interlockLabel)
        m_interlockLabel->hide();
    QMetaObject::invokeMethod(m_worker, "applyConfig",
                              Qt::QueuedConnection, Q_ARG(AppConfig, m_cfg));
    QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection);
    // 不在此处置 m_listening：只有 RsiWorker 真正 bind 成功后发出的
    // listening() 才算数，否则界面会先宣称监听中、随后又弹绑定失败。
}

void MainWindow::onStopListening()
{
    if (!m_worker)
        return;
    QMetaObject::invokeMethod(m_worker, "stop", Qt::QueuedConnection);
    m_listening = false;
    // socket 一关就不可能再有回包，跟踪也就无从谈起。按钮状态由 onRefresh
    // 依据 s.connected 统一刷新，这里不必手动置灰。
    updateConnControls();
}

void MainWindow::onEditParams()
{
    QDialog dlg(this);
    dlg.setWindowTitle("控制参数");

    auto *form = new QFormLayout(&dlg);
    auto *kpP  = new QDoubleSpinBox; kpP->setRange(0.0, 100.0);  kpP->setDecimals(3);
    auto *kpR  = new QDoubleSpinBox; kpR->setRange(0.0, 100.0);  kpR->setDecimals(3);
    auto *vP   = new QDoubleSpinBox; vP->setRange(0.0, 10000.0); vP->setSuffix(" mm/s");
    auto *vR   = new QDoubleSpinBox; vR->setRange(0.0, 10000.0); vR->setSuffix(" °/s");
    auto *alP  = new QDoubleSpinBox; alP->setRange(0.0, 10000.0); alP->setSuffix(" mm");
    auto *alR  = new QDoubleSpinBox; alR->setRange(0.0, 10000.0); alR->setSuffix(" °");
    kpP->setValue(m_cfg.kpPos);  kpR->setValue(m_cfg.kpRot);
    vP->setValue(m_cfg.vmaxPosMmS);  vR->setValue(m_cfg.vmaxRotDegS);
    alP->setValue(m_cfg.accumLimitPosMm);  alR->setValue(m_cfg.accumLimitRotDeg);

    form->addRow("Kp 位置", kpP);
    form->addRow("Kp 姿态", kpR);
    form->addRow("限速位置", vP);
    form->addRow("限速姿态", vR);
    form->addRow("累积上限位置", alP);
    form->addRow("累积上限姿态", alR);

    // 运行中锁定 + 显示 KRC 硬限与余量
    const bool locked = m_state.snapshot().state == ControlState::Tracking;
    const std::array<QWidget *, 6> fields = {kpP, kpR, vP, vR, alP, alR};
    for (QWidget *w : fields)
        w->setEnabled(!locked);
    if (locked) {
        auto *note = new QLabel("运行中：参数已锁定，停止跟踪后可修改。", &dlg);
        note->setStyleSheet("color: #a06000;");
        form->addRow(note);
    } else {
        form->addRow(new QLabel(QStringLiteral(
            "KRC 硬限: 位置 %1 mm / 姿态 %2 °；当前主机上限 %3 / %4，余量 %5 / %6")
            .arg(m_cfg.krcPoscorrLimitPosMm).arg(m_cfg.krcPoscorrLimitRotDeg)
            .arg(alP->value()).arg(alR->value())
            .arg(m_cfg.krcPoscorrLimitPosMm - alP->value())
            .arg(m_cfg.krcPoscorrLimitRotDeg - alR->value()), &dlg));
    }

    auto *btn = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    connect(btn, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btn, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    form->addRow(btn);

    if (dlg.exec() != QDialog::Accepted)
        return;
    AppConfig c2 = m_cfg;
    c2.kpPos = kpP->value();  c2.kpRot = kpR->value();
    c2.vmaxPosMmS = vP->value();  c2.vmaxRotDegS = vR->value();
    c2.accumLimitPosMm = alP->value();  c2.accumLimitRotDeg = alR->value();
    m_cfg = c2;
    QMetaObject::invokeMethod(m_worker, "applyConfig",
                              Qt::QueuedConnection, Q_ARG(AppConfig, c2));
}

QWidget *MainWindow::buildTargetPanel()
{
    auto *box = new QGroupBox("目标位姿 (BASE)　／　当前位姿", this);
    // grid 先不挂到 box：顶部要加步长选择行，统一装进一个 VBox
    auto *grid = new QGridLayout;

    grid->addWidget(new QLabel("目标", box),     0, 2, Qt::AlignHCenter);
    grid->addWidget(new QLabel("当前实际", box), 0, 3, Qt::AlignHCenter);

    for (int i = 0; i < 6; ++i) {
        const int r = i + 1;
        grid->addWidget(new QLabel(kAxisName[i], box), r, 0);

        auto *sl = new QSlider(Qt::Horizontal, box);
        sl->setRange(int(axisMin(i) * 10.0), int(axisMax(i) * 10.0));
        grid->addWidget(sl, r, 1);
        m_targetSlider[i] = sl;

        auto *sp = new QDoubleSpinBox(box);
        sp->setRange(axisMin(i), axisMax(i));
        sp->setDecimals(2);
        sp->setSingleStep(0.5);
        sp->setSuffix(axisUnit(i));
        sp->setKeyboardTracking(false);
        grid->addWidget(sp, r, 2);
        m_targetSpin[i] = sp;

        // 当前实际值紧贴目标值右侧，操作时最需要并排看的就是这一对
        auto *live = new QLabel("--", box);
        live->setMinimumWidth(95);
        live->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        live->setStyleSheet("color: #0057b8; font-weight: bold;");
        grid->addWidget(live, r, 3);
        m_liveLabel[i] = live;

        // 小步进：每轴 [-][+] 按钮，步长由顶部选择器决定
        auto *minus = new QPushButton("−", box);
        minus->setMaximumWidth(28);
        grid->addWidget(minus, r, 4);
        m_stepMinus[i] = minus;
        auto *plus = new QPushButton("＋", box);
        plus->setMaximumWidth(28);
        grid->addWidget(plus, r, 5);
        m_stepPlus[i] = plus;

        const int axis = i;
        connect(minus, &QPushButton::clicked, this, [this, axis] {
            const double step = m_stepSel->currentText().toDouble();
            m_targetSpin[axis]->setValue(m_targetSpin[axis]->value() - step);
        });
        connect(plus, &QPushButton::clicked, this, [this, axis] {
            const double step = m_stepSel->currentText().toDouble();
            m_targetSpin[axis]->setValue(m_targetSpin[axis]->value() + step);
        });

        // 滑块与数值框联动（滑块用 0.1 单位整数）
        connect(sl, &QSlider::valueChanged, this, [this, i](int v) {
            if (m_suppressTargetSignal)
                return;
            m_suppressTargetSignal = true;
            m_targetSpin[i]->setValue(v / 10.0);
            m_suppressTargetSignal = false;
            onTargetEdited();
        });
        connect(sp, &QDoubleSpinBox::valueChanged,
                this, [this, i](double v) {
            if (m_suppressTargetSignal)
                return;
            m_suppressTargetSignal = true;
            m_targetSlider[i]->setValue(int(v * 10.0));
            m_suppressTargetSignal = false;
            onTargetEdited();
        });
    }

    // 步长选择：位置 mm / 姿态 °
    auto *stepLay = new QHBoxLayout;
    stepLay->addWidget(new QLabel("步长", box));
    m_stepSel = new QComboBox(box);
    m_stepSel->addItems({"0.1", "0.5", "1", "5"});
    stepLay->addWidget(m_stepSel);
    stepLay->addStretch();

    auto *v = new QVBoxLayout(box);
    v->addLayout(stepLay);
    v->addLayout(grid);
    return box;
}

QWidget *MainWindow::buildSingularWarn()
{
    m_singularWarnLabel = new QLabel(this);
    m_singularWarnLabel->setStyleSheet(
        "color: #a06000; font-weight: bold;");
    m_singularWarnLabel->setWordWrap(true);
    m_singularWarnLabel->hide();
    return m_singularWarnLabel;
}

QWidget *MainWindow::buildParamPanel()
{
    auto *box = new QGroupBox("控制参数", this);
    auto *grid = new QGridLayout(box);
    grid->addWidget(new QLabel("位置", box), 0, 1);
    grid->addWidget(new QLabel("姿态", box), 0, 2);

    const int rows = 3;
    const char *names[rows] = {"Kp", "限速", "累积上限"};
    for (int r = 0; r < rows; ++r) {
        grid->addWidget(new QLabel(names[r], box), r + 1, 0);
        for (int c = 0; c < 2; ++c) {
            auto *val = new QLabel("--", box);
            val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            grid->addWidget(val, r + 1, c + 1);
            m_paramVal[r * 2 + c] = val;
        }
    }

    // 参数编辑移入「控制参数…」对话框：Tracking 期间对话框内参数禁用，
    // 主面板只读区仅显示生效值，由 onRefresh 随 m_cfg 刷新。
    m_paramsBtn = new QPushButton("控制参数…", box);
    connect(m_paramsBtn, &QPushButton::clicked,
            this, &MainWindow::onEditParams);
    grid->addWidget(m_paramsBtn, rows + 1, 0, 1, 3);
    return box;
}

QWidget *MainWindow::buildReadoutPanel()
{
    // 三列卡片化：当前位姿 / 目标误差 / 累积修正 各成一个 QGroupBox 并排。
    // 读数标签复用 m_actualLabel / m_errorLabel / m_accumLabel 数组，
    // onRefresh 的填充逻辑原样不动。
    auto *row = new QWidget(this);
    auto *lay = new QHBoxLayout(row);
    const char *titles[3] = {"当前位姿", "目标误差", "累积修正"};
    for (int col = 0; col < 3; ++col) {
        auto *box = new QGroupBox(titles[col], row);
        auto *g = new QGridLayout(box);
        for (int i = 0; i < 6; ++i) {
            g->addWidget(new QLabel(kAxisName[i], box), i, 0);
            auto *lab = new QLabel("--", box);
            lab->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            lab->setStyleSheet(col == 1 ? "color: #b06;"
                              : col == 2 ? "color: #068;"
                                         : "color: #0057b8; font-weight: bold;");
            g->addWidget(lab, i, 1);
            (col == 0 ? m_actualLabel[i]
                      : col == 1 ? m_errorLabel[i]
                                 : m_accumLabel[i]) = lab;
        }
        lay->addWidget(box, 1);
    }
    return row;
}

void MainWindow::onTargetEdited()
{
    // 目标面板在通信线程建立之前构建，槽比 worker 早存在一瞬。
    if (!m_worker)
        return;

    Pose t;
    t.x = m_targetSpin[0]->value();
    t.y = m_targetSpin[1]->value();
    t.z = m_targetSpin[2]->value();
    t.a = m_targetSpin[3]->value();
    t.b = m_targetSpin[4]->value();
    t.c = m_targetSpin[5]->value();
    QMetaObject::invokeMethod(m_worker, "applyTarget",
                              Qt::QueuedConnection, Q_ARG(Pose, t));

    // 欧拉奇异区：B≈±90° 时 A/C 耦合，姿态误差计算退化。仅警告，不拦截。
    if (std::fabs(m_targetSpin[4]->value()) >= 85.0) {
        m_singularWarnLabel->setText(
            QStringLiteral("⚠ 接近欧拉奇异区 (B≈±90°)，姿态控制可能退化"));
        m_singularWarnLabel->show();
    } else {
        m_singularWarnLabel->hide();
    }
}

void MainWindow::onRefresh()
{
    // 一次 snapshot()，所有字段来自同一帧。
    const StatusSnapshot s = m_state.snapshot();

    const double act[6] = {s.actual.x, s.actual.y, s.actual.z,
                           s.actual.a, s.actual.b, s.actual.c};
    const double err[6] = {s.error.x, s.error.y, s.error.z,
                           s.error.a, s.error.b, s.error.c};
    const double acc[6] = {s.accum.x, s.accum.y, s.accum.z,
                           s.accum.a, s.accum.b, s.accum.c};
    for (int i = 0; i < 6; ++i) {
        const QString cur = QString::number(act[i], 'f', 3);
        m_actualLabel[i]->setText(cur);
        m_errorLabel[i]->setText(QString::number(err[i], 'f', 3));
        m_accumLabel[i]->setText(QString::number(acc[i], 'f', 3));
        // 目标输入框旁边那一列
        m_liveLabel[i]->setText(cur);
    }

    // ── 状态卡：颜色分级，让「机器人是否真的连接/可动」一眼可判 ──
    // 7 态 ControlState 映射：Fault 红 / StaleFrame 黄（原 degraded 条件并入，
    // 另保留丢包/周期偏离的黄警告）/ Tracking+Ready 绿 / Syncing+
    // WaitingFirstFrame 蓝 / Disconnected 灰。监听中但未收到任何帧时快照仍是
    // Disconnected，此处以 m_listening 兜底保持蓝（与旧"◐ 监听中"一致）。
    QString cardText;
    bool red = false, yellow = false;
    const bool degraded =
        s.missedCount > 0 || s.krcDelay > 0
        || (s.measuredCycleMs > 0.0 && m_cfg.cycleMs > 0.0
            && std::fabs(s.measuredCycleMs - m_cfg.cycleMs)
                   > 0.10 * m_cfg.cycleMs);
    if (s.state == ControlState::Fault) {
        red = true;
        cardText = QStringLiteral("● 故障: %1").arg(s.faultReason);
    } else if (s.connected) {
        switch (s.state) {
        case ControlState::StaleFrame:
            yellow = true;
            cardText = QStringLiteral("● 已连接（注意）帧异常");
            break;
        case ControlState::Tracking:
            yellow = degraded;
            cardText = degraded ? "● 已连接（注意）  跟踪中"
                                : "● 已连接  跟踪中";
            break;
        case ControlState::Ready:
            yellow = degraded;
            cardText = degraded ? "● 已连接（注意）" : "● 已连接";
            break;
        case ControlState::Syncing:
            cardText = QStringLiteral("◐ 同步首帧");
            break;
        case ControlState::WaitingFirstFrame:
            cardText = QStringLiteral("◐ 等待首帧");
            break;
        default:   // Disconnected 不会出现在已连接快照里；保守回退
            cardText = QStringLiteral("● 已连接");
            break;
        }
    } else if (m_listening) {
        cardText = "◐ 监听中（等待 KRC 发帧）";
    } else {
        cardText = "○ 未监听";
    }
    const bool blue = s.state == ControlState::Syncing
                      || s.state == ControlState::WaitingFirstFrame
                      || (!s.connected && m_listening);
    const char *cardColor = red    ? "#c00"
                            : yellow ? "#a06000"
                            : blue   ? "#069"
                            : s.connected ? "#080"
                                          : "#888";
    m_stateCard->setText(cardText);
    m_stateCard->setStyleSheet(
        QStringLiteral("font-size: 18px; font-weight: bold; color: %1;")
            .arg(QLatin1String(cardColor)));

    // ── 详情行：诊断字段 ──
    const QString peer = s.peerIp4
                             ? QStringLiteral("%1:%2")
                                   .arg(QHostAddress(s.peerIp4).toString())
                                   .arg(s.peerPort)
                             : QStringLiteral("?");
    m_stateDetail->setText(
        QStringLiteral("KRC %1   IPOC %2   周期 %3 ms（均值 %4 / 最大 %5 / P99 %6）"
                       "   回包 %7 µs   丢包 %8 / 累计 %9   RSI Delay %10")
            .arg(peer)
            .arg(s.ipoc)
            .arg(s.measuredCycleMs, 0, 'f', 1)
            .arg(s.cycleMeanMs, 0, 'f', 2)
            .arg(s.cycleMaxMs, 0, 'f', 2)
            .arg(s.cycleP99Ms, 0, 'f', 2)
            .arg(s.maxReplyUs, 0, 'f', 0)
            .arg(s.missedCount)
            .arg(s.lifetimeLost)
            .arg(s.krcDelay));

    // ── 控制参数只读区：显示 m_cfg 生效值（编辑在「控制参数…」对话框内） ──
    m_paramVal[0]->setText(QString::number(m_cfg.kpPos, 'f', 3));
    m_paramVal[1]->setText(QString::number(m_cfg.kpRot, 'f', 3));
    m_paramVal[2]->setText(QString::number(m_cfg.vmaxPosMmS, 'f', 1));
    m_paramVal[3]->setText(QString::number(m_cfg.vmaxRotDegS, 'f', 1));
    m_paramVal[4]->setText(QString::number(m_cfg.accumLimitPosMm, 'f', 1));
    m_paramVal[5]->setText(QString::number(m_cfg.accumLimitRotDeg, 'f', 1));

    // ── 使能按钮状态机（7 态） ──
    // Fault → 「归零并复位」可点；Tracking → 「已使能跟踪」禁用；
    // Ready → 「准备跟踪」可点；其余（WaitingFirstFrame/Syncing/StaleFrame/
    // Disconnected）→ 禁用。使能必须落在已就绪的会话上，帧异常或尚未就绪
    // 时不允许启动跟踪。
    if (s.state == ControlState::Fault) {
        m_enableBtn->setText("归零并复位");
        m_enableBtn->setEnabled(true);
    } else if (s.state == ControlState::Tracking) {
        m_enableBtn->setText("已使能跟踪");
        m_enableBtn->setEnabled(false);
    } else if (s.state == ControlState::Ready) {
        m_enableBtn->setText("准备跟踪");
        m_enableBtn->setEnabled(true);
    } else {
        m_enableBtn->setText("准备跟踪");
        m_enableBtn->setEnabled(false);
    }

    m_chartPos->updateFrom(m_ring);
    m_chartRot->updateFrom(m_ring);
}

void MainWindow::onZeroToActual()
{
    // 目标置为实际、状态回 Idle、清除故障原因。
    // 【累积修正量刻意不清零】——RELATIVE 模式下 KRC 侧已施加的修正是累积的，
    // 不会因主机侧归零而消失。只有真正的 RSI 会话重启（由 RsiWorker 自己按
    // 静默时长判定）才可以调 beginSession() 清账本。界面永远不得调
    // beginSession：否则反复「停止→归零→使能」就能不断领取新的安全预算，
    // 把总修正一路推过 POSCORR 的 ~50mm 硬限。
    if (!m_worker)
        return;
    QMetaObject::invokeMethod(m_worker, "resetToActual",
                              Qt::QueuedConnection);
    const Pose a = m_state.snapshot().actual;
    m_suppressTargetSignal = true;
    const double v[6] = {a.x, a.y, a.z, a.a, a.b, a.c};
    for (int i = 0; i < 6; ++i) {
        m_targetSpin[i]->setValue(v[i]);
        m_targetSlider[i]->setValue(int(v[i] * 10.0));
    }
    m_suppressTargetSignal = false;
}

void MainWindow::onPrepareTracking()
{
    if (!m_worker)
        return;
    const StatusSnapshot s = m_state.snapshot();

    if (s.state == ControlState::Fault) {
        // 故障锁存：必须归零并复位才能重新使能
        QMetaObject::invokeMethod(m_worker, "resetToActual",
                                  Qt::QueuedConnection);
        return;   // 按钮文本由 onRefresh 统一刷新
    }

    // 联锁：硬拦截无覆盖
    const QStringList blocked =
        SessionGuard::enableChecks(m_cfg, s.measuredCycleMs);
    if (!blocked.isEmpty()) {
        m_interlockLabel->setText(QStringLiteral("使能被拦截：\n")
                                  + blocked.join(QLatin1Char('\n')));
        m_interlockLabel->show();
        return;
    }
    m_interlockLabel->hide();

    // 两阶段确认：操作员核对 BASE/TOOL、目标位姿、限值余量
    const Pose t = s.target;
    const QMessageBox::StandardButton r = QMessageBox::question(
        this, "确认使能跟踪",
        QStringLiteral(
            "使能前请确认：\n"
            "1. 示教器当前 BASE / TOOL 正确\n"
            "2. 目标位姿符合预期\n"
            "3. 限值余量足够（累积 %1 / 上限 %2）\n\n"
            "当前目标: X %3  Y %4  Z %5  A %6  B %7  C %8\n\n"
            "继续？")
            .arg(s.accum.x, 0, 'f', 1)
            .arg(m_cfg.accumLimitPosMm, 0, 'f', 1)
            .arg(t.x, 0, 'f', 1).arg(t.y, 0, 'f', 1)
            .arg(t.z, 0, 'f', 1).arg(t.a, 0, 'f', 1)
            .arg(t.b, 0, 'f', 1).arg(t.c, 0, 'f', 1),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (r != QMessageBox::Yes)
        return;

    QMetaObject::invokeMethod(m_worker, "setTracking",
                              Qt::QueuedConnection, Q_ARG(bool, true));
}

void MainWindow::onStopTracking()
{
    // 软停止，不是急停：把目标拉回实际使误差归零、机器人停在原地，
    // 但通信线程一个周期都不停地继续回包。停止回包会让 RSI 判定通信故障
    // 并直接错误停机（第 4 层），那不是"停下"而是"摔停"。
    // 真正的急停只有示教器上的物理按钮，界面上的红字提示说的就是这件事。
    // 软停止走 onZeroToActual（目标归零 → 误差零 → 停），按钮状态由 onRefresh 管。
    onZeroToActual();
}
