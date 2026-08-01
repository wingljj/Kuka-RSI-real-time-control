#include "ui/MainWindow.h"

#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QPushButton>
#include <QScreen>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QMessageBox>
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

    m_statusLabel = new QLabel("未连接", this);
    m_statusLabel->setStyleSheet("font-weight: bold;");

    auto *central = new QWidget(this);
    auto *outer = new QVBoxLayout(central);
    outer->addWidget(m_statusLabel);
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

    // 两段式使能：构造时不勾选，连接本身不会启动运动。操作员先看读数是否
    // 合理，再手动勾选。任何"连上就自动使能"的便利都不可接受。
    m_trackCheck = new QCheckBox("使能跟踪", this);
    connect(m_trackCheck, &QCheckBox::toggled,
            this, &MainWindow::onTrackingToggled);
    bar->addWidget(m_trackCheck);

    auto *stopBtn = new QPushButton("停止跟踪", this);
    connect(stopBtn, &QPushButton::clicked,
            this, &MainWindow::onStopTracking);
    bar->addWidget(stopBtn);

    bar->addStretch();
    m_safetyNote = new QLabel(
        "「停止跟踪」是软停止，不是急停。急停只能用示教器上的物理急停按钮。",
        this);
    m_safetyNote->setStyleSheet("color: #b00; font-weight: bold;");
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
    left->addWidget(buildParamPanel());
    left->addStretch();
    row->addLayout(left, 1);
    auto *right = new QVBoxLayout;
    m_chart = new ErrorChart(m_cfg.chartWindowS, this);
    right->addWidget(m_chart, 2);
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
    // socket 一关就不可能再有回包，跟踪也就无从谈起。让勾选框如实反映这点，
    // 而不是留一个打着勾的控件覆盖一台已经断开的机器。
    if (m_trackCheck)
        m_trackCheck->setChecked(false);
    updateConnControls();
}

QWidget *MainWindow::buildTargetPanel()
{
    auto *box = new QGroupBox("目标位姿 (BASE)　／　当前位姿", this);
    auto *grid = new QGridLayout(box);

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
    return box;
}

QWidget *MainWindow::buildParamPanel()
{
    auto *box = new QGroupBox("控制参数", this);
    auto *grid = new QGridLayout(box);
    grid->addWidget(new QLabel("位置", box), 0, 1);
    grid->addWidget(new QLabel("姿态", box), 0, 2);

    struct Row { const char *name; double pos, rot, step; int dec; };
    const Row rows[3] = {
        {"Kp",       m_cfg.kpPos,           m_cfg.kpRot,            0.01, 2},
        {"限速",     m_cfg.vmaxPosMmS,      m_cfg.vmaxRotDegS,      1.0,  1},
        {"累积上限", m_cfg.accumLimitPosMm, m_cfg.accumLimitRotDeg, 1.0,  1},
    };

    for (int r = 0; r < 3; ++r) {
        grid->addWidget(new QLabel(rows[r].name, box), r + 1, 0);
        for (int c = 0; c < 2; ++c) {
            auto *sp = new QDoubleSpinBox(box);
            sp->setRange(0.0, 100000.0);
            sp->setDecimals(rows[r].dec);
            sp->setSingleStep(rows[r].step);
            sp->setValue(c == 0 ? rows[r].pos : rows[r].rot);
            sp->setKeyboardTracking(false);
            grid->addWidget(sp, r + 1, c + 1);

            connect(sp, &QDoubleSpinBox::valueChanged,
                    this, [this, r, c](double v) {
                AppConfig c2 = m_cfg;
                if (r == 0) (c == 0 ? c2.kpPos : c2.kpRot) = v;
                if (r == 1) (c == 0 ? c2.vmaxPosMmS : c2.vmaxRotDegS) = v;
                if (r == 2) (c == 0 ? c2.accumLimitPosMm
                                    : c2.accumLimitRotDeg) = v;
                m_cfg = c2;
                if (!m_worker)
                    return;
                // 必须排队：直连会在通信线程读 m_cfg.senType 的同时改写它。
                QMetaObject::invokeMethod(m_worker, "applyConfig",
                                          Qt::QueuedConnection,
                                          Q_ARG(AppConfig, c2));
            });
        }
    }
    return box;
}

QWidget *MainWindow::buildReadoutPanel()
{
    auto *box = new QGroupBox("读数", this);
    auto *grid = new QGridLayout(box);
    grid->addWidget(new QLabel("当前位姿", box), 0, 1);
    grid->addWidget(new QLabel("误差",     box), 0, 2);
    grid->addWidget(new QLabel("累积修正", box), 0, 3);

    for (int i = 0; i < 6; ++i) {
        grid->addWidget(new QLabel(kAxisName[i], box), i + 1, 0);
        m_actualLabel[i] = new QLabel("--", box);
        m_errorLabel[i]  = new QLabel("--", box);
        m_accumLabel[i]  = new QLabel("--", box);
        grid->addWidget(m_actualLabel[i], i + 1, 1);
        grid->addWidget(m_errorLabel[i],  i + 1, 2);
        grid->addWidget(m_accumLabel[i],  i + 1, 3);
    }
    return box;
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

    // 三态而不是两态：「已绑定但一帧未收」和「根本没绑上」对操作员是两件
    // 完全不同的事——前者该去看 KRC 那边的 KRL 程序有没有跑起来、地址端口
    // 对不对；后者是本机的绑定就失败了，多半端口被占。原先合并成「未连接」
    // 会把人指向错误的排查方向。
    QString st;
    if (s.connected)
        st = "● 已连接";
    else if (m_listening)
        st = "◐ 监听中（等待 KRC 发帧）";
    else
        st = "○ 未监听";
    if (s.state == TrackState::Tracking)
        st += "  跟踪中";
    else if (s.state == TrackState::Fault)
        st += "  故障: " + s.faultReason;
    st += QStringLiteral("   IPOC %1   周期 %2 ms   最大回包 %3 µs"
                         "   丢包 %4   KRC丢包 %5   异源 %6   发送失败 %7")
              .arg(s.ipoc)
              .arg(s.measuredCycleMs, 0, 'f', 1)
              .arg(s.maxReplyUs, 0, 'f', 0)
              .arg(s.missedCount)
              .arg(s.krcDelay)
              .arg(s.peerRejected)
              .arg(s.sendFails);
    // Fault 是锁存的：PoseController::setTracking(true) 在 Fault 下不会转
    // Tracking，必须先经「归零到当前位姿」清除。所以此时勾选框若还打着勾，
    // 显示的就是一个与机器状态不符的谎言——强制取消勾选。
    if (s.state == TrackState::Fault && m_trackCheck->isChecked())
        m_trackCheck->setChecked(false);

    m_statusLabel->setText(st);

    m_chart->updateFrom(m_ring);
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
    m_trackCheck->setChecked(false);
}

void MainWindow::onTrackingToggled(bool on)
{
    if (!m_worker)
        return;
    if (on) {
        // 联锁：硬拦截无覆盖。不通过就不置勾，红字列出全部原因。
        const StatusSnapshot s = m_state.snapshot();
        const QStringList blocked =
            SessionGuard::enableChecks(m_cfg, s.measuredCycleMs);
        if (!blocked.isEmpty()) {
            m_trackCheck->blockSignals(true);
            m_trackCheck->setChecked(false);
            m_trackCheck->blockSignals(false);
            m_interlockLabel->setText(QStringLiteral("使能被拦截：\n")
                                      + blocked.join(QLatin1Char('\n')));
            m_interlockLabel->show();
            return;
        }
        m_interlockLabel->hide();
    }
    // 必须排队：直连会在通信线程 step() 读状态的同时改写它。
    QMetaObject::invokeMethod(m_worker, "setTracking",
                              Qt::QueuedConnection, Q_ARG(bool, on));
}

void MainWindow::onStopTracking()
{
    // 软停止，不是急停：把目标拉回实际使误差归零、机器人停在原地，
    // 但通信线程一个周期都不停地继续回包。停止回包会让 RSI 判定通信故障
    // 并直接错误停机（第 4 层），那不是"停下"而是"摔停"。
    // 真正的急停只有示教器上的物理按钮，界面上的红字提示说的就是这件事。
    m_trackCheck->setChecked(false);
    onZeroToActual();
}
