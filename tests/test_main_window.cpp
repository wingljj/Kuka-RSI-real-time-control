// MainWindow 的控件初始状态单测。
//
// 为什么这一条必须在真实窗口上测、而不是测 uilogic::buttonStates：
// 被锁住的缺陷不在判定里。buttonStates 在默认（Disconnected、未监听）快照下
// 本来就返回 resetFault / enableTrack / stopTrack 全假——纯函数测它一直是绿的。
// 出错的是「判定结果有没有被施加到控件上」：这三个动作从 QPushButton 改成
// QAction 时，构造期那两句 setEnabled(false) 被删掉且无替代，而 QAction 默认
// 是启用的，updateConnControls 又只施加了 8 项里的 5 项。于是从构造完成到第一
// 次 onRefresh 之间（一个 refresh_ms），未监听状态下「复位故障 / 使能跟踪 /
// 停止跟踪」连同 F9 / Esc 快捷键全部可点。
// 只有查真实 QAction 的 isEnabled() 才能发现这种「算对了但没接线」的缺陷。

#include <QtTest>
#include <QAction>
#include <QLineEdit>
#include <QSpinBox>
#include <QSettings>
#include <QTemporaryDir>
#include "core/AppConfig.h"
#include "ui/MainWindow.h"

class TestMainWindow : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase()
    {
        // MainWindow 在构造期读、在析构期写 QSettings。默认格式在 Windows 上是
        // 注册表，跑一次测试就会把操作员保存的窗口布局覆盖成 offscreen 平台下
        // 的几何。改成 Ini 并指到临时目录，测试对真实配置零副作用。
        QVERIFY(m_settingsDir.isValid());
        QSettings::setDefaultFormat(QSettings::IniFormat);
        QSettings::setPath(QSettings::IniFormat, QSettings::UserScope,
                           m_settingsDir.path());
    }

    // 构造完成、第一次 onRefresh 之前，三个控制动作必须是禁用的。
    //
    // 刻意不跑事件循环：refresh 定时器和 worker 的 listening 信号都要等事件
    // 循环才会到达，所以窗口此刻正处于「刚构造好、还没刷新过」这个窗口期内，
    // 也就是缺陷现场。跑了 QTest::qWait 就测不到这一条了。
    void controlActionsStartDisabled()
    {
        AppConfig cfg;
        // 不用默认的 192.168.44.1：测试机上没有那张网卡，绑定会失败。
        // 绑不绑上都不影响本条断言（listening 是跨线程排队信号，不跑事件循环
        // 就不会到达，m_listening 恒为 false），但换成回环少一次噪声。
        cfg.listenIp = "127.0.0.1";
        cfg.listenPort = 0;          // 0 = 让 OS 挑空闲端口，避免与并行测试撞车

        MainWindow w(cfg);

        auto *resetFault = w.findChild<QAction *>("resetFaultAct");
        auto *enableTrack = w.findChild<QAction *>("enableTrackAct");
        auto *stopTrack = w.findChild<QAction *>("stopTrackAct");
        QVERIFY(resetFault);
        QVERIFY(enableTrack);
        QVERIFY(stopTrack);

        // 未连接时复位故障没有故障可复位；未就绪时使能跟踪会被 PoseController
        // 拒绝（点了没反应，操作员会以为程序卡死）；没有跟踪时停止跟踪无意义。
        QVERIFY(!resetFault->isEnabled());
        QVERIFY(!enableTrack->isEnabled());
        QVERIFY(!stopTrack->isEnabled());
    }

    // 同一个窗口期内其余五项也必须已经施加，否则「只施加一部分字段」这个
    // 缺陷换个字段就会复现。开始监听可用、停止监听禁用、IP/端口可编辑
    //（未监听），「读取当前值」禁用（frameCount 为 0，actual 还是全零）。
    void everyButtonStateFieldIsAppliedAtConstruction()
    {
        AppConfig cfg;
        cfg.listenIp = "127.0.0.1";
        cfg.listenPort = 0;

        MainWindow w(cfg);

        auto *startListen = w.findChild<QAction *>("startListenAct");
        auto *stopListen = w.findChild<QAction *>("stopListenAct");
        auto *ipEdit = w.findChild<QLineEdit *>("listenIpEdit");
        auto *portSpin = w.findChild<QSpinBox *>("listenPortSpin");
        auto *readActual = w.findChild<QWidget *>("readActualBtn");
        QVERIFY(startListen);
        QVERIFY(stopListen);
        QVERIFY(ipEdit);
        QVERIFY(portSpin);
        QVERIFY(readActual);

        QVERIFY(startListen->isEnabled());
        QVERIFY(!stopListen->isEnabled());
        QVERIFY(ipEdit->isEnabled());
        QVERIFY(portSpin->isEnabled());
        // frameCount 为 0：actual 是全零，抄进目标框再「应用目标」就是让
        // 机器人朝 BASE 原点走。
        QVERIFY(!readActual->isEnabled());
    }

private:
    QTemporaryDir m_settingsDir;
};

QTEST_MAIN(TestMainWindow)
#include "test_main_window.moc"
