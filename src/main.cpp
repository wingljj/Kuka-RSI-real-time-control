#include <QApplication>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QMetaType>
#include <QSharedMemory>
#include "core/AppConfig.h"
#include "core/Pose.h"
#include "ui/MainWindow.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    // 单实例保护。Qt 的 QUdpSocket 默认带 ShareAddress，两个进程能同时
    // 绑定同一个 UDP 端口而不报错——结果 UDP 包被 OS 随机分配给其中
    // 一个，另一个因收不到帧而看门狗反复触发：connected 在已连接/未监听
    // 之间抖动、ring 清空导致图表闪烁。这也正是用户报告的间歇性断连重连。
    // QSharedMemory 在进程退出时由 OS 自动回收，崩溃也不会残留锁。
    static QSharedMemory singleton("kuka_rsi_host_instance");
    if (!singleton.create(1)) {
        QMessageBox::critical(nullptr, "已在运行",
            "rsi_host 已经有一个实例在运行。\n\n"
            "请先关闭已打开的窗口，再重新启动。\n\n"
            "如果确定没有别的实例，请打开任务管理器\n"
            "结束所有 rsi_host.exe 进程后重试。");
        return 1;
    }

    // Pose / AppConfig 以字符串名 invokeMethod + Q_ARG 跨线程排队传递。
    // Q_DECLARE_METATYPE 只声明类型，显式注册是对"队列连接静默失败"的保险，
    // 那种失败的表象是"机器人不理会界面"，极难定位。
    qRegisterMetaType<Pose>();
    qRegisterMetaType<AppConfig>();

    AppConfig cfg = AppConfig::defaults();

    // 按顺序找配置：先 exe 同级的 config/（打包分发时 exe 与 config 并列），
    // 再上一级的 config/（开发树里 exe 在 build/ 下）。
    // 找不到不是致命错误——内置默认值是安全的一组，但要让用户知道用的是它们，
    // 否则界面上显示的限值与他以为在编辑的那份文件毫无关系。
    const QDir appDir(QCoreApplication::applicationDirPath());
    const QStringList candidates{
        appDir.filePath("config/rsi_config.json"),
        appDir.filePath("../config/rsi_config.json"),
    };

    bool loaded = false;
    QStringList tried;
    for (const QString &path : candidates) {
        if (!QFile::exists(path))
            continue;
        QString err;
        if (AppConfig::loadFromFile(path, &cfg, &err)) {
            loaded = true;
            break;
        }
        tried << QStringLiteral("%1\n    %2").arg(path, err);
    }

    if (!loaded) {
        if (tried.isEmpty())
            for (const QString &p : candidates)
                tried << QStringLiteral("%1\n    文件不存在").arg(p);
        QMessageBox::warning(nullptr, "配置",
            QStringLiteral("未能加载配置，已回退到内置默认值。\n\n"
                           "尝试过：\n%1\n\n"
                           "注意内置默认的累积上限是 30mm / 15°，"
                           "可能与你配置文件里的值不同。")
                .arg(tried.join("\n")));
    }

    // 刻意不加载任何样式表：界面用系统原生外观。硬编码的浅色背景会盖掉
    // 操作员在系统层面设的高对比度主题（工业现场的视觉辅助设置），
    // 而样式表选择器还会级联到后代部件——本次审查里 "QFrame{...}"
    // 污染状态卡片内每个 QLabel 就是这么来的。语义色改由
    // uilogic::severityColor 经 QPalette 施加，不级联。

    MainWindow w(cfg);
    w.show();
    return app.exec();
}
