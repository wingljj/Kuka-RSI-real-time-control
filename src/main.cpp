#include <QApplication>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QMetaType>
#include "core/AppConfig.h"
#include "core/Pose.h"
#include "ui/MainWindow.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

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

    MainWindow w(cfg);
    w.show();
    return app.exec();
}
