#include <QApplication>
#include <QDir>
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
    const QString path =
        QDir(QCoreApplication::applicationDirPath())
            .filePath("../config/rsi_config.json");
    QString err;
    if (!AppConfig::loadFromFile(path, &cfg, &err)) {
        QMessageBox::warning(nullptr, "配置",
            QStringLiteral("未能加载 %1\n%2\n\n将使用内置默认值。")
                .arg(path, err));
    }

    MainWindow w(cfg);
    w.show();
    return app.exec();
}
