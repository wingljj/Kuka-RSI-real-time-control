#include <QApplication>
#include <QLabel>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    QLabel label("RSI host skeleton");
    label.resize(320, 120);
    label.show();
    return app.exec();
}
