#include <gui/controller/controller_generatewindow.h>
#include <QApplication>
#include <QMainWindow>

int main(int argc, char *argv[])
{
    // 设置应用程序的 DPI 适应性，这可以使应用程序在高 DPI 屏幕上看起来更清晰
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    QMainWindow *main_win;
    ControllerGenerateWindow *win = new ControllerGenerateWindow(main_win);
    return win->win->exec(); // 主事件循环
}