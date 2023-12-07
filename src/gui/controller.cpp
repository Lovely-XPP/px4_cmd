#include <gui/controller/controller_mainwindow.h>
#include <gui/controller/controller_loadwindow.h>
#include "gui/controller/moc_controller_mainwindow.cpp"
#include "gui/controller/moc_controller_modewindow.cpp"
#include "gui/controller/moc_controller_takeoffwindow.cpp"
#include <QApplication>
#include <QMainWindow>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "controller");
    // 设置应用程序的 DPI 适应性，这可以使应用程序在高 DPI 屏幕上看起来更清晰
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    QMainWindow *main_win;
    ControllerLoadWindow *load_win = new ControllerLoadWindow(main_win);
    load_win->exec();
    QStringList nodes = load_win->nodes;
    delete load_win;
    if (load_win->push_button)
    {
        ControllerMainWindow *controller = new ControllerMainWindow(main_win, nodes);
        controller->exec(); // 主事件循环
    }
    usleep(200000);
    return 0;
}