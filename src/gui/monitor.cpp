#include <gui/monitor/monitor_loadwindow.h>
#include <gui/monitor/monitor_mainwindow.h>
#include "gui/monitor/moc_monitor_mainwindow.cpp"
#include <QApplication>
#include <QMainWindow>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "monitor");
    // 设置应用程序的 DPI 适应性，这可以使应用程序在高 DPI 屏幕上看起来更清晰
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    QMainWindow *main_win;
    MonitorLoadWindow *load_win = new MonitorLoadWindow(main_win);
    load_win->exec();
    QStringList nodes = load_win->nodes;
    delete load_win;
    if (load_win->push_button)
    {
        MonitorMainWindow *monitor = new MonitorMainWindow(main_win, nodes);
        return monitor->exec(); // 主事件循环
    }
    sleep(1);
    return 0;
}