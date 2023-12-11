// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <gui/monitor/monitor_loadwindow.h>
#include <gui/monitor/monitor_mainwindow.h>
#include "gui/monitor/moc_monitor_mainwindow.cpp"
#include "gui/monitor/moc_monitor_imagewindow.cpp"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "monitor");
    // 设置应用程序的 DPI 适应性，这可以使应用程序在高 DPI 屏幕上看起来更清晰
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    // 初始化 QT app
    QApplication app(argc, argv);
    // 统一设置为 Fusion 样式
    app.setStyle("Fusion");
    // 开启加载窗口
    MonitorLoadWindow *load_win = new MonitorLoadWindow();
    load_win->exec();
    // 获取节点
    QStringList nodes = load_win->nodes;
    // 如果点击了开始按钮则开启主控制窗口
    if (load_win->push_button)
    {
        // 删除内存
        delete load_win;
        // 开启 monitor 主窗口
        MonitorMainWindow *monitor = new MonitorMainWindow(nodes);
        monitor->exec();
    }
    // 退出 QT 应用
    app.exit();
    return 0;
}