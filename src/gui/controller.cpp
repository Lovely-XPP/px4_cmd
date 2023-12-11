// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <gui/controller/controller_mainwindow.h>
#include <gui/controller/controller_loadwindow.h>
#include "gui/controller/moc_controller_mainwindow.cpp"
#include "gui/controller/moc_controller_modewindow.cpp"
#include "gui/controller/moc_controller_takeoffwindow.cpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "controller");
    // 设置应用程序的 DPI 适应性，这可以使应用程序在高 DPI 屏幕上看起来更清晰
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    // 初始化 QT app
    QApplication app(argc, argv);
    // 统一设置为 Fusion 样式
    app.setStyle("Fusion");
    // 开启加载窗口
    ControllerLoadWindow *load_win = new ControllerLoadWindow();
    load_win->exec();
    // 获取节点
    QStringList nodes = load_win->nodes;
    // 如果点击了开始按钮则开启主控制窗口
    if (load_win->push_button)
    {
        // 回收内存
        delete load_win;
        // 开启 controller 主窗口
        ControllerMainWindow *controller = new ControllerMainWindow(nodes);
        controller->exec();
    }
    // 退出 QT 应用
    app.exit();
    return 0;
}