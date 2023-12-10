// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <gui/generator/generator_mainwindow.h>
#include <QApplication>
#include <QMainWindow>


int main(int argc, char *argv[])
{
    // 设置应用程序的 DPI 适应性，这可以使应用程序在高 DPI 屏幕上看起来更清晰
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    // 初始化 QT app
    QApplication app(argc, argv);
    // 统一设置为 Fusion 样式
    app.setStyle("Fusion");
    // 开启 generator 主窗口
    GeneratorMainWindow *generator = new GeneratorMainWindow();
    generator->exec();
    // 执行 QT 应用
    app.exit();
    return 0; // 主事件循环
}