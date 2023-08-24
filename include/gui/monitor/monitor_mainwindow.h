#ifndef MONITORMAINWINDOW_H
#define MONITORMAINWINDOW_H
#include <QMainWindow>
#include <QApplication>
#include <QDialog>
#include <QMessageBox>
#include <QLabel>
#include <QFrame>
#include <QIcon>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QTableView>
#include <QPushButton>
#include <QHeaderView>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QString>
#include <QStringList>
#include <string>
#include <vector>
#include <iostream>
#include <sys/stat.h>

#include <ros/ros.h>

#include <print_utility/printf_utility.h>
#include <qcustomplot.h>

using namespace std;

class MonitorMainWindow : public QWidget
{

    public:
        QDialog *win = new QDialog();
        QWidget *parent;
        MonitorMainWindow(QWidget *parent_widget, QStringList nodes_input)
        {
            nodes = nodes_input;
            setup();
        }

    private:
        // settings
        string version = "V1.0.0";
        int local_port = 34580;
        int remote_port = 24540;
        int sitl_port = 18570;
        int tcp_port = 4560;
        vector<string> topic_types = {"{Vehicle Type}_{ID}", "uav_{ID}"};
        vector<string> vehicle_types = {"iris", "typhoon_h480", "plane"};
        vector<string> sensor_types = {"None", "Lidar", "Depth Camera", "RGB Camera", "Stereo Camera", "Realsense Camera"};
        vector<string> table_headers = {"Vehicle", "Sensor", "x", "y", "z", "R", "P", "Y"};

        // init vector
        QStringList nodes;
        vector<string> vehicles = {};
        vector<string> sensors = {};
        vector<vector<string>> init_pos;
        vector<string> world_files = {};
        // output file string
        string output_file = "";
        bool update_signal = false;

        //Widgets
        QMessageBox *msg_box;

        void setup()
        {
            QIcon *icon = new QIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
            win->setWindowIcon(*icon);
            win->setFixedSize(1080, 750);
            win->setWindowTitle(("PX4 Cmd Simulation Monitor [Version: " + version + "]").c_str());
            win->setStyleSheet("background-color: rgb(255,250,250)");
        }
};

#endif