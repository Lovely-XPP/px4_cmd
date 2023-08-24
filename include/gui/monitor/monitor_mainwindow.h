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
//#include <vehicle.h>

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
        int local_port = 14580;
        int remote_port = 14540;
        int sitl_port = 18570;
        int tcp_port = 4560;
        vector<string> topic_types = {"{Vehicle Type}_{ID}", "uav_{ID}"};
        vector<string> vehicle_types = {"iris", "typhoon_h480", "plane"};
        vector<string> sensor_types = {"None", "Lidar", "Depth Camera", "RGB Camera", "Stereo Camera", "Realsense Camera"};
        vector<string> table_headers_pos = {"Vehicle", "Sensor", "x", "y", "z", "pitch", "roll", "yaw"};

        // init vector
        QStringList nodes;
        vector<string> vehicles = {};
        vector<string> sensors = {};
        
        struct plot_data
        {
            QVector<double> x;
            QVector<double> y;
        };

        // output file string
        string output_file = "";
        bool update_signal = false;

        //Widgets
        QMessageBox *msg_box;
        QCustomPlot *plot;
        QTableView *table_pos;
        QStandardItemModel *model_pos;
        QHBoxLayout *hbox = new QHBoxLayout();
        QVBoxLayout *vbox = new QVBoxLayout();

        void setup()
        {
            QIcon *icon = new QIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
            win->setWindowIcon(*icon);
            win->setFixedSize(1080, 750);
            win->setWindowTitle(("PX4 Cmd Simulation Monitor [Version: " + version + "]").c_str());
            win->setStyleSheet("background-color: rgb(255,250,250)");

            // add plot
            plot = new QCustomPlot(win);
            vbox->addWidget(plot);

            // add table
            table_pos = new QTableView(win);
            table_pos->setStyleSheet("background-color: white");
            model_pos = new QStandardItemModel(0, table_headers_pos.size(), win);
            QStringList list_pos;
            for (auto item = table_headers_pos.begin(); item != table_headers_pos.end(); item++)
            {
                list_pos.append(&(*item->c_str()));
            }
            model_pos->setHorizontalHeaderLabels(list_pos);
            table_pos->setModel(model_pos);
            table_pos->horizontalHeader()->setStretchLastSection(true);
            table_pos->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
            vbox->addWidget(table_pos, 1);
            win->setLayout(vbox);
        }
};

#endif