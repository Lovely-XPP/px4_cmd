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
#include <thread>

#include <ros/ros.h>

#include <print_utility/printf_utility.h>
#include <qcustomplot.h>
#include <vehicle.h>

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
        vector<string> table_headers_pos = {"Vehicle", "Sensor", "x", "y", "z", "roll", "pitch", "yaw"};
        QVector<vehicle*> data;

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
            // get data
            for (auto item = nodes.begin(); item != nodes.end(); item++)
            {
                vehicle *vec = new vehicle();
                vec->set_node_name((*item).toStdString());
                data.push_back(vec);
            }

            // add plot
            plot = new QCustomPlot(win);
            hbox->addWidget(plot, 1);

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
            hbox->addWidget(table_pos, 2);
            vbox->addLayout(hbox);
            win->setLayout(vbox);

            std::thread table_thread(&MonitorMainWindow::update_table, this);
            table_thread.detach();
        }


        void update_table()
        {
            QStringList list;
            QStandardItem *item_1;
            QStandardItem *item_2;
            QStandardItem *item_3;
            QStandardItem *item_4;
            QStandardItem *item_5;
            QStandardItem *item_6;
            QStandardItem *item_7;
            QStandardItem *item_8;
            int i;
            while (ros::ok())
            {
                i = 0;
                for (auto item = data.begin(); item != data.end(); item++)
                {
                    item_1 = new QStandardItem();
                    item_2 = new QStandardItem();
                    item_3 = new QStandardItem();
                    item_4 = new QStandardItem();
                    item_5 = new QStandardItem();
                    item_6 = new QStandardItem();
                    item_7 = new QStandardItem();
                    item_8 = new QStandardItem();
                    item_1->setText((*item)->vehicle_name.c_str());
                    item_2->setText((*item)->sensor_name.c_str());
                    item_3->setText(to_string((*((*item)->x.end()-1))).c_str());
                    item_4->setText(to_string((*((*item)->y.end()-1))).c_str());
                    item_5->setText(to_string((*((*item)->z.end()-1))).c_str());
                    item_6->setText(to_string((*((*item)->roll.end()-1))).c_str());
                    item_7->setText(to_string((*((*item)->pitch.end()-1))).c_str());
                    item_8->setText(to_string((*((*item)->yaw.end()-1))).c_str());
                    item_1->setEditable(false);
                    item_2->setEditable(false);
                    model_pos->setItem(i, 0, item_1);
                    model_pos->setItem(i, 1, item_2);
                    model_pos->setItem(i, 2, item_3);
                    model_pos->setItem(i, 3, item_4);
                    model_pos->setItem(i, 4, item_5);
                    model_pos->setItem(i, 5, item_6);
                    model_pos->setItem(i, 6, item_7);
                    model_pos->setItem(i, 7, item_8);
                    i = i + 1;
                }
                table_pos->setModel(model_pos);
                table_pos->horizontalHeader()->setStretchLastSection(true);
                table_pos->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
                sleep(1);
            }
        }
};

#endif