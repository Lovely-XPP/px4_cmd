#ifndef MONITORMAINWINDOW_H
#define MONITORMAINWINDOW_H
#include <QMainWindow>
#include <QApplication>
#include <QMetaType>
#include <QDialog>
#include <QMessageBox>
#include <QLabel>
#include <QFrame>
#include <QIcon>
#include <QVector>
#include <QPen>
#include <QColor>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QTableView>
#include <QPushButton>
#include <QHeaderView>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QAbstractItemModel>
#include <QPersistentModelIndex>
#include <QString>
#include <QStringList>
#include <string>
#include <vector>
#include <thread>

#include <ros/ros.h>

#include <gui/monitor/monitor_infowindow.h>
#include <qcustomplot.h>
#include <vehicle_state.h>

#define PI 3.14159265358979323846

using namespace std;

class MonitorMainWindow : public QWidget
{
    public:
        QDialog *win = new QDialog();
        MonitorInfoWindow *info_win = new MonitorInfoWindow(win);
        QWidget *parent;
        MonitorMainWindow(QWidget *parent_widget, QStringList nodes_input)
        {
            qRegisterMetaType<QList<QPersistentModelIndex>>("QList<QPersistentModelIndex>");
            qRegisterMetaType<QList<QAbstractItemModel::LayoutChangeHint>>("QList<QAbstractItemModel::LayoutChangeHint>");
            nodes = nodes_input;
            setup();
        }

    private:
        // settings
        string version = "V1.0.0";
        double update_time = 0.3;
        vector<string> table_headers_pos = {"Vehicle", "Sensor", "Mode", "x", "y", "z", "vx", "vy", "vz", "roll (deg)", "pitch (deg)", "yaw (deg)"};
        vector<string> table_headers_topic = {"Node", "Sensor", "Senor Topic"};

        // init vector
        QStringList nodes;
        QVector<double> x_end = {0};
        QVector<double> y_end = {0};
        QStringList topics;
        QVector<vehicle_state *> data;
        QVector<std::thread *> threads;
        bool thread_stop = false;

        // output file string
        string output_file = "";
        bool update_signal = false;

        //Widgets
        QMessageBox *msg_box;
        QCustomPlot *plot;
        QTableView *table_pos;
        QTableView *table_topic;
        QLabel *info_label;
        QStandardItemModel *model_pos;
        QStandardItemModel *model_topic;
        QPushButton *signal_button;
        QPushButton *info_button;
        QPushButton *exit_button;

        void setup()
        {
            // layout
            QHBoxLayout *hbox = new QHBoxLayout();
            QVBoxLayout *vbox = new QVBoxLayout();
            QIcon *icon = new QIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
            win->setWindowIcon(*icon);
            win->setFixedSize(1280, 750);
            win->setWindowTitle(("PX4 Cmd Simulation Monitor [Version: " + version + "]").c_str());
            win->setStyleSheet("background-color: rgb(255,250,250)");
            // get data
            for (auto item = nodes.begin(); item != nodes.end(); item++)
            {
                vehicle_state *vec = new vehicle_state();
                vec->get_state((*item).toStdString());
                data.push_back(vec);
            }

            // add label
            QLabel *topic_label = new QLabel("[ Vehicle Sensor Topics Information ]", win);
            QLabel *pos_label = new QLabel("[ Vehicle Position & Pose Information ]", win);
            QLabel *plot_label = new QLabel("[ Vehicle Position 2D Plane Plot (x-y Plane) ]", win);
            info_label = new QLabel(("[ROS State]  Running\n[Vehicle Count]  " + to_string(nodes.size())).c_str(), win);
            info_label->setStyleSheet("color: green; font-size: 14pt");
            topic_label->setStyleSheet("color: rgb(255,137,46); font-size: 14pt; font-weight: bold");
            pos_label->setStyleSheet("color: rgb(120,200,200); font-size: 14pt; font-weight: bold");
            plot_label->setStyleSheet("color: black; font-size: 14pt; font-weight: bold");

            // add button
            signal_button = new QPushButton(win);
            signal_button->setFixedSize(1, 1);
            //info
            info_button = new QPushButton("About", win);
            info_button->setMinimumHeight(50);
            info_button->setStyleSheet("background-color: rgb(255,227,132); color: black; font-size: 16pt; font-weight: bold");
            exit_button = new QPushButton("Exit", win);
            exit_button->setMinimumHeight(50);
            exit_button->setStyleSheet("background-color: rgb(255,106,106); color: black; font-size: 16pt; font-weight: bold");

            // add table
            // topic
            table_topic = new QTableView(win);
            table_topic->setStyleSheet("background-color: rgb(255,245,235)");
            model_topic = new QStandardItemModel(data.size(), table_headers_topic.size(), win);
            QStringList list_topic;
            QString node_name;
            QString sensor_name;
            int topic_count = 0;
            for (auto item = table_headers_topic.begin(); item != table_headers_topic.end(); item++)
            {
                list_topic.append(&(*item->c_str()));
            }
            model_topic->setHorizontalHeaderLabels(list_topic);
            for (auto node_item = data.begin(); node_item != data.end(); node_item++)
            {
                node_name = (*node_item)->node_name.c_str();
                sensor_name = (*node_item)->sensor_name.c_str();
                for (auto topic_item = (*node_item)->sensor_topics.begin(); topic_item != (*node_item)->sensor_topics.end(); topic_item++)
                {
                    QStandardItem *item_1 = new QStandardItem();
                    QStandardItem *item_2 = new QStandardItem();
                    QStandardItem *item_3 = new QStandardItem();
                    item_1->setEditable(false);
                    item_2->setEditable(false);
                    item_3->setEditable(false);
                    item_1->setTextAlignment(Qt::AlignCenter);
                    item_2->setTextAlignment(Qt::AlignCenter);
                    item_3->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
                    item_1->setText(node_name);
                    item_2->setText(sensor_name);
                    item_3->setText(*topic_item);
                    model_topic->setItem(topic_count, 0, item_1);
                    model_topic->setItem(topic_count, 1, item_2);
                    model_topic->setItem(topic_count, 2, item_3);
                    topic_count++;
                }
            }
            table_topic->setModel(model_topic);
            table_topic->horizontalHeader()->setStretchLastSection(true);
            table_topic->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
            // pos
            table_pos = new QTableView(win);
            table_pos->setStyleSheet("background-color: rgb(239,255,254)");
            model_pos = new QStandardItemModel(data.size(), table_headers_pos.size(), win);
            QStringList list_pos;
            for (auto item = table_headers_pos.begin(); item != table_headers_pos.end(); item++)
            {
                list_pos.append(&(*item->c_str()));
            }
            model_pos->setHorizontalHeaderLabels(list_pos);
            table_pos->setModel(model_pos);
            table_pos->horizontalHeader()->setStretchLastSection(true);
            table_pos->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

            // add plot
            QPen pen;
            int count = data.size();
            plot = new QCustomPlot(win);
            plot->setBackground(QColor(255, 250, 250)); 
            plot->axisRect()->setBackground(QColor(255, 250, 250));
            plot->legend->setBrush(QColor(100, 100, 100, 0));
            plot->legend->setBorderPen(Qt::NoPen);
            plot->xAxis2->setVisible(true);
            plot->xAxis2->setTickLabels(false);
            plot->yAxis2->setVisible(true);
            plot->yAxis2->setTickLabels(false);
            for (int i = 0; i < count; i++)
            {
                pen.setColor(QColor::fromRgb(gen_rand(255), gen_rand(255), gen_rand(255)));
                plot->addGraph()->setName("");
                plot->legend->removeItem(0);
                plot->graph()->setPen(pen);
                plot->graph()->setAntialiasedFill(true);
            }
            for (int i = 0; i < count; i++)
            {
                pen.setColor(QColor::fromRgb(gen_rand(255), gen_rand(255), gen_rand(255)));
                plot->addGraph()->setName(nodes[i]);
                plot->graph()->setPen(pen);
                plot->graph()->setLineStyle(QCPGraph::LineStyle::lsNone);
                plot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 8));
                plot->graph()->setAntialiasedFill(true);
            }
            for (int i = 1; i < count; i++)
            {
                plot->legend->addElement(0, i, plot->legend->item(i));
            }
            plot->xAxis->setLabel("x (meter)");
            plot->yAxis->setLabel("y (meter)");
            plot->legend->setVisible(true);
            plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
            int plot_count = plot->plotLayout()->rowCount();
            int indent = win->width() / 2 - 50 * nodes.size();
            if (indent <= 0)
            {
                indent = 1;
            }
            plot->legend->setMargins(QMargins(indent, 1, indent, 1));
            plot->plotLayout()->addElement(plot_count, 0, plot->legend);
            plot->plotLayout()->setRowStretchFactor(plot_count, 0.0001);
            plot->xAxis->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom | Qt::AlignRight);
            // set layout
            QVBoxLayout *vbox_1 = new QVBoxLayout();
            QVBoxLayout *vbox_2 = new QVBoxLayout();
            QHBoxLayout *hbox_1 = new QHBoxLayout();
            QHBoxLayout *hbox_2 = new QHBoxLayout();
            QHBoxLayout *hbox_3 = new QHBoxLayout();
            hbox_1->addWidget(topic_label);
            hbox_2->addWidget(pos_label);
            hbox_3->addWidget(plot_label);
            hbox_1->setAlignment(Qt::AlignCenter);
            hbox_2->setAlignment(Qt::AlignCenter);
            hbox_3->setAlignment(Qt::AlignCenter);
            vbox_2->addWidget(info_label, 2);
            vbox_2->addStretch(1);
            vbox_2->addWidget(info_button, 1);
            vbox_2->addStretch(1);
            vbox_2->addWidget(exit_button, 1);
            hbox->addLayout(vbox_2, 1);
            vbox_1->addLayout(hbox_1, 1);
            vbox_1->addWidget(table_topic, 6);
            hbox->addLayout(vbox_1, 4);
            vbox->addLayout(hbox, 8);
            vbox->addLayout(hbox_2, 1);
            vbox->addWidget(table_pos, 8);
            vbox->addLayout(hbox_3, 1);
            vbox->addWidget(plot, 12);
            win->setLayout(vbox);
            for (int i = 0; i < data.size(); i++)
            {
                std::thread thread(&MonitorMainWindow::update_table, this, i);
                thread.detach();
                threads.push_back(&thread);
            }
            std::thread thread(&MonitorMainWindow::update_info, this);
            thread.detach();
            threads.push_back(&thread);

            // connect signal and slot
            QObject::connect(signal_button, &QPushButton::clicked, this, &MonitorMainWindow::update_plot_slot);
            QObject::connect(info_button, &QPushButton::clicked, this, &MonitorMainWindow::info_window_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &MonitorMainWindow::exit_slot);
        }

        void update_info()
        {
            while (!thread_stop)
            {
                if (ros::master::check())
                {
                    info_label->setText(("[ROS State]  Running\n[Vehicle Count]  " + to_string(nodes.size())).c_str());
                    info_label->setStyleSheet("color: green; font-size: 14pt;");
                }
                else
                {
                    info_label->setText("[ROS State]  Not Running\n[Vehicle Count]  0");
                    info_label->setStyleSheet("color: red; font-size: 14pt;");
                }
                ros::Duration(0.2).sleep();
            }
        }

        void update_table(int thread_id)
        {
            vehicle_state *vec = data[thread_id];
            string mode;
            QStandardItem *item_1 = new QStandardItem();
            QStandardItem *item_2 = new QStandardItem();
            QStandardItem *item_3 = new QStandardItem();
            QStandardItem *item_4 = new QStandardItem();
            QStandardItem *item_5 = new QStandardItem();
            QStandardItem *item_6 = new QStandardItem();
            QStandardItem *item_7 = new QStandardItem();
            QStandardItem *item_8 = new QStandardItem();
            QStandardItem *item_9 = new QStandardItem();
            QStandardItem *item_10 = new QStandardItem();
            QStandardItem *item_11 = new QStandardItem();
            QStandardItem *item_12 = new QStandardItem();
            item_1->setEditable(false);
            item_2->setEditable(false);
            item_3->setEditable(false);
            item_4->setEditable(false);
            item_5->setEditable(false);
            item_6->setEditable(false);
            item_7->setEditable(false);
            item_8->setEditable(false);
            item_9->setEditable(false);
            item_10->setEditable(false);
            item_11->setEditable(false);
            item_12->setEditable(false);
            item_1->setTextAlignment(Qt::AlignCenter);
            item_2->setTextAlignment(Qt::AlignCenter);
            item_3->setTextAlignment(Qt::AlignCenter);
            item_4->setTextAlignment(Qt::AlignCenter);
            item_5->setTextAlignment(Qt::AlignCenter);
            item_6->setTextAlignment(Qt::AlignCenter);
            item_7->setTextAlignment(Qt::AlignCenter);
            item_8->setTextAlignment(Qt::AlignCenter);
            item_9->setTextAlignment(Qt::AlignCenter);
            item_10->setTextAlignment(Qt::AlignCenter);
            item_11->setTextAlignment(Qt::AlignCenter);
            item_12->setTextAlignment(Qt::AlignCenter);
            item_1->setText(vec->vehicle_name.c_str());
            item_2->setText(vec->sensor_name.c_str());
            mode = vec->state_mode;
            // Remove "AUTO."
            if (mode.find_first_of(".") != string::npos)
            {
                mode.erase(0, 5);
            }
            item_3->setText(mode.c_str());
            item_4->setText(to_string((*(vec->x.end() - 1))).c_str());
            item_5->setText(to_string((*(vec->y.end() - 1))).c_str());
            item_6->setText(to_string((*(vec->z.end() - 1))).c_str());
            item_7->setText(to_string((*(vec->vx.end() - 1))).c_str());
            item_8->setText(to_string((*(vec->vy.end() - 1))).c_str());
            item_9->setText(to_string((*(vec->vz.end() - 1))).c_str());
            item_10->setText(to_string((*(vec->roll.end() - 1) * 180 / PI)).c_str());
            item_11->setText(to_string((*(vec->pitch.end() - 1) * 180 / PI)).c_str());
            item_12->setText(to_string((*(vec->yaw.end() - 1) * 180 / PI)).c_str());
            model_pos->setItem(thread_id, 0, item_1);
            model_pos->setItem(thread_id, 1, item_2);
            model_pos->setItem(thread_id, 2, item_3);
            model_pos->setItem(thread_id, 3, item_4);
            model_pos->setItem(thread_id, 4, item_5);
            model_pos->setItem(thread_id, 5, item_6);
            model_pos->setItem(thread_id, 6, item_7);
            model_pos->setItem(thread_id, 7, item_8);
            model_pos->setItem(thread_id, 8, item_9);
            model_pos->setItem(thread_id, 9, item_10);
            model_pos->setItem(thread_id, 10, item_11);
            model_pos->setItem(thread_id, 11, item_12);
            while (ros::ok() && !thread_stop)
            {
                mode = vec->state_mode;
                // Remove "AUTO."
                if (mode.find_first_of(".") != string::npos)
                {
                    mode.erase(0, 5);
                }
                item_3->setText(mode.c_str());
                item_4->setText(to_string((*(vec->x.end() - 1))).c_str());
                item_5->setText(to_string((*(vec->y.end() - 1))).c_str());
                item_6->setText(to_string((*(vec->z.end() - 1))).c_str());
                item_7->setText(to_string((*(vec->vx.end() - 1))).c_str());
                item_8->setText(to_string((*(vec->vy.end() - 1))).c_str());
                item_9->setText(to_string((*(vec->vz.end() - 1))).c_str());
                item_10->setText(to_string((*(vec->roll.end() - 1) * 180 / PI)).c_str());
                item_11->setText(to_string((*(vec->pitch.end() - 1) * 180 / PI)).c_str());
                item_12->setText(to_string((*(vec->yaw.end() - 1) * 180 / PI)).c_str());
                if (thread_id == 0)
                {
                    signal_button->click();
                }
                ros::Duration(update_time).sleep();
            }
        }

        void update_plot_slot()
        {
            int i = 0;
            for (auto item = data.begin(); item != data.end(); item++)
            {
                plot->graph(i)->setData((*item)->x, (*item)->y);
                if (i == 0)
                {
                    plot->graph(i)->rescaleAxes();
                }
                else
                {
                    plot->graph(i)->rescaleAxes(true);
                }
                i++;
            }
            for (auto item = data.begin(); item != data.end(); item++)
            {
                x_end[0] = *((*item)->x.end() - 1);
                y_end[0] = *((*item)->y.end() - 1);
                plot->graph(i)->setData(x_end, y_end);
                i++;
            }
            plot->yAxis->scaleRange(1.5, plot->yAxis->range().center());
            plot->xAxis->scaleRange(1.5, plot->xAxis->range().center());
            plot->replot();
        }

        // slot functions
        void info_window_slot()
        {
            info_win->win->exec();
        }

        void exit_slot()
        {
            bool stop = false;
            int tmp = 0;
            thread_stop = true;
            for (auto item = data.begin(); item != data.end(); item++)
            {
                (*item)->thread_stop = true;
            }
            ros::Duration(0.2).sleep();
            win->close();
        }

        // utility functions
        // get rand number from 0 to rand_max
        int gen_rand(int rand_max)
        {
            return int(rand() / (double)RAND_MAX * rand_max);
        }
};
#endif