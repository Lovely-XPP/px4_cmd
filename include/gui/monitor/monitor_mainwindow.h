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

#include <gui/monitor/monitor_infowindow.h>
#include <gui/monitor/monitor_imagewindow.h>
#include <qcustomplot.h>
#include <vehicle_state.h>

#define PI 3.14159265358979323846

struct cell_info
{
    int node_id;
    int cell_id;
    QString str;
};
Q_DECLARE_METATYPE(cell_info);

using namespace std;

class MonitorMainWindow : public QDialog
{
    Q_OBJECT
    public:
        MonitorInfoWindow *info_win = new MonitorInfoWindow(this);
        QWidget *parent;
        MonitorMainWindow(QWidget *parent_widget, QStringList nodes_input)
        {
            this->setAttribute(Qt::WA_DeleteOnClose);
            qRegisterMetaType<QList<QPersistentModelIndex>>("QList<QPersistentModelIndex>");
            qRegisterMetaType<QList<QAbstractItemModel::LayoutChangeHint>>("QList<QAbstractItemModel::LayoutChangeHint>");
            qRegisterMetaType<cell_info>("cell_info");
            qRegisterMetaType<cell_info>("cell_info&");
            nodes = nodes_input;
            setup();
        }

        ~MonitorMainWindow()
        {
            exit_slot();
        }
    
    signals:
        void update_info_signal();
        void update_cell_info_signal(QVariant info);

    private:
        // settings
        string version = "V1.0.3";
        double update_time = 0.2;
        vector<string> table_headers_pos = {"Vehicle", "Sensor", "Mode", "x", "y", "z", "vx", "vy", "vz", "roll (deg)", "pitch (deg)", "yaw (deg)"};
        vector<string> table_headers_topic = {"Node", "Sensor", "Senor Topic"};

        // init vector
        QStringList nodes;
        QVector<double> x_end = {0};
        QVector<double> y_end = {0};
        QStringList topics;
        vector<vector<QStandardItem *>> table_items;
        QVector<vehicle_state *> data;
        QVector<QCPCurve *> curves;
        QVector<string> img_topic_showing;
        bool thread_stop = false;

        // selected topic to show
        string topic_name_show = "";
        cv::Mat img;

        // output file string
        string output_file = "";
        bool update_signal = false;

        //Widgets
        QMessageBox *msg_box;
        QCustomPlot *plot;
        QTableView *table_info;
        QTableView *table_topic;
        QLabel *info_label;
        QStandardItemModel *model_info;
        QStandardItemModel *model_topic;
        QPushButton *signal_button;
        QPushButton *info_button;
        QPushButton *exit_button;

        void setup()
        {
            int argc = 0;
            char **argv;
            ros::init(argc, argv, "px4_cmd/monitor");

            // layout
            QHBoxLayout *hbox = new QHBoxLayout();
            QVBoxLayout *vbox = new QVBoxLayout();
            QIcon *icon = new QIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
            this->setWindowIcon(*icon);
            this->setFixedSize(1280, 750);
            this->setWindowTitle(("PX4 Cmd Simulation Monitor [Version: " + version + "]").c_str());
            this->setStyleSheet("background-color: rgb(255,250,250)");
            // get data
            for (auto item = nodes.begin(); item != nodes.end(); item++)
            {
                vehicle_state *vec = new vehicle_state();
                vec->get_state((*item).toStdString());
                data.push_back(vec);
            }

            // add label
            QLabel *topic_label = new QLabel("[ Vehicle Sensor Topics Information ]", this);
            QLabel *pos_label = new QLabel("[ Vehicle Position & Pose Information ]", this);
            QLabel *plot_label = new QLabel("[ Vehicle Position 2D Plane Plot (x-y Plane) ]", this);
            info_label = new QLabel(("[ROS State]  Running\n[Vehicle Count]  " + to_string(nodes.size())).c_str(), this);
            info_label->setStyleSheet("color: green; font-size: 14pt");
            topic_label->setStyleSheet("color: rgb(255,137,46); font-size: 14pt; font-weight: bold");
            pos_label->setStyleSheet("color: rgb(120,200,200); font-size: 14pt; font-weight: bold");
            plot_label->setStyleSheet("color: black; font-size: 14pt; font-weight: bold");

            // add button
            signal_button = new QPushButton(this);
            signal_button->setFixedSize(1, 1);
            //info
            info_button = new QPushButton("About", this);
            info_button->setMinimumHeight(50);
            info_button->setStyleSheet("background-color: rgb(255,227,132); color: black; font-size: 16pt; font-weight: bold");
            exit_button = new QPushButton("Exit", this);
            exit_button->setMinimumHeight(50);
            exit_button->setStyleSheet("background-color: rgb(255,106,106); color: black; font-size: 16pt; font-weight: bold");

            // add table
            // topic
            table_topic = new QTableView(this);
            table_topic->setStyleSheet("background-color: rgb(255,245,235)");
            table_topic->setContextMenuPolicy(Qt::CustomContextMenu);
            model_topic = new QStandardItemModel(data.size(), table_headers_topic.size(), this);
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
            // table of infomation
            table_info = new QTableView(this);
            table_info->setStyleSheet("background-color: rgb(239,255,254)");
            model_info = new QStandardItemModel(data.size(), table_headers_pos.size(), this);
            QStringList list_pos;
            for (auto item = table_headers_pos.begin(); item != table_headers_pos.end(); item++)
            {
                list_pos.append(&(*item->c_str()));
            }
            model_info->setHorizontalHeaderLabels(list_pos);
            vector<QStandardItem *> table_item;
            for (int i = 0; i < nodes.size(); i++)
            {
                table_item.clear();
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
                item_1->setText(nodes[i]);
                item_2->setText(data[i]->sensor_name.c_str());
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
                model_info->setItem(i, 0, item_1);
                model_info->setItem(i, 1, item_2);
                model_info->setItem(i, 2, item_3);
                model_info->setItem(i, 3, item_4);
                model_info->setItem(i, 4, item_5);
                model_info->setItem(i, 5, item_6);
                model_info->setItem(i, 6, item_7);
                model_info->setItem(i, 7, item_8);
                model_info->setItem(i, 8, item_9);
                model_info->setItem(i, 9, item_10);
                model_info->setItem(i, 10, item_11);
                model_info->setItem(i, 11, item_12);
                table_item.push_back(item_1);
                table_item.push_back(item_2);
                table_item.push_back(item_3);
                table_item.push_back(item_4);
                table_item.push_back(item_5);
                table_item.push_back(item_6);
                table_item.push_back(item_7);
                table_item.push_back(item_8);
                table_item.push_back(item_9);
                table_item.push_back(item_10);
                table_item.push_back(item_11);
                table_item.push_back(item_12);
                table_items.push_back(table_item);
            }
            table_info->setModel(model_info);
            table_info->horizontalHeader()->setStretchLastSection(true);
            table_info->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

            // add plot
            QPen pen;
            int count = data.size();
            plot = new QCustomPlot(this);
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
                QCPCurve *curve = new QCPCurve(plot->xAxis, plot->yAxis);
                curve->setPen(pen);
                curve->removeFromLegend();
                curve->setAntialiasedFill(true);
                curves.push_back(curve);   
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
            int indent = this->width() / 2 - 50 * nodes.size();
            if (indent <= 0)
            {
                indent = 1;
            }
            plot->legend->setMargins(QMargins(indent, 1, indent, 1));
            plot->plotLayout()->addElement(plot_count, 0, plot->legend);
            plot->plotLayout()->setRowStretchFactor(plot_count, 0.0001);
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
            vbox->addWidget(table_info, 8);
            vbox->addLayout(hbox_3, 1);
            vbox->addWidget(plot, 12);
            this->setLayout(vbox);
            for (int i = 0; i < data.size(); i++)
            {
                std::thread thread(&MonitorMainWindow::update_table_info, this, i);
                thread.detach();
            }
            std::thread thread(&MonitorMainWindow::update_info, this);
            thread.detach();

            // connect signal and slot
            QObject::connect(signal_button, &QPushButton::clicked, this, &MonitorMainWindow::update_plot_slot);
            QObject::connect(info_button, &QPushButton::clicked, this, &MonitorMainWindow::info_window_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &MonitorMainWindow::exit_slot);
            QObject::connect(table_topic, &QTableView::customContextMenuRequested, this, &MonitorMainWindow::table_click_slot);
            QObject::connect(this, &MonitorMainWindow::update_info_signal, this, &MonitorMainWindow::update_info_slot);
            QObject::connect(this, &MonitorMainWindow::update_cell_info_signal, this, &MonitorMainWindow::update_cell_info_slot);
        }

        void update_info()
        {
            while (!thread_stop)
            {
                emit update_info_signal();
                usleep(200000);
            }
        }

        void update_info_slot()
        {
            if (ros::ok() && !data[0]->ros_stop)
            {
                info_label->setText(("[ROS State]  Running\n[Vehicle Count]  " + to_string(nodes.size())).c_str());
                info_label->setStyleSheet("color: green; font-size: 14pt;");
            }
            else
            {
                info_label->setText("[ROS State]  Not Running\n[Vehicle Count]  0");
                info_label->setStyleSheet("color: red; font-size: 14pt;");
            }
        }

        void update_table_info(int thread_id)
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
            QVariant info;
            cell_info info3, info4, info5, info6, info7, info8, info9, info10, info11, info12;
            info3.node_id = thread_id;
            info4.node_id = thread_id;
            info5.node_id = thread_id;
            info6.node_id = thread_id;
            info7.node_id = thread_id;
            info8.node_id = thread_id;
            info9.node_id = thread_id;
            info10.node_id = thread_id;
            info11.node_id = thread_id;
            info12.node_id = thread_id;
            info3.cell_id = 2;
            info4.cell_id = 3;
            info5.cell_id = 4;
            info6.cell_id = 5;
            info7.cell_id = 6;
            info8.cell_id = 7;
            info9.cell_id = 8;
            info10.cell_id = 9;
            info11.cell_id = 10;
            info12.cell_id = 11;
            while (ros::ok() && !thread_stop)
            {
                mode = vec->state_mode;
                // Remove "AUTO."
                if (mode.find_first_of(".") != string::npos)
                {
                    mode.erase(0, 5);
                }
                info3.str = mode.c_str();
                info4.str = to_string((*(vec->x.end() - 1))).c_str();
                info5.str = to_string((*(vec->y.end() - 1))).c_str();
                info6.str = to_string((*(vec->z.end() - 1))).c_str();
                info7.str = to_string((*(vec->vx.end() - 1))).c_str();
                info8.str = to_string((*(vec->vy.end() - 1))).c_str();
                info9.str = to_string((*(vec->vz.end() - 1))).c_str();
                info10.str = to_string((*(vec->roll.end() - 1) * 180 / PI)).c_str();
                info11.str = to_string((*(vec->pitch.end() - 1) * 180 / PI)).c_str();
                info12.str = to_string((*(vec->yaw.end() - 1) * 180 / PI)).c_str();
                info.setValue(info3);
                emit update_cell_info_signal(info);
                info.setValue(info4);
                emit update_cell_info_signal(info);
                info.setValue(info5);
                emit update_cell_info_signal(info);
                info.setValue(info6);
                emit update_cell_info_signal(info);
                info.setValue(info7);
                emit update_cell_info_signal(info);
                info.setValue(info8);
                emit update_cell_info_signal(info);
                info.setValue(info9);
                emit update_cell_info_signal(info);
                info.setValue(info10);
                emit update_cell_info_signal(info);
                info.setValue(info11);
                emit update_cell_info_signal(info);
                info.setValue(info12);
                emit update_cell_info_signal(info);
                usleep(100000);
            }
        }

        void update_cell_info_slot(QVariant info)
        {
            cell_info data = info.value<cell_info>();
            table_items[data.node_id][data.cell_id]->setText(data.str);
            if (data.node_id == 0 && data.cell_id == 3)
            {
                signal_button->click();
            }
        };

        void update_plot_slot()
        {
            int i = 0;
            for (auto item = data.begin(); item != data.end(); item++)
            {
                curves[i]->setData((*item)->t, (*item)->x, (*item)->y);
                if (i == 0)
                {
                    curves[i]->rescaleAxes();
                }
                else
                {
                    curves[i]->rescaleAxes(true);
                }
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
            info_win->exec();
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
            usleep(200000);
            this->close();
        }

        void table_click_slot(QPoint pos)
        {
            QModelIndex index = table_topic->indexAt(pos);
            int column = index.column();
            QVariant data = index.data();
            topic_name_show = data.toString().toStdString();
            // new一个菜单
            QMenu *popMenu = new QMenu(this);
            popMenu->setStyleSheet("QMenu::item {background-color: #FFFFFF; color: #000000;} QMenu::item::selected{background-color: #0078D7; color: #FFFFFF;}");

            if (index.isValid() && column == 2)
            {
                // 新建一个 QAction（可建多个），并设置显示的文本
                QAction *actionUpdateInfo = new QAction();
                actionUpdateInfo->setText(QString(QStringLiteral("Show Topic Data")));

                // 把获取到的行数存储到 QAction中
                popMenu->addAction(actionUpdateInfo);

                // QAction 绑定槽函数，当点击QAction时触发
                QObject::connect(actionUpdateInfo, &QAction::triggered, this, &MonitorMainWindow::show_topic_data_slot);

                // 菜单显示到鼠标的位置
                popMenu->exec(QCursor::pos());
            }
            // 释放内存
            QList<QAction *> list = popMenu->actions();
            foreach (QAction *pAction, list) delete pAction;
            delete popMenu;
        }

        void show_topic_data_slot()
        {
            const string topic_name = topic_name_show;
            for (auto item = img_topic_showing.begin(); item != img_topic_showing.end(); item++)
            {
                if (*item == topic_name)
                {
                    return;
                }
            }
            img_topic_showing.push_back(topic_name_show);
            MonitorImageWindow *img_win = new MonitorImageWindow(this, topic_name_show, img_topic_showing.size());
            std::thread show_topic_data_thread(&MonitorMainWindow::show_topic_data_thread_func, this, topic_name, img_win);
            show_topic_data_thread.detach();
        }

        void show_topic_data_thread_func(string topic_name, MonitorImageWindow *img_win)
        {
            img_win->start();
            img_win->exec();
            img_topic_showing.erase(std::find(img_topic_showing.begin(), img_topic_showing.end(), topic_name));
        }

        // utility functions
        // get rand number from 0 to rand_max
        int gen_rand(int rand_max)
        {
            return int(rand() / (double)RAND_MAX * rand_max);
        }
};
#endif