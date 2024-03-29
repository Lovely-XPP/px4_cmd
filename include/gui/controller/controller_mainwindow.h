// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef CONTROLLER_MAINWINDOW_H
#define CONTROLLER_MAINWINDOW_H
#include <QMainWindow>
#include <QApplication>
#include <QDialog>
#include <QMessageBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QVector>
#include <QBrush>
#include <QColor>
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
#include <sys/stat.h>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <vehicle_state.h>

#include <gui/controller/controller_infowindow.h>
#include <gui/controller/controller_modewindow.h>
#include <gui/controller/controller_takeoffwindow.h>
#include <gui/controller/controller_manualwindow.h>
#include <gui/controller/controller_trajectorywindow.h>
#include <gui/controller/controller_generatewindow.h>
#include <gui/controller/controller_imagewindow.h>
#include <gui/controller/controller_runnodewindow.h>
#include <gui/widgets/MultiComboBox.h>
#include <qcustomplot.h>

#define PI 3.14159265358979323846

struct cell_info
{
    int node_id;
    int cell_id;
    QString str;
};
struct button_info
{
    QPushButton * button_name;
    bool state;
};
Q_DECLARE_METATYPE(cell_info);
Q_DECLARE_METATYPE(button_info);

using namespace std;

class ControllerMainWindow : public QDialog
{
    Q_OBJECT
    public:
        ControllerInfoWindow *info_win = new ControllerInfoWindow(this);
        ControllerModeWindow *mode_win = new ControllerModeWindow(this);
        ControllerTakeoffWindow *takeoff_win = new ControllerTakeoffWindow(this);
        ControllerManualWindow *manual_win = new ControllerManualWindow(this);
        ControllerTrajectoryWindow *trajectory_win = new ControllerTrajectoryWindow(this);
        ControllerGenerateWindow *generate_win = new ControllerGenerateWindow(this);
        ControllerRunNodeWindow *node_win = nullptr;
        ControllerMainWindow(QStringList nodes_input, QWidget *parent_widget = 0)
        {
            this->setAttribute(Qt::WA_DeleteOnClose);
            qRegisterMetaType<QList<QPersistentModelIndex>>("QList<QPersistentModelIndex>");
            qRegisterMetaType<QList<QAbstractItemModel::LayoutChangeHint>>("QList<QAbstractItemModel::LayoutChangeHint>");
            qRegisterMetaType<QList<Qt::Orientation>>("QList<Qt::Orientation>");
            qRegisterMetaType<cell_info>("cell_info");
            qRegisterMetaType<cell_info>("cell_info&");
            qRegisterMetaType<button_info>("button_info");
            qRegisterMetaType<button_info>("button_info&");
            nodes = nodes_input;
            setup();
        }
        ~ControllerMainWindow()
        {
            thread_stop = true;
            usleep(200000);
        }

    signals:
        void update_controller_cell_info_signal(QVariant info);
        void update_monitor_cell_info_signal(QVariant info);
        void update_controller_info_signal();
        void update_button_state_signal(QVariant state);

    private:
        // settings
        string version = "V2.0.0 alpha";
        QString current_cmd = "None";
        double update_time = 0.3;

        // ros node handle
        ros::NodeHandle nh;

        // init vectors
        QVector<vehicle_command *> data;
        QVector<px4_cmd::Command> cmds;
        QVector<ros::Publisher> pubs;
        vector<vector<double>> cmd_values;
        vector<vector<QStandardItem *>> controller_table_items;
        vector<vector<QStandardItem *>> monitor_table_items;
        vector<string> table_headers_pos = {"Vehicle", "Sensor", "Mode", "x", "y", "z", "vx", "vy", "vz", "roll (deg)", "pitch (deg)", "yaw (deg)"};
        QVector<vehicle_state *> uav_state_data;
        QVector<vehicle_state *> plot_uav_state_data;
        QVector<QCPCurve *> plot_curves;
        QVector<double> x_end = {0};
        QVector<double> y_end = {0};
        bool thread_stop = false;
        bool land_return_operate = false;
        bool ext_cmd_state = false;
        QStringList nodes;
        QString operating_info;
        bool fix_wing_include = false;
        mutex cmd_mutex;

        // Widgets
        QMessageBox *msg_box;
        QCustomPlot *plot;
        QPushButton *mode_button;
        QPushButton *signal_button_1;
        QPushButton *signal_button_2;
        QPushButton *signal_button_3;
        QPushButton *arm_button;
        QPushButton *takeoff_button;
        QPushButton *disarm_button;
        QPushButton *about_button;
        QPushButton *exit_button;
        QPushButton *manual_button;
        QPushButton *trajectory_button;
        QPushButton *external_button;
        QPushButton *hover_button;
        QPushButton *land_button;
        QPushButton *return_button;
        QPushButton *generate_button;
        QPushButton *topic_view_button;
        QPushButton *run_ros_node_button;
        QComboBox *plot_select;
        QMultiComboBox *plot_uav_select;
        QTableView *controller_info_table = new QTableView(this);
        QTableView *monitor_info_table = new QTableView(this);
        QStandardItemModel *controller_info_model;
        QStandardItemModel *monitor_info_model;
        QLabel *info_label_1;
        QLabel *info_label_2;
        QLabel *info_label_3;
        QLabel *state_label_1;
        QLabel *state_label_2;
        QLabel *state_label_3;
        QLabel *plot_label;
        QLabel *plot_uav_label;
        QStringList plot_modes = {
            "[Postion] x - y",
            "[Height] t - z",
            "[Roll] t - roll",
            "[Pitch] t - pitch",
            "[Yaw] t - yaw"};
        QStringList plot_uavs = {};

        void setup()
        {
            // set window
            QIcon *icon = new QIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
            this->setWindowIcon(*icon);
            this->setFixedSize(1300, 900);
            this->setWindowTitle(("PX4 Cmd Simulation Controller [Version: " + version + "]").c_str());
            this->setStyleSheet("background-color: rgb(255,250,250)");

            // get data
            for (auto item = nodes.begin(); item != nodes.end(); item++)
            {
                vehicle_command *vec_cmd = new vehicle_command();
                vec_cmd->start((*item).toStdString());
                data.push_back(vec_cmd);
                vehicle_state *vec_state = new vehicle_state();
                vec_state->get_state((*item).toStdString());
                uav_state_data.push_back(vec_state);
            }

            // ros setting
            int argc = 0;
            char **argv;
            ros::init(argc, argv, "px4_cmd/px4_controller");
            for (size_t i = 0; i < nodes.size(); i++)
            {
                px4_cmd::Command cmd;
                if (data[i]->vehicle_type == px4_cmd::Command::FixWing)
                {
                    cmd.Vehicle = px4_cmd::Command::FixWing;
                    fix_wing_include = true;
                }
                else
                {
                    cmd.Vehicle = px4_cmd::Command::Multicopter;
                }
                cmd.desire_cmd[0] = data[i]->init_x;
                cmd.desire_cmd[1] = data[i]->init_y;
                cmd.desire_cmd[2] = 0;
                ros::Publisher pub = nh.advertise<px4_cmd::Command>((nodes[i] + "/px4_cmd/control_command").toStdString().c_str(), 50);
                pubs.push_back(pub);
                cmds.push_back(cmd);
                cmd_values.push_back({0, 0, 0, 0});
                plot_uavs.push_back(nodes[i]);
            }

            // start ros thread
            for (size_t i = 0; i < nodes.size(); i++)
            {
                std::thread ros_thread(&ControllerMainWindow::ros_thread_func, this, i);
                ros_thread.detach();
            }

            // add labels
            info_label_1 = new QLabel("[ROS State]  Running", this);
            info_label_1->setStyleSheet("color: green; font-size: 14pt");
            info_label_2 = new QLabel(("[Vehicle Count]  " + to_string(nodes.size())).c_str(), this);
            info_label_2->setStyleSheet("color: green; font-size: 14pt");
            info_label_3 = new QLabel(("[Current Mode]  " + data[0]->state_mode).c_str(), this);
            info_label_3->setStyleSheet("color: green; font-size: 14pt");
            state_label_1 = new QLabel("[Arm State]  DisArm", this);
            state_label_1->setStyleSheet("color: orange; font-size: 14pt");
            state_label_2 = new QLabel("[Current CMD]  None", this);
            state_label_2->setStyleSheet("color: green; font-size: 14pt");
            state_label_3 = new QLabel("", this);
            state_label_3->setStyleSheet("color: orange; font-size: 14pt");
            info_label_3->setMinimumWidth(300);
            state_label_2->setMinimumWidth(300);

            // add top buttons
            QPushButton *split_line = new QPushButton("", this);
            split_line->setMaximumHeight(1);
            split_line->setFocusPolicy(Qt::NoFocus);
            split_line->setEnabled(false);
            mode_button = new QPushButton("Mode", this);
            arm_button = new QPushButton("Arm", this);
            takeoff_button = new QPushButton("Take Off", this);
            disarm_button = new QPushButton("DisArm", this);
            about_button = new QPushButton("About", this);
            exit_button = new QPushButton("Exit", this);
            disarm_button->setEnabled(false);
            mode_button->setMinimumHeight(45);
            arm_button->setMinimumHeight(45);
            takeoff_button->setMinimumHeight(45);
            disarm_button->setMinimumHeight(45);
            about_button->setMinimumHeight(45);
            exit_button->setMinimumHeight(45);
            mode_button->setStyleSheet("background-color: rgb(50,191,255); font-size: 16pt");
            arm_button->setStyleSheet("background-color: rgb(84,255,159); font-size: 16pt");
            takeoff_button->setStyleSheet("background-color: rgb(170,123,255); font-size: 16pt");
            disarm_button->setStyleSheet("background-color: rgb(255,190,155); font-size: 16pt");
            about_button->setStyleSheet("background-color: rgb(255,227,132); font-size: 16pt");
            exit_button->setStyleSheet("background-color: rgb(255,106,106); font-size: 16pt");

            // add buttom button
            manual_button = new QPushButton("Manual CMD", this);
            trajectory_button = new QPushButton("Trajectory CMD", this);
            external_button = new QPushButton("External CMD", this);
            hover_button = new QPushButton("Hover / Loiter", this);
            land_button = new QPushButton("Land", this);
            return_button = new QPushButton("Return", this);
            topic_view_button = new QPushButton("Image Topic Viewer", this);
            run_ros_node_button = new QPushButton("Run Ros Node", this);
            generate_button = new QPushButton("Generate Template External CMD Code");
            generate_button->setVisible(false);
            signal_button_1 = new QPushButton("", this);
            signal_button_2 = new QPushButton("", this);
            signal_button_3 = new QPushButton("", this);
            signal_button_1->setVisible(false);
            signal_button_2->setVisible(false);
            signal_button_3->setVisible(false);
            generate_button->setVisible(false);
            manual_button->setMinimumHeight(40);
            trajectory_button->setMinimumHeight(40);
            external_button->setMinimumHeight(40);
            hover_button->setMinimumHeight(40);
            land_button->setMinimumHeight(40);
            return_button->setMinimumHeight(40);
            topic_view_button->setMinimumHeight(40);
            topic_view_button->setMaximumWidth(300);
            run_ros_node_button->setMinimumHeight(40);
            run_ros_node_button->setMaximumWidth(300);
            generate_button->setMinimumHeight(25);
            manual_button->setEnabled(false);
            trajectory_button->setEnabled(false);
            external_button->setEnabled(false);
            hover_button->setEnabled(false);
            land_button->setEnabled(false);
            return_button->setEnabled(false);
            manual_button->setStyleSheet("background-color: rgb(182,228,222); font-size: 16pt");
            trajectory_button->setStyleSheet("background-color: rgb(182,228,222); font-size: 16pt");
            external_button->setStyleSheet("background-color: rgb(182,228,222); font-size: 16pt");
            hover_button->setStyleSheet("background-color: rgb(255,245,129); font-size: 16pt");
            land_button->setStyleSheet("background-color: rgb(255,245,129); font-size: 16pt");
            return_button->setStyleSheet("background-color: rgb(255,245,129); font-size: 16pt");
            topic_view_button->setStyleSheet("background-color: rgb(255,198,17); font-size: 16pt");
            run_ros_node_button->setStyleSheet("background-color: rgb(129,79,255); font-size: 16pt");
            generate_button->setStyleSheet("background-color: rgb(233,181,177); font-size: 16pt");

            // add table - controller
            QStringList table_headers_ext_cmd = {
                "Node",
                "Vehicle Type",
                "CMD Mode",
                "Frame",
                "CMD 1",
                "CMD 2",
                "CMD 3",
                "CMD 4  [Yaw]",
                "External State",
                "External CMD Topic"
            };
            controller_info_model = new QStandardItemModel(nodes.size(), table_headers_ext_cmd.size(), this);
            controller_info_model->setHorizontalHeaderLabels(table_headers_ext_cmd);
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
                item_1->setText("  " + nodes[i] + "  ");
                if (data[i]->vehicle_type == px4_cmd::Command::FixWing)
                {
                    item_2->setText("FixWing");
                }
                else
                {
                    item_2->setText("Multicopter");
                }
                item_10->setText((nodes[i] + "/px4_cmd/external_command").toStdString().c_str());
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
                controller_info_model->setItem(i, 0, item_1);
                controller_info_model->setItem(i, 1, item_2);
                controller_info_model->setItem(i, 2, item_3);
                controller_info_model->setItem(i, 3, item_4);
                controller_info_model->setItem(i, 4, item_5);
                controller_info_model->setItem(i, 5, item_6);
                controller_info_model->setItem(i, 6, item_7);
                controller_info_model->setItem(i, 7, item_8);
                controller_info_model->setItem(i, 8, item_9);
                controller_info_model->setItem(i, 9, item_10);
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
                controller_table_items.push_back(table_item);
            }
            controller_info_table->setModel(controller_info_model);
            controller_info_table->horizontalHeader()->setStretchLastSection(true);
            controller_info_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
            controller_info_table->setStyleSheet("background-color: rgb(236,245,225)");

            // add table - monitor
            monitor_info_table->setStyleSheet("background-color: rgb(239,255,254)");
            monitor_info_model = new QStandardItemModel(data.size(), table_headers_pos.size(), this);
            QStringList list_pos;
            for (auto item = table_headers_pos.begin(); item != table_headers_pos.end(); item++)
            {
                list_pos.append(&(*item->c_str()));
            }
            monitor_info_model->setHorizontalHeaderLabels(list_pos);
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
                item_2->setText(uav_state_data[i]->sensor_name.c_str());
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
                monitor_info_model->setItem(i, 0, item_1);
                monitor_info_model->setItem(i, 1, item_2);
                monitor_info_model->setItem(i, 2, item_3);
                monitor_info_model->setItem(i, 3, item_4);
                monitor_info_model->setItem(i, 4, item_5);
                monitor_info_model->setItem(i, 5, item_6);
                monitor_info_model->setItem(i, 6, item_7);
                monitor_info_model->setItem(i, 7, item_8);
                monitor_info_model->setItem(i, 8, item_9);
                monitor_info_model->setItem(i, 9, item_10);
                monitor_info_model->setItem(i, 10, item_11);
                monitor_info_model->setItem(i, 11, item_12);
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
                monitor_table_items.push_back(table_item);
            }
            monitor_info_table->setModel(monitor_info_model);
            monitor_info_table->horizontalHeader()->setStretchLastSection(true);
            monitor_info_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

            // add plot
            QPen pen;
            plot = new QCustomPlot(this);
            plot->setBackground(QColor(255, 250, 250));
            plot->axisRect()->setBackground(QColor(255, 250, 250));
            plot->legend->setBrush(QColor(100, 100, 100, 0));
            plot->legend->setBorderPen(Qt::NoPen);
            plot->xAxis2->setVisible(true);
            plot->xAxis2->setTickLabels(false);
            plot->yAxis2->setVisible(true);
            plot->yAxis2->setTickLabels(false);
            plot->xAxis->setLabel("x (meter)");
            plot->yAxis->setLabel("y (meter)");
            plot->legend->setVisible(true);
            // select plot mode and uav
            plot_select = new QComboBox(this);
            plot_select->addItems(plot_modes);
            plot_select->setCurrentIndex(0);
            plot_select->setStyleSheet("background-color: rgb(134,198,237); font-size: 14pt");
            plot_label = new QLabel("plot mode:", this);
            plot_label->setStyleSheet("font-size: 14pt");
            plot_label->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
            plot_label->setMaximumWidth(100);
            plot_uav_select = new QMultiComboBox(this);
            for (size_t i = 0; i < plot_uavs.size(); i++)
            {
                plot_uav_select->addDataItem(plot_uavs[i]);
            }
            plot_uav_select->setStyleSheet("background-color: rgb(134,198,237); font-size: 14pt");
            plot_uav_label = new QLabel("plot uav:", this);
            plot_uav_label->setStyleSheet("font-size: 14pt");
            plot_uav_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
            plot_uav_label->setMaximumWidth(100);
            signal_button_3->blockSignals(true);

            // set layout
            // layout
            QVBoxLayout *vbox = new QVBoxLayout();
            QHBoxLayout *hbox_top = new QHBoxLayout();
            QVBoxLayout *vbox_top_1 = new QVBoxLayout();
            QVBoxLayout *vbox_top_2 = new QVBoxLayout();
            QVBoxLayout *vbox_top_3 = new QVBoxLayout();
            QVBoxLayout *vbox_top_4 = new QVBoxLayout();
            QVBoxLayout *vbox_top_5 = new QVBoxLayout();
            QVBoxLayout *vbox_buttom = new QVBoxLayout();
            QHBoxLayout *hbox_buttom_contoller_buttons = new QHBoxLayout();
            QVBoxLayout *vbox_buttom_monitor_buttons = new QVBoxLayout();
            QHBoxLayout *hbox_buttom_plot = new QHBoxLayout();
            QHBoxLayout *hbox_buttom_plot_1 = new QHBoxLayout();
            QHBoxLayout *hbox_buttom_plot_2 = new QHBoxLayout();

            vbox->setContentsMargins(30, 20, 30, 20);
            vbox_top_1->setSpacing(15);
            vbox_top_2->setSpacing(15);
            vbox_top_3->setSpacing(15);
            vbox_top_4->setSpacing(15);
            vbox_top_5->setSpacing(15);
            vbox_buttom_monitor_buttons->setAlignment(Qt::AlignHCenter);
            hbox_buttom_contoller_buttons->setAlignment(Qt::AlignTop);
            hbox_buttom_contoller_buttons->setSpacing(30);
            vbox_buttom->setSpacing(10);
            vbox_top_1->addWidget(info_label_1);
            vbox_top_1->addWidget(info_label_2);
            vbox_top_1->addWidget(info_label_3);
            hbox_top->addLayout(vbox_top_1);
            vbox_top_2->addWidget(state_label_1);
            vbox_top_2->addWidget(state_label_2);
            vbox_top_2->addWidget(state_label_3);
            hbox_top->addLayout(vbox_top_2);
            vbox_top_3->addWidget(mode_button);
            vbox_top_3->addWidget(arm_button);
            hbox_top->addLayout(vbox_top_3);
            vbox_top_4->addWidget(takeoff_button);
            vbox_top_4->addWidget(disarm_button);
            hbox_top->addLayout(vbox_top_4);
            vbox_top_5->addWidget(about_button);
            vbox_top_5->addWidget(exit_button);
            hbox_top->addLayout(vbox_top_5);
            hbox_top->setSpacing(30);
            vbox->addLayout(hbox_top, 10);
            vbox->addSpacing(15);
            vbox->addWidget(split_line, 1);
            vbox->addSpacing(15);
            hbox_buttom_contoller_buttons->addWidget(manual_button);
            hbox_buttom_contoller_buttons->addWidget(trajectory_button);
            hbox_buttom_contoller_buttons->addWidget(external_button);
            hbox_buttom_contoller_buttons->addWidget(hover_button);
            hbox_buttom_contoller_buttons->addWidget(land_button);
            hbox_buttom_contoller_buttons->addWidget(return_button);
            vbox_buttom->addLayout(hbox_buttom_contoller_buttons, 1);
            vbox_buttom->addWidget(controller_info_table, 3);
            vbox_buttom->addSpacing(15);
            hbox_buttom_plot_1->addWidget(plot_label, 1);
            hbox_buttom_plot_1->addWidget(plot_select, 1);
            vbox_buttom_monitor_buttons->addStretch(1);
            vbox_buttom_monitor_buttons->addLayout(hbox_buttom_plot_1);
            vbox_buttom_monitor_buttons->addSpacing(15);
            hbox_buttom_plot_2->addWidget(plot_uav_label);
            hbox_buttom_plot_2->addWidget(plot_uav_select);
            vbox_buttom_monitor_buttons->addLayout(hbox_buttom_plot_2);
            vbox_buttom_monitor_buttons->addStretch(1);
            vbox_buttom_monitor_buttons->addWidget(topic_view_button);
            vbox_buttom_monitor_buttons->addStretch(1);
            vbox_buttom_monitor_buttons->addWidget(run_ros_node_button);
            vbox_buttom_monitor_buttons->addStretch(2);
            hbox_buttom_plot->addLayout(vbox_buttom_monitor_buttons, 1);
            hbox_buttom_plot->addWidget(plot, 5);
            vbox_buttom->addLayout(hbox_buttom_plot, 5);
            vbox_buttom->addWidget(monitor_info_table, 3);
            vbox->addLayout(vbox_buttom);
            this->setLayout(vbox);

            // connect
            QObject::connect(about_button, &QPushButton::clicked, this, &ControllerMainWindow::info_window_slot);
            QObject::connect(mode_button, &QPushButton::clicked, this, &ControllerMainWindow::mode_window_slot);
            QObject::connect(takeoff_button, &QPushButton::clicked, this, &ControllerMainWindow::take_off_window_slot);
            QObject::connect(arm_button, &QPushButton::clicked, this, &ControllerMainWindow::arm_slot);
            QObject::connect(disarm_button, &QPushButton::clicked, this, &ControllerMainWindow::disarm_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &ControllerMainWindow::exit_slot);
            QObject::connect(hover_button, &QPushButton::clicked, this, &ControllerMainWindow::hover_slot);
            QObject::connect(land_button, &QPushButton::clicked, this, &ControllerMainWindow::land_slot);
            QObject::connect(return_button, &QPushButton::clicked, this, &ControllerMainWindow::return_slot);
            QObject::connect(manual_button, &QPushButton::clicked, this, &ControllerMainWindow::manual_cmd_slot);
            QObject::connect(trajectory_button, &QPushButton::clicked, this, &ControllerMainWindow::trajectory_cmd_slot);
            QObject::connect(external_button, &QPushButton::clicked, this, &ControllerMainWindow::ext_cmd_slot);
            QObject::connect(topic_view_button, &QPushButton::clicked, this, &ControllerMainWindow::image_topic_window_slot);
            QObject::connect(run_ros_node_button, &QPushButton::clicked, this, &ControllerMainWindow::run_ros_node_window_slot);
            QObject::connect(signal_button_1, &QPushButton::clicked, this, &ControllerMainWindow::ext_cmd_err_msg_slot);
            QObject::connect(signal_button_2, &QPushButton::clicked, this, &ControllerMainWindow::arm_fail_slot);
            QObject::connect(signal_button_3, &QPushButton::clicked, this, &ControllerMainWindow::update_plot_slot);
            QObject::connect(plot_uav_select, &QMultiComboBox::editTextChanged, this, &ControllerMainWindow::update_plot_uav_slot);
            QObject::connect(plot_select, &QComboBox::currentTextChanged, this, &ControllerMainWindow::update_plot_mode_slot);
            QObject::connect(mode_win, &ControllerModeWindow::change_mode_signal, this, &ControllerMainWindow::change_mode_slot);
            QObject::connect(takeoff_win, &ControllerTakeoffWindow::take_off_info_signal, this, &ControllerMainWindow::take_off_info_slot);
            QObject::connect(generate_button, &QPushButton::clicked, this, &ControllerMainWindow::ext_cmd_generate_slot);
            QObject::connect(this, &ControllerMainWindow::update_controller_cell_info_signal, this, &ControllerMainWindow::update_controller_cell_info_slot);
            QObject::connect(this, &ControllerMainWindow::update_monitor_cell_info_signal, this, &ControllerMainWindow::update_monitor_cell_info_slot);
            QObject::connect(this, &ControllerMainWindow::update_controller_info_signal, this, &ControllerMainWindow::update_info_slot);
            QObject::connect(this, &ControllerMainWindow::update_button_state_signal, this, &ControllerMainWindow::update_button_state_slot);

            // thread
            for (size_t i = 0; i < nodes.size(); i++)
            {
                std::thread update_controller_table_thread(&ControllerMainWindow::update_controller_table_info, this, i);
                update_controller_table_thread.detach();
                std::thread update_monitor_table_thread(&ControllerMainWindow::update_monitor_table_info, this, i);
                update_monitor_table_thread.detach();
            }
            std::thread update_thread(&ControllerMainWindow::update_info, this);
            update_thread.detach();
        }

        // ros thread
        void ros_thread_func(int node_id)
        {
            while (ros::ok() && !thread_stop)
            {
                pubs[node_id].publish(cmds[node_id]);
                ros::spinOnce();
                usleep(20000);
            }
        }

        // slot functions
        void info_window_slot()
        {
            info_win->show();
        }

        void mode_window_slot()
        {
            mode_win->set_data(data);
            mode_win->exec();
        }

        void arm_slot()
        {
            operating_info = "Arming...";
            std::thread arm_thread(&ControllerMainWindow::arm_thread_func, this);
            arm_thread.detach();
        }

        void arm_thread_func()
        {
            QVariant button_data;
            button_info button;
            string err = "";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                err = data[i]->set_mode("Arm");
                if (err != "")
                {
                    msg_box = new QMessageBox(this);
                    msg_box->setIcon(QMessageBox::Icon::Critical);
                    msg_box->setWindowTitle("Error");
                    msg_box->setText("Armed Failure, Please Retry.");
                    msg_box->exec();
                    for (size_t j = 0; j < nodes.size(); j++)
                    {
                        data[i]->set_mode(mavros_msgs::State::MODE_PX4_RTL);
                    }
                    operating_info = "";
                    return;
                }
            }
            operating_info = "";
            current_cmd = "Arm";
            button.button_name = arm_button;
            button.state = false;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = disarm_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
        }

        void disarm_slot()
        {
            operating_info = "DisArming...";
            std::thread disarm_thread(&ControllerMainWindow::disarm_thread_func, this);
            disarm_thread.detach();
        }

        void disarm_thread_func()
        {
            QVariant button_data;
            button_info button;
            string err = "";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                err = data[i]->set_mode("DisArm");
                if (err != "")
                {
                    msg_box = new QMessageBox(this);
                    msg_box->setIcon(QMessageBox::Icon::Critical);
                    msg_box->setWindowTitle("Error");
                    msg_box->setText(err.c_str());
                    msg_box->exec();
                    operating_info = "";
                    return;
                }
            }
            operating_info = "";
            current_cmd = "DisArm";
            button.button_name = arm_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = disarm_button;
            button.state = false;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
        }

        void take_off_window_slot()
        {
            if (!takeoff_win->set_data(data))
            {
                return;
            }
            takeoff_win->exec();
            if (takeoff_win->set_height)
            {
                takeoff_button->setEnabled(false);
                arm_button->setEnabled(false);
                mode_button->setEnabled(false);
                std::thread take_off_thread(&ControllerMainWindow::take_off_thread_func, this);
                take_off_thread.detach();
            }
        }

        void take_off_thread_func()
        {
            QVariant button_data;
            button_info button;
            string err = "";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                cmds[i].Mode = px4_cmd::Command::Takeoff;
                cmds[i].Move_frame = px4_cmd::Command::ENU;
                cmd_values[i][0] = data[i]->home_position[0] + data[i]->init_x;
                cmd_values[i][1] = data[i]->home_position[1] + data[i]->init_y;
                cmd_values[i][2] = takeoff_win->takeoff_height;
                cmd_values[i][3] = 0;
                cmds[i].desire_cmd[0] = data[i]->home_position[0];
                cmds[i].desire_cmd[1] = data[i]->home_position[1];
                cmds[i].desire_cmd[2] = cmd_values[i][2];
                cmds[i].yaw_cmd = cmd_values[i][3] * PI / 180;
                err = data[i]->set_mode("Arm");
                if (err != "")
                {
                    msg_box = new QMessageBox(this);
                    msg_box->setIcon(QMessageBox::Icon::Critical);
                    msg_box->setWindowTitle("Error");
                    msg_box->setText(("Armed Failure:" + err + "\nPlease Retry...").c_str());
                    msg_box->exec();
                    for (size_t j = 0; j < nodes.size(); j++)
                    {
                        data[i]->set_mode(mavros_msgs::State::MODE_PX4_RTL);
                    }
                    operating_info = "";
                    signal_button_2->click();
                    return;
                }
            }
            operating_info = "";
            current_cmd = "Take Off";
            button.button_name = mode_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = takeoff_button;
            button.state = false;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = arm_button;
            button.state = false;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = disarm_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = manual_button;
            button.state = true;
            manual_button->setEnabled(true);
            button.button_name = trajectory_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = external_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = hover_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = land_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = return_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
        }

        void trajectory_cmd_slot()
        {
            trajectory_win->set_nodes(nodes);
            trajectory_win->exec();
            if (trajectory_win->exec_state)
            {
                current_cmd = "Trajectory CMD";
                std::thread trajectory_cmd_thread(&ControllerMainWindow::trajectory_cmd_thread_func, this);
                trajectory_cmd_thread.detach();
                usleep(500000);
            }
        }

        void trajectory_cmd_thread_func()
        {
            double sleep_time = trajectory_win->set_time;
            for (size_t i = 0; i < trajectory_win->cmd_values.size(); i++)
            {
                cmd_mutex.lock();
                cmd_values = trajectory_win->cmd_values[i];
                cmd_mutex.unlock();
                // set cmd
                for (size_t j = 0; j < cmds.size(); j++)
                {
                    cmds[j].Mode = px4_cmd::Command::Move;
                    cmds[j].Move_mode = trajectory_win->set_mode;
                    cmds[j].Move_frame = trajectory_win->set_frame;
                    if (cmds[j].Move_mode == px4_cmd::Command::XYZ_REL_POS)
                    {
                        cmds[j].desire_cmd[0] = cmds[j].desire_cmd[0] + cmd_values[j][0] + data[j]->home_position[0];
                        cmds[j].desire_cmd[1] = cmds[j].desire_cmd[1] + cmd_values[j][1] + data[j]->home_position[1];
                        cmds[j].desire_cmd[2] = cmds[j].desire_cmd[2] + cmd_values[j][2];
                    }
                    else
                    {
                        cmds[j].desire_cmd[0] = cmd_values[j][0];
                        cmds[j].desire_cmd[1] = cmd_values[j][1];
                        cmds[j].desire_cmd[2] = cmd_values[j][2];
                        if (cmds[j].Move_mode == px4_cmd::Command::XYZ_POS)
                        {
                            cmds[j].desire_cmd[0] = cmds[j].desire_cmd[0] - data[j]->init_x;
                            cmds[j].desire_cmd[1] = cmds[j].desire_cmd[1] - data[j]->init_y;
                            cmds[j].desire_cmd[2] = cmds[j].desire_cmd[2] - data[j]->init_z;
                        }
                        else
                        {
                            if (cmds[j].Move_mode == px4_cmd::Command::XY_VEL_Z_POS)
                            {
                                cmds[j].desire_cmd[2] = cmds[j].desire_cmd[2] - data[j]->init_z;
                            }
                        }
                    }
                    cmds[j].yaw_cmd = cmd_values[j][3] * PI / 180;
                }
                operating_info = ("Flying to Trajectory Point [" + to_string(i) + "] ...").c_str();
                usleep(500000);
                // judge if achieve desire cmd
                while (!judge_all_achieve_state(true))
                {
                    usleep(200000);
                }
                // change yaw values to next cmd
                if (i < (trajectory_win->cmd_values.size() - 1))
                {
                    operating_info = ("Wait for Next Point [" + to_string(i + 1) + "] ...").c_str();
                    for (size_t j = 0; j < cmds.size(); j++)
                    {
                        cmd_values[j][3] = trajectory_win->cmd_values[i + 1][j][3];
                        cmds[j].yaw_cmd = trajectory_win->cmd_values[i + 1][j][3] * PI / 180;
                    }
                }
                // wait for gap time
                usleep(floor(sleep_time * 1000000));
            }
            operating_info = "Trajectory CMD Done. Change to Hover Mode.";
            sleep(3);
            hover_button->click();
        }

        void ext_cmd_slot()
        {
            if (!ext_cmd_state)
            {
                msg_box = new QMessageBox(this);
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setWindowTitle("Error");
                msg_box->setText("Not Detect External Command Topic, Please Check.");
                msg_box->exec();
                return;
            }
            current_cmd = "External Command";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                std::thread ext_cmd_thread(&ControllerMainWindow::ext_cmd_thread_func, this, i);
                ext_cmd_thread.detach();
            }
            usleep(500000);
            manual_button->setEnabled(false);
            trajectory_button->setEnabled(false);
            external_button->setEnabled(false);
        }

        void ext_cmd_thread_func(int node_id)
        {
            while (ext_cmd_state && (current_cmd == "External Command"))
            {
                // basic information
                cmds[node_id].Mode = data[node_id]->ext_cmd.Mode;
                cmds[node_id].Move_mode = data[node_id]->ext_cmd.Move_mode;
                cmds[node_id].Move_frame = data[node_id]->ext_cmd.Move_frame;
                cmds[node_id].Vehicle = data[node_id]->ext_cmd.Vehicle;
                cmds[node_id].Custom_Command_Mode = data[node_id]->ext_cmd.Custom_Command_Mode;
                if (data[node_id]->vehicle_type == px4_cmd::Command::FixWing)
                {
                    cmds[node_id].fx_custom_mode = data[node_id]->ext_cmd.fx_custom_mode;
                }

                // custom command 
                if (cmds[node_id].Move_mode == px4_cmd::Command::Custom_Command)
                {
                    // update Gui info
                    cmd_mutex.lock();
                    int count = 0;
                    while (count < 3)
                    {
                        int i = 0;
                        if (isnan(data[node_id]->ext_cmd.custom_cmd[i]))
                        {
                            i++;
                            continue;
                        }
                        cmd_values[node_id][count] = data[node_id]->ext_cmd.custom_cmd[i];
                        i++;
                        count++;
                    }
                    cmd_mutex.unlock();

                    // transform data
                    for (size_t i = 0; i < 20; i++)
                    {
                        cmds[node_id].custom_cmd[i] = data[node_id]->ext_cmd.custom_cmd[i];
                    }

                    if (cmds[node_id].Custom_Command_Mode == px4_cmd::Command::Custom_Command_TargetLocal)
                    {
                        if (!isnan(cmds[node_id].custom_cmd[0]))
                        {
                            cmds[node_id].custom_cmd[0] = cmds[node_id].custom_cmd[0] - data[node_id]->init_x;
                        }
                        if (!isnan(cmds[node_id].custom_cmd[1]))
                        {
                            cmds[node_id].custom_cmd[1] = cmds[node_id].custom_cmd[1] - data[node_id]->init_y;
                        }
                        if (!isnan(cmds[node_id].custom_cmd[2]))
                        {
                            cmds[node_id].custom_cmd[2] = cmds[node_id].custom_cmd[2] - data[node_id]->init_z;
                        }
                    }
                    cmds[node_id].yaw_cmd = cmd_values[node_id][3] * PI / 180;
                    data[node_id]->ext_cmd_sub_state = true;
                    usleep(20000);
                    continue;
                }

                // common command
                cmd_mutex.lock();
                cmd_values[node_id][0] = data[node_id]->ext_cmd.desire_cmd[0];
                cmd_values[node_id][1] = data[node_id]->ext_cmd.desire_cmd[1];
                cmd_values[node_id][2] = data[node_id]->ext_cmd.desire_cmd[2];
                cmd_values[node_id][3] = data[node_id]->ext_cmd.yaw_cmd * 180 / PI;
                cmd_mutex.unlock();
                if (cmds[node_id].Move_mode == px4_cmd::Command::XYZ_REL_POS)
                {
                    cmds[node_id].desire_cmd[0] = cmds[node_id].desire_cmd[0] + cmd_values[node_id][0] + data[node_id]->home_position[0];
                    cmds[node_id].desire_cmd[1] = cmds[node_id].desire_cmd[1] + cmd_values[node_id][1] + data[node_id]->home_position[1];
                    cmds[node_id].desire_cmd[2] = cmds[node_id].desire_cmd[2] + cmd_values[node_id][2];
                }
                else
                {
                    cmds[node_id].desire_cmd[0] = cmd_values[node_id][0];
                    cmds[node_id].desire_cmd[1] = cmd_values[node_id][1];
                    cmds[node_id].desire_cmd[2] = cmd_values[node_id][2];
                    if (cmds[node_id].Move_mode == px4_cmd::Command::XYZ_POS)
                    {
                        cmds[node_id].desire_cmd[0] = cmds[node_id].desire_cmd[0] - data[node_id]->init_x;
                        cmds[node_id].desire_cmd[1] = cmds[node_id].desire_cmd[1] - data[node_id]->init_y;
                        cmds[node_id].desire_cmd[2] = cmds[node_id].desire_cmd[2] - data[node_id]->init_z;
                    }
                    else
                    {
                        if (cmds[node_id].Move_mode == px4_cmd::Command::XY_VEL_Z_POS)
                        {
                            cmds[node_id].desire_cmd[2] = cmds[node_id].desire_cmd[2] - data[node_id]->init_z;
                        }
                    }
                }
                cmds[node_id].yaw_cmd = cmd_values[node_id][3] * PI / 180;
                data[node_id]->ext_cmd_sub_state = true;
                usleep(20000);
            }
            data[node_id]->ext_cmd_sub_state = false;
            if (!ext_cmd_state)
            {
                if (data[node_id]->vehicle_type == px4_cmd::Command::FixWing)
                {
                    cmds[node_id].Mode = px4_cmd::Command::Loiter;
                }
                else
                {
                    cmds[node_id].Mode = px4_cmd::Command::Hover;
                }
            }
            manual_button->setEnabled(true);
            external_button->setEnabled(true);
            trajectory_button->setEnabled(true);
        }

        void hover_slot()
        {
            operating_info = "";
            current_cmd = "Hover";
            std::thread hover_thread(&ControllerMainWindow::hover_thread_func, this);
            hover_thread.detach();
        }

        void hover_thread_func()
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->vehicle_type == px4_cmd::Command::FixWing)
                {
                    cmds[i].Mode = px4_cmd::Command::Loiter;
                    continue;
                }
                cmds[i].Mode = px4_cmd::Command::Hover;
            }
        }

        void land_slot()
        {
            current_cmd = "Land";
            mode_button->setEnabled(false);
            takeoff_button->setEnabled(false);
            arm_button->setEnabled(false);
            manual_button->setEnabled(false);
            trajectory_button->setEnabled(false);
            external_button->setEnabled(false);
            hover_button->setEnabled(false);
            land_button->setEnabled(false);
            return_button->setEnabled(false);
            std::thread land_thread(&ControllerMainWindow::land_thread_func, this);
            land_thread.detach();
        }

        void land_thread_func()
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                data[i]->set_mode(mavros_msgs::State::MODE_PX4_LAND);
            }
        }

        void return_slot()
        {
            current_cmd = "Return";
            mode_button->setEnabled(false);
            takeoff_button->setEnabled(false);
            arm_button->setEnabled(false);
            manual_button->setEnabled(false);
            trajectory_button->setEnabled(false);
            external_button->setEnabled(false);
            hover_button->setEnabled(false);
            land_button->setEnabled(false);
            // fix wing need to land manually
            if (fix_wing_include)
            {
                land_button->setEnabled(true);
            }
            return_button->setEnabled(false);
            std::thread return_thread(&ControllerMainWindow::return_thread_func, this);
            return_thread.detach();
        }

        void return_thread_func()
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                data[i]->set_mode(mavros_msgs::State::MODE_PX4_RTL);
            }
        }

        void change_mode_slot()
        {
            current_cmd = "Change Mode";
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
            usleep(20000);
            this->close();
        }

        void take_off_info_slot()
        {
            operating_info = "Take off and Arming...";
        }

        void manual_cmd_slot()
        {
            manual_win->set_nodes(nodes);
            manual_win->exec();
            if (manual_win->exec_state)
            {
                current_cmd = "Manual CMD";
                if (cmds[0].Move_mode == px4_cmd::Command::XYZ_REL_POS || cmds[0].Move_mode == px4_cmd::Command::XYZ_POS)
                {
                    operating_info = "Flying to Set Point...";
                }
                else
                {
                    operating_info = "Flying with Desire Velocity ...";
                }
                std::thread manual_cmd_thread(&ControllerMainWindow::manual_cmd_thread_func, this);
                manual_cmd_thread.detach();
            }
        }

        void manual_cmd_thread_func()
        {
            bool achieve = false;
            cmd_mutex.lock();
            cmd_values = manual_win->cmd_values[0];
            cmd_mutex.unlock();
            for (size_t i = 0; i < cmds.size(); i++)
            {
                cmds[i].Mode = px4_cmd::Command::Move;
                cmds[i].Move_mode = manual_win->set_mode;
                cmds[i].Move_frame = manual_win->set_frame;
                if (cmds[i].Move_mode == px4_cmd::Command::XYZ_REL_POS)
                {
                    cmds[i].desire_cmd[0] = cmds[i].desire_cmd[0] + cmd_values[i][0];
                    cmds[i].desire_cmd[1] = cmds[i].desire_cmd[1] + cmd_values[i][1];
                    cmds[i].desire_cmd[2] = cmds[i].desire_cmd[2] + cmd_values[i][2];
                }
                else
                {
                    cmds[i].desire_cmd[0] = cmd_values[i][0];
                    cmds[i].desire_cmd[1] = cmd_values[i][1];
                    cmds[i].desire_cmd[2] = cmd_values[i][2];
                    if (cmds[i].Move_mode == px4_cmd::Command::XYZ_POS)
                    {
                        cmds[i].desire_cmd[0] = cmds[i].desire_cmd[0] - data[i]->init_x;
                        cmds[i].desire_cmd[1] = cmds[i].desire_cmd[1] - data[i]->init_y;
                        cmds[i].desire_cmd[2] = cmds[i].desire_cmd[2] - data[i]->init_z;
                    }
                    else
                    {
                        if (cmds[i].Move_mode == px4_cmd::Command::XY_VEL_Z_POS)
                        {
                            cmds[i].desire_cmd[2] = cmds[i].desire_cmd[2] - data[i]->init_z;
                        }
                    }
                }
                cmds[i].yaw_cmd = cmd_values[i][3] * PI / 180;
            }
            // judge if achieve desire cmd
            sleep(1);
            while (!judge_all_achieve_state(true))
            {
                usleep(200000);
            }
            operating_info = "Manual CMD Done. Change to Hover Mode";
            sleep(3);
            hover_button->click();
        }

        void ext_cmd_err_msg_slot()
        {
            msg_box = new QMessageBox(this);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            msg_box->setText("Disconnection to External Command Topic, Please Check. Change to Hover Mode.");
            msg_box->exec();
        }

        void ext_cmd_generate_slot()
        {
            generate_win->exec();
        }

        void arm_fail_slot()
        {
            mode_button->setEnabled(true);
            takeoff_button->setEnabled(true);
            arm_button->setEnabled(true);
        }

        void update_info()
        {
            while (ros::ok() && !thread_stop)
            {
                emit update_controller_info_signal();
                signal_button_3->click();
                usleep(100000);
            }
        }

        void update_info_slot()
        {
            bool pre_arm_state = false;
            bool arm_state = false;
            bool ext_cmd_state_single = false;
            QString pre_cmd = "None";
            string current_mode = "";
            string pre_mode = "";
            QStringList table_headers_ext_cmd = {
                "Node",
                "Vehicle Type",
                "CMD Mode",
                "Frame",
                "CMD 1",
                "CMD 2",
                "CMD 3",
                "CMD 4  [Yaw]",
                "External State",
                "External CMD Topic"
            };
        
            // ros state
            if (ros::master::check())
            {
                info_label_1->setText("[ROS State]  Running");
                info_label_1->setStyleSheet("color: green; font-size: 14pt");
                info_label_2->setText(("[Vehicle Count]  " + to_string(nodes.size())).c_str());
                info_label_2->setStyleSheet("color: green; font-size: 14pt");
            }
            else
            {
                info_label_1->setText("[ROS State]  Not Running");
                info_label_1->setStyleSheet("color: red; font-size: 14pt");
                info_label_2->setText("[Vehicle Count]  0");
                info_label_2->setStyleSheet("color: green; font-size: 14pt");
            }

            // arm state
            current_mode = data[0]->state_mode;
            if (current_mode == mavros_msgs::State::MODE_PX4_RTL || current_mode == mavros_msgs::State::MODE_PX4_LAND)
            {
                arm_state = judge_all_arm_state(false);
            }
            else
            {
                arm_state = judge_all_arm_state(true);
            }
            if (arm_state != pre_arm_state || current_mode != pre_mode)
            {
                if (arm_state)
                {
                    state_label_1->setText("[Arm State]  Arm");
                    state_label_1->setStyleSheet("color: green; font-size: 14pt");
                    arm_button->setEnabled(false);
                    disarm_button->setEnabled(true);
                    if (judge_all_land_state(false))
                    {
                        mode_button->setEnabled(true);
                        manual_button->setEnabled(true);
                        trajectory_button->setEnabled(true);
                        external_button->setEnabled(true);
                        hover_button->setEnabled(true);
                        land_button->setEnabled(true);
                        return_button->setEnabled(true);
                    }
                    if ((current_mode == mavros_msgs::State::MODE_PX4_LAND) && !judge_all_land_state(false))
                    {
                        operating_info = "Landing...";
                        mode_button->setEnabled(false);
                        takeoff_button->setEnabled(false);
                        arm_button->setEnabled(false);
                        disarm_button->setEnabled(false);
                        manual_button->setEnabled(false);
                        trajectory_button->setEnabled(false);
                        external_button->setEnabled(false);
                        hover_button->setEnabled(false);
                        land_button->setEnabled(false);
                        return_button->setEnabled(false);
                    }
                    if ((current_mode == mavros_msgs::State::MODE_PX4_RTL) && !judge_all_land_state(false))
                    {
                        operating_info = "Returning...";
                        mode_button->setEnabled(false);
                        takeoff_button->setEnabled(false);
                        arm_button->setEnabled(false);
                        disarm_button->setEnabled(false);
                        manual_button->setEnabled(false);
                        trajectory_button->setEnabled(false);
                        external_button->setEnabled(false);
                        hover_button->setEnabled(false);
                        land_button->setEnabled(false);
                        // fix wing need to land manually
                        if (fix_wing_include)
                        {
                            land_button->setEnabled(true);
                        }
                        return_button->setEnabled(false);
                    }
                }
                else
                {
                    state_label_1->setText("[Arm State]  DisArm");
                    state_label_1->setStyleSheet("color: orange; font-size: 14pt");
                    arm_button->setEnabled(true);
                    disarm_button->setEnabled(false);
                    takeoff_button->setEnabled(true);
                    manual_button->setEnabled(false);
                    trajectory_button->setEnabled(false);
                    external_button->setEnabled(false);
                    hover_button->setEnabled(false);
                    land_button->setEnabled(false);
                    return_button->setEnabled(false);
                    if (current_mode == mavros_msgs::State::MODE_PX4_LAND || current_mode == mavros_msgs::State::MODE_PX4_RTL)
                    {
                        mode_button->setEnabled(true);
                        operating_info = "";
                    }
                }
                pre_arm_state = arm_state;
            }

            // current cmd
            if (pre_cmd != current_cmd)
            {
                state_label_2->setText("[Current CMD]  " + current_cmd);
                pre_cmd = current_cmd;
            }
            if (current_cmd == "External Command" && !ext_cmd_state)
            {
                current_cmd = "Hover";
                signal_button_1->click();
            }

            // update headers
            // update cmd mode info
            if (current_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                if (current_cmd == "Take Off")
                {
                    table_headers_ext_cmd[4] = "CMD 1  [x]";
                    table_headers_ext_cmd[5] = "CMD 2  [y]";
                    table_headers_ext_cmd[6] = "CMD 3  [z]";
                }
                else
                {
                    switch (cmds[0].Move_mode)
                    {
                        case px4_cmd::Command::XYZ_POS:
                            table_headers_ext_cmd[4] = "CMD 1  [x]";
                            table_headers_ext_cmd[5] = "CMD 2  [y]";
                            table_headers_ext_cmd[6] = "CMD 3  [z]";
                            break;

                        case px4_cmd::Command::XYZ_REL_POS:
                            table_headers_ext_cmd[4] = "CMD 1  [Rel x]";
                            table_headers_ext_cmd[5] = "CMD 2  [Rel y]";
                            table_headers_ext_cmd[6] = "CMD 3  [Rel z]";
                            break;

                        case px4_cmd::Command::XYZ_VEL:
                            table_headers_ext_cmd[4] = "CMD 1  [vx]";
                            table_headers_ext_cmd[5] = "CMD 2  [vy]";
                            table_headers_ext_cmd[6] = "CMD 3  [vz]";
                            break;

                        case px4_cmd::Command::XY_VEL_Z_POS:
                            table_headers_ext_cmd[4] = "CMD 1  [vx]";
                            table_headers_ext_cmd[5] = "CMD 2  [vy]";
                            table_headers_ext_cmd[6] = "CMD 3  [z]";
                            break;
                    }
                }
            }
            else
            {
                table_headers_ext_cmd[4] = "CMD 1";
                table_headers_ext_cmd[5] = "CMD 2";
                table_headers_ext_cmd[6] = "CMD 3";
            }
            controller_info_model->setHorizontalHeaderLabels(table_headers_ext_cmd);

            // current mode
            if (pre_mode != current_mode)
            {
                info_label_3->setText(("[Current Mode]  " + current_mode).c_str());
                pre_mode = current_mode;
                if (current_mode == mavros_msgs::State::MODE_PX4_LAND)
                {
                    current_cmd = "Land";
                    state_label_2->setText("[Current CMD]  " + current_cmd);
                    pre_cmd = current_cmd;
                }
                if (current_mode == mavros_msgs::State::MODE_PX4_RTL)
                {
                    current_cmd = "Return";
                    state_label_2->setText("[Current CMD]  " + current_cmd);
                    pre_cmd = current_cmd;
                }
            }

            // operating info
            state_label_3->setText(operating_info);

            // ext_cmd_state
            ext_cmd_state_single = false;
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->ext_cmd_pub_state)
                {
                    ext_cmd_state_single = true;
                    continue;
                }
                ext_cmd_state_single = false;
                break;
            }
            ext_cmd_state = ext_cmd_state_single;
        }

        void update_controller_table_info(int node_id)
        {
            QString cmd_mode = "Loiter";
            QString pre_cmd = "";
            QString frame = "";
            string current_mode = "";
            string pre_mode = "";
            QString null_str = "  -----------  ";
            QString cmd1 = "";
            QString cmd2 = "";
            QString cmd3 = "";
            QString cmd4 = "";
            QVariant info;
            cell_info info1, info2, info3, info4, info5, info6, info7;
            info1.node_id = node_id;
            info2.node_id = node_id;
            info3.node_id = node_id;
            info4.node_id = node_id;
            info5.node_id = node_id;
            info6.node_id = node_id;
            info7.node_id = node_id;
            info1.cell_id = 2;
            info2.cell_id = 3;
            info3.cell_id = 4;
            info4.cell_id = 5;
            info5.cell_id = 6;
            info6.cell_id = 7;
            info7.cell_id = 8;
            while (ros::master::check() && !thread_stop)
            {
                // current mode
                current_mode = data[node_id]->state_mode;
                if (pre_mode != current_mode || pre_cmd != current_cmd)
                {
                    pre_mode = current_mode;
                    pre_cmd = current_cmd;
                    if (current_mode != mavros_msgs::State::MODE_PX4_OFFBOARD)
                    {
                        info2.str = null_str;
                        info3.str = null_str;
                        info4.str = null_str;
                        info5.str = null_str;
                        info6.str = null_str;
                        info.setValue(info2);
                        emit update_controller_cell_info_signal(info);
                        info.setValue(info3);
                        emit update_controller_cell_info_signal(info);
                        info.setValue(info4);
                        emit update_controller_cell_info_signal(info);
                        info.setValue(info5);
                        emit update_controller_cell_info_signal(info);
                        info.setValue(info6);
                        emit update_controller_cell_info_signal(info);
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_LOITER)
                    {
                        cmd_mode = "Loiter";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_MANUAL)
                    {
                        cmd_mode = "Manual";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_LAND)
                    {
                        cmd_mode = "Land";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_RTL)
                    {
                        cmd_mode = "Return";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_POSITION)
                    {
                        cmd_mode = "Postition Control";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_ALTITUDE)
                    {
                        cmd_mode = "Altitude Control";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_STABILIZED)
                    {
                        cmd_mode = "Stabilized";
                    }
                }

                // ext_cmd_publish_info
                if (ext_cmd_state)
                {
                    info7.str = "  Active  ";
                    info.setValue(info7);
                    emit update_controller_cell_info_signal(info);
                }
                else
                {
                    info7.str = "  Deactive  ";
                    info.setValue(info7);
                    emit update_controller_cell_info_signal(info);
                }

                // table info
                if (current_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    // update cmd mode
                    if (current_cmd == "Take Off")
                    {
                        cmd_mode = "Take Off";
                    }
                    else
                    {
                        switch (cmds[0].Move_mode)
                        {
                            case px4_cmd::Command::XYZ_POS:
                                cmd_mode = "Postion";
                                break;

                            case px4_cmd::Command::XYZ_REL_POS:
                                cmd_mode = "Rel Position";
                                break;

                            case px4_cmd::Command::XYZ_VEL:
                                cmd_mode = "Velocity";
                                break;

                            case px4_cmd::Command::XY_VEL_Z_POS:
                                cmd_mode = "Velocity with Height";
                                break;
                        }
                    }

                    // update frame info
                    switch (cmds[0].Move_frame)
                    {
                        case px4_cmd::Command::ENU:
                            frame = "ENU";
                            break;

                        case px4_cmd::Command::BODY:
                            frame = "Body";
                            break;
                    }
                    // set frame info
                    info2.str = "  " + frame + "  ";
                    info.setValue(info2);
                    emit update_controller_cell_info_signal(info);
                }
                
                // set cmd mode info
                info1.str = "  " + cmd_mode + "  ";
                info.setValue(info1);
                emit update_controller_cell_info_signal(info);

                if (current_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    cmd_mutex.lock();
                    info3.str = ("  " + to_string(cmd_values[node_id][0]) + "  ").c_str();
                    info4.str = ("  " + to_string(cmd_values[node_id][1]) + "  ").c_str();
                    info5.str = ("  " + to_string(cmd_values[node_id][2]) + "  ").c_str();
                    info6.str = ("  " + to_string(cmd_values[node_id][3]) + "  ").c_str();
                    cmd_mutex.unlock();
                    info.setValue(info3);
                    emit update_controller_cell_info_signal(info);
                    info.setValue(info4);
                    emit update_controller_cell_info_signal(info);
                    info.setValue(info5);
                    emit update_controller_cell_info_signal(info);
                    info.setValue(info6);
                    emit update_controller_cell_info_signal(info);
                }
                // sleep
                usleep(200000);
            }
        }

        void update_controller_cell_info_slot(QVariant info)
        {
            cell_info data = info.value<cell_info>();
            controller_table_items[data.node_id][data.cell_id]->setText(data.str);
            if (ext_cmd_state)
            {
                controller_info_model->setData(controller_info_model->index(data.node_id, 8), QBrush(QColor(0, 150, 0)), Qt::TextColorRole);
            }
            else
            {
                controller_info_model->setData(controller_info_model->index(data.node_id, 8), QBrush(Qt::red), Qt::TextColorRole);
            }
        };

        void update_monitor_table_info(int thread_id)
        {
            vehicle_state *vec = uav_state_data[thread_id];
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
                emit update_monitor_cell_info_signal(info);
                info.setValue(info4);
                emit update_monitor_cell_info_signal(info);
                info.setValue(info5);
                emit update_monitor_cell_info_signal(info);
                info.setValue(info6);
                emit update_monitor_cell_info_signal(info);
                info.setValue(info7);
                emit update_monitor_cell_info_signal(info);
                info.setValue(info8);
                emit update_monitor_cell_info_signal(info);
                info.setValue(info9);
                emit update_monitor_cell_info_signal(info);
                info.setValue(info10);
                emit update_monitor_cell_info_signal(info);
                info.setValue(info11);
                emit update_monitor_cell_info_signal(info);
                info.setValue(info12);
                emit update_monitor_cell_info_signal(info);
                usleep(100000);
            }
        }

        void update_monitor_cell_info_slot(QVariant info)
        {
            cell_info data = info.value<cell_info>();
            monitor_table_items[data.node_id][data.cell_id]->setText(data.str);
        };

        void update_button_state_slot(QVariant state)
        {
            button_info data = state.value<button_info>();
            data.button_name->setEnabled(data.state);
        };

        void update_plot_mode_slot()
        {
            int current_index = plot_select->currentIndex();
            // x - y
            if (current_index == 0)
            {
                plot->xAxis->setLabel("x (meter)");
                plot->yAxis->setLabel("y (meter)");
            }
            // height
            if (current_index == 1)
            {
                plot->xAxis->setLabel("Time (second)");
                plot->yAxis->setLabel("Height (meter)");
            }
            // roll
            if (current_index == 2)
            {
                plot->xAxis->setLabel("Time (second)");
                plot->yAxis->setLabel("Roll (rad)");
            }
            // pitch
            if (current_index == 3)
            {
                plot->xAxis->setLabel("Time (second)");
                plot->yAxis->setLabel("Pitch (rad)");
            }
            // yaw
            if (current_index == 4)
            {
                plot->xAxis->setLabel("Time (second)");
                plot->yAxis->setLabel("Yaw (rad)");
            }
            update_plot_uav_slot();
        }

        void update_plot_slot()
        {
            int i = 0;
            int current_index = plot_select->currentIndex();
            for (auto item = plot_uav_state_data.begin(); item != plot_uav_state_data.end(); item++)
            {
                // x- y
                if (current_index == 0)
                {
                    plot_curves[i]->setData((*item)->count_pose, (*item)->x, (*item)->y);
                    if (i == 0)
                    {
                        plot_curves[i]->rescaleAxes();
                    }
                    else
                    {
                        plot_curves[i]->rescaleAxes(true);
                    }
                    x_end[0] = *((*item)->x.end() - 1);
                    y_end[0] = *((*item)->y.end() - 1);
                    plot->graph(i)->setData(x_end, y_end);
                    i++;
                }
                // height
                if (current_index == 1)
                {
                    plot_curves[i]->setData((*item)->count_pose, (*item)->t_pose, (*item)->z);
                    if (i == 0)
                    {
                        plot_curves[i]->rescaleAxes();
                    }
                    else
                    {
                        plot_curves[i]->rescaleAxes(true);
                    }
                    x_end[0] = *((*item)->t_pose.end() - 1);
                    y_end[0] = *((*item)->z.end() - 1);
                    plot->graph(i)->setData(x_end, y_end);
                    i++;
                }
                // roll
                if (current_index == 2)
                {
                    plot_curves[i]->setData((*item)->count_pose, (*item)->t_pose, (*item)->roll);
                    if (i == 0)
                    {
                        plot_curves[i]->rescaleAxes();
                    }
                    else
                    {
                        plot_curves[i]->rescaleAxes(true);
                    }
                    x_end[0] = *((*item)->t_pose.end() - 1);
                    y_end[0] = *((*item)->roll.end() - 1);
                    plot->graph(i)->setData(x_end, y_end);
                    i++;
                }
                // pitch
                if (current_index == 3)
                {
                    plot_curves[i]->setData((*item)->count_pose, (*item)->t_pose, (*item)->pitch);
                    if (i == 0)
                    {
                        plot_curves[i]->rescaleAxes();
                    }
                    else
                    {
                        plot_curves[i]->rescaleAxes(true);
                    }
                    x_end[0] = *((*item)->t_pose.end() - 1);
                    y_end[0] = *((*item)->pitch.end() - 1);
                    plot->graph(i)->setData(x_end, y_end);
                    i++;
                }
                // yaw
                if (current_index == 4)
                {
                    plot_curves[i]->setData((*item)->count_pose, (*item)->t_pose, (*item)->yaw);
                    if (i == 0)
                    {
                        plot_curves[i]->rescaleAxes();
                    }
                    else
                    {
                        plot_curves[i]->rescaleAxes(true);
                    }
                    x_end[0] = *((*item)->t_pose.end() - 1);
                    y_end[0] = *((*item)->yaw.end() - 1);
                    plot->graph(i)->setData(x_end, y_end);
                    i++;
                }
            }
            plot->yAxis->scaleRange(1.5, plot->yAxis->range().center());
            plot->xAxis->scaleRange(1.5, plot->xAxis->range().center());
            plot->replot();
        }

        void update_plot_uav_slot()
        {
            QPen pen;
            QList<int> uavs_idx = plot_uav_select->selectedDataIndex();
            plot_uav_state_data.clear();
            signal_button_3->blockSignals(true);
            for (auto uav_idx : uavs_idx)
            {
                plot_uav_state_data.append(uav_state_data[uav_idx]);
            }
            int count = plot_uav_state_data.size();
            plot_curves.clear();
            plot->clearGraphs();
            plot->clearPlottables();
            for (int i = 0; i < count; i++)
            {
                pen.setColor(QColor::fromRgb(gen_rand(255), gen_rand(255), gen_rand(255)));
                QCPCurve *curve = new QCPCurve(plot->xAxis, plot->yAxis);
                curve->setPen(pen);
                curve->removeFromLegend();
                curve->setAntialiasedFill(true);
                plot_curves.push_back(curve);
            }
            for (int i = 0; i < count; i++)
            {
                pen.setColor(QColor::fromRgb(gen_rand(255), gen_rand(255), gen_rand(255)));
                plot->addGraph()->setName(nodes[uavs_idx[i]]);
                plot->graph()->setPen(pen);
                plot->graph()->setLineStyle(QCPGraph::LineStyle::lsNone);
                plot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 8));
                plot->graph()->setAntialiasedFill(true);
            }
            for (int i = 1; i < count; i++)
            {
                plot->legend->addElement(0, i, plot->legend->item(i));
            }
            if (plot_curves.size() > 0)
            {
                signal_button_3->blockSignals(false);
            }
            else
            {
                plot->replot();
            }
        }

        void image_topic_window_slot()
        {
            ControllerImageWindow *img_win = new ControllerImageWindow(nh, uav_state_data);
            img_win->setWindowModality(Qt::NonModal);
            img_win->open();
        }

        void run_ros_node_window_slot()
        {
            if (node_win == nullptr)
            {
                node_win = new ControllerRunNodeWindow(nh);
                node_win->setWindowModality(Qt::NonModal);
            }
            else
            {
                node_win->reset_window();
            }
            node_win->open();
        }

        // utility functions
        bool judge_all_arm_state(bool desire)
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->arm_state != desire)
                {
                    return !desire;
                }
            }
            return desire;
        }

        bool judge_all_land_state(bool desire)
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->land_state != desire)
                {
                    return !desire;
                }
            }
            return desire;
        }

        bool judge_all_achieve_state(bool desire)
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->achieve_desire != desire)
                {
                    return !desire;
                }
            }
            return desire;
        }

        // get rand number from 0 to rand_max
        int gen_rand(int rand_max)
        {
            return int(rand() / (double)RAND_MAX * rand_max);
        }
};

#endif