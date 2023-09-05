#ifndef CONTROLLERMAINWINDOW_H
#define CONTROLLERMAINWINDOW_H
#include <QMainWindow>
#include <QApplication>
#include <QDialog>
#include <QMessageBox>
#include <QLabel>
#include <QVector>
#include <QBrush>
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

#include <ros/ros.h>
#include <mavros_msgs/State.h>

#include <gui/controller/controller_infowindow.h>
#include <gui/controller/controller_modewindow.h>
#include <gui/controller/controller_takeoffwindow.h>
#include <gui/controller/controller_manualwindow.h>

#define PI 3.14159265358979323846
using namespace std;

class ControllerMainWindow : public QWidget
{
    public:
        QDialog *win = new QDialog();
        ControllerInfoWindow *info_win = new ControllerInfoWindow(win);
        ControllerModeWindow *mode_win = new ControllerModeWindow(win);
        ControllerTakeoffWindow *takeoff_win = new ControllerTakeoffWindow(win);
        ControllerManualWindow *manual_win = new ControllerManualWindow(win);
        QWidget *parent;
        ControllerMainWindow(QWidget *parent_widget, QStringList nodes_input)
        {
            qRegisterMetaType<QList<QPersistentModelIndex>>("QList<QPersistentModelIndex>");
            qRegisterMetaType<QList<QAbstractItemModel::LayoutChangeHint>>("QList<QAbstractItemModel::LayoutChangeHint>");
            qRegisterMetaType<QList<Qt::Orientation>>("QList<Qt::Orientation>");
            nodes = nodes_input;
            setup();
        }

    private:
        // settings
        string version = "V1.0.0";
        QString current_cmd = "None";
        double update_time = 0.3;

        // init vectors
        QVector<vehicle_command *> data;
        QVector<std::thread *> threads;
        QVector<std::thread *> ext_cmd_threads;
        QVector<px4_cmd::Command> cmds;
        QVector<px4_cmd::Command> ext_cmds;
        QVector<ros::Publisher> pubs;
        vector<vector<vector<double>>> cmd_values;
        bool thread_stop = false;
        bool land_return_operate = false;
        bool ext_cmd_state = false;
        QStringList nodes;
        QString operating_info;

        //Widgets
        QMessageBox *msg_box;
        QPushButton *mode_button;
        QPushButton *signal_button_1;
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
        QTableView *info_table = new QTableView(win);
        QStandardItemModel *info_model;
        QLabel *info_label_1;
        QLabel *info_label_2;
        QLabel *info_label_3;
        QLabel *state_label_1;
        QLabel *state_label_2;
        QLabel *state_label_3;

        void setup()
        {
            // set window
            QIcon *icon = new QIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
            win->setWindowIcon(*icon);
            win->setFixedSize(1280, 750);
            win->setWindowTitle(("PX4 Cmd Simulation Controller [Version: " + version + "]").c_str());
            win->setStyleSheet("background-color: rgb(255,250,250)");

            // get data
            for (auto item = nodes.begin(); item != nodes.end(); item++)
            {
                vehicle_command *vec = new vehicle_command();
                vec->start((*item).toStdString());
                data.push_back(vec);
            }

            // ros setting
            int argc = 0;
            char **argv;
            ros::init(argc, argv, "px4_cmd/px4_controller");
            ros::NodeHandle nh;
            for (size_t i = 0; i < nodes.size(); i++)
            {
                px4_cmd::Command cmd;
                if (data[i]->vehicle_name == "plane")
                {
                    cmd.Vehicle = px4_cmd::Command::FixWing;
                }
                else
                {
                    cmd.Vehicle = px4_cmd::Command::Multicopter;
                }
                cmd.desire_cmd[0] = data[i]->init_x;
                cmd.desire_cmd[1] = data[i]->init_y;
                cmd.desire_cmd[2] = data[i]->init_z;
                ros::Publisher pub = nh.advertise<px4_cmd::Command>((nodes[i] + "/px4_cmd/control_command").toStdString().c_str(), 50);
                pubs.push_back(pub);
                cmds.push_back(cmd);
                ext_cmds.push_back(cmd);
            }
            std::thread ros_thread(&ControllerMainWindow::ros_thread_func, this);
            ros_thread.detach();

            // layout
            QVBoxLayout *vbox = new QVBoxLayout();
            QHBoxLayout *hbox_top = new QHBoxLayout();
            QVBoxLayout *vbox_top_1 = new QVBoxLayout();
            QVBoxLayout *vbox_top_2 = new QVBoxLayout();
            QVBoxLayout *vbox_top_3 = new QVBoxLayout();
            QVBoxLayout *vbox_top_4 = new QVBoxLayout();
            QVBoxLayout *vbox_top_5 = new QVBoxLayout();
            QHBoxLayout *hbox_buttom = new QHBoxLayout();
            QVBoxLayout *vbox_buttom_1 = new QVBoxLayout();
            QVBoxLayout *vbox_buttom_2 = new QVBoxLayout();
            QHBoxLayout *hbox_buttom_1 = new QHBoxLayout();

            // add labels
            info_label_1 = new QLabel("[ROS State]  Running", win);
            info_label_1->setStyleSheet("color: green; font-size: 14pt");
            info_label_2 = new QLabel(("[Vehicle Count]  " + to_string(nodes.size())).c_str(), win);
            info_label_2->setStyleSheet("color: green; font-size: 14pt");
            info_label_3 = new QLabel(("[Current Mode]  " + data[0]->state_mode).c_str(), win);
            info_label_3->setStyleSheet("color: green; font-size: 14pt");
            state_label_1 = new QLabel("[Arm State]  DisArm", win);
            state_label_1->setStyleSheet("color: orange; font-size: 14pt");
            state_label_2 = new QLabel("[Current CMD]  None", win);
            state_label_2->setStyleSheet("color: green; font-size: 14pt");
            state_label_3 = new QLabel("", win);
            state_label_3->setStyleSheet("color: orange; font-size: 14pt");
            QLabel *label_1 = new QLabel("[Move CMD Mode]", win);
            label_1->setStyleSheet("color: rgb(0,169,190); font-size: 16pt; font-weight: bold");
            label_1->setAlignment(Qt::AlignmentFlag::AlignCenter);
            QLabel *label_2 = new QLabel("[State CMD Mode]", win);
            label_2->setStyleSheet("color: rgb(209,167,0); font-size: 16pt; font-weight: bold");
            label_2->setAlignment(Qt::AlignmentFlag::AlignCenter);
            info_label_3->setMinimumWidth(300);
            state_label_2->setMinimumWidth(300);

            // add top buttons
            QPushButton *split_line = new QPushButton("", win);
            split_line->setMaximumHeight(1);
            split_line->setFocusPolicy(Qt::NoFocus);
            split_line->setEnabled(false);
            mode_button = new QPushButton("Mode", win);
            arm_button = new QPushButton("Arm", win);
            takeoff_button = new QPushButton("Take Off", win);
            disarm_button = new QPushButton("DisArm", win);
            about_button = new QPushButton("About", win);
            exit_button = new QPushButton("Exit", win);
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
            manual_button = new QPushButton("Manual CMD", win);
            trajectory_button = new QPushButton("Trajectory CMD", win);
            external_button = new QPushButton("External CMD", win);
            hover_button = new QPushButton("Hover", win);
            land_button = new QPushButton("Land", win);
            return_button = new QPushButton("Return", win);
            generate_button = new QPushButton("Generate Template External CMD Code");
            signal_button_1 = new QPushButton("", win);
            signal_button_1->setVisible(false);
            manual_button->setMinimumHeight(60);
            trajectory_button->setMinimumHeight(60);
            external_button->setMinimumHeight(60);
            hover_button->setMinimumHeight(60);
            land_button->setMinimumHeight(60);
            return_button->setMinimumHeight(60);
            generate_button->setMinimumHeight(45);
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
            generate_button->setStyleSheet("background-color: rgb(233,181,177); font-size: 16pt");

            // add table
            QStringList table_headers_ext_cmd = {
                "Node",
                "CMD Mode",
                "Frame",
                "CMD 1",
                "CMD 2",
                "CMD 3",
                "CMD 4  [Yaw]",
                "External State",
                "External CMD Topic"
            };
            info_model = new QStandardItemModel(nodes.size(), table_headers_ext_cmd.size(), win);
            info_model->setHorizontalHeaderLabels(table_headers_ext_cmd);
            for (int i = 0; i < nodes.size(); i++)
            {
                QStandardItem *item_1 = new QStandardItem();
                QStandardItem *item_2 = new QStandardItem();
                QStandardItem *item_3 = new QStandardItem();
                QStandardItem *item_4 = new QStandardItem();
                QStandardItem *item_5 = new QStandardItem();
                QStandardItem *item_6 = new QStandardItem();
                QStandardItem *item_7 = new QStandardItem();
                QStandardItem *item_8 = new QStandardItem();
                QStandardItem *item_9 = new QStandardItem();
                item_1->setText("  " + nodes[i] + "  ");
                item_9->setText((nodes[i] + "/px4_cmd/external_command").toStdString().c_str());
                item_1->setEditable(false);
                item_2->setEditable(false);
                item_3->setEditable(false);
                item_4->setEditable(false);
                item_5->setEditable(false);
                item_6->setEditable(false);
                item_7->setEditable(false);
                item_8->setEditable(false);
                item_9->setEditable(false);
                item_1->setTextAlignment(Qt::AlignCenter);
                item_2->setTextAlignment(Qt::AlignCenter);
                item_3->setTextAlignment(Qt::AlignCenter);
                item_4->setTextAlignment(Qt::AlignCenter);
                item_5->setTextAlignment(Qt::AlignCenter);
                item_6->setTextAlignment(Qt::AlignCenter);
                item_7->setTextAlignment(Qt::AlignCenter);
                item_8->setTextAlignment(Qt::AlignCenter);
                item_9->setTextAlignment(Qt::AlignCenter);
                info_model->setItem(i, 0, item_1);
                info_model->setItem(i, 1, item_2);
                info_model->setItem(i, 2, item_3);
                info_model->setItem(i, 3, item_4);
                info_model->setItem(i, 4, item_5);
                info_model->setItem(i, 5, item_6);
                info_model->setItem(i, 6, item_7);
                info_model->setItem(i, 7, item_8);
                info_model->setItem(i, 8, item_9);
            }
            info_table->setModel(info_model);
            info_table->horizontalHeader()->setStretchLastSection(true);
            info_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
            info_table->setStyleSheet("background-color: rgb(236,245,225)");

            // set layout
            vbox->setContentsMargins(30, 20, 30, 20);
            vbox_top_1->setSpacing(15);
            vbox_top_2->setSpacing(15);
            vbox_top_3->setSpacing(15);
            vbox_top_4->setSpacing(15);
            vbox_top_5->setSpacing(15);
            vbox_buttom_1->setAlignment(Qt::AlignmentFlag::AlignTop);
            vbox_buttom_1->setSpacing(20);
            vbox_buttom_2->setSpacing(10);
            hbox_buttom->setSpacing(40);
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
            vbox_buttom_1->addWidget(label_1);
            vbox_buttom_1->addWidget(manual_button);
            vbox_buttom_1->addWidget(trajectory_button);
            vbox_buttom_1->addWidget(external_button);
            vbox_buttom_1->addSpacing(15);
            vbox_buttom_1->addWidget(label_2);
            vbox_buttom_1->addWidget(hover_button);
            vbox_buttom_1->addWidget(land_button);
            vbox_buttom_1->addWidget(return_button);
            hbox_buttom->addLayout(vbox_buttom_1, 1);
            vbox_buttom_2->addWidget(info_table);
            vbox_buttom_2->addWidget(generate_button);
            hbox_buttom->addLayout(vbox_buttom_2, 5);
            vbox->addLayout(hbox_buttom);
            win->setLayout(vbox);

            // connect
            QObject::connect(about_button, &QPushButton::clicked, this, &ControllerMainWindow::info_window_slot);
            QObject::connect(mode_button, &QPushButton::clicked, this, &ControllerMainWindow::mode_window_slot);
            QObject::connect(takeoff_button, &QPushButton::clicked, this, &ControllerMainWindow::takeoff_window_slot);
            QObject::connect(arm_button, &QPushButton::clicked, this, &ControllerMainWindow::arm_slot);
            QObject::connect(disarm_button, &QPushButton::clicked, this, &ControllerMainWindow::disarm_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &ControllerMainWindow::exit_slot);
            QObject::connect(hover_button, &QPushButton::clicked, this, &ControllerMainWindow::hover_slot);
            QObject::connect(land_button, &QPushButton::clicked, this, &ControllerMainWindow::land_slot);
            QObject::connect(return_button, &QPushButton::clicked, this, &ControllerMainWindow::return_slot);
            QObject::connect(manual_button, &QPushButton::clicked, this, &ControllerMainWindow::manual_cmd_slot);
            QObject::connect(external_button, &QPushButton::clicked, this, &ControllerMainWindow::ext_cmd_slot);
            QObject::connect(signal_button_1, &QPushButton::clicked, this, &ControllerMainWindow::ext_cmd_err_msg_slot);
            QObject::connect(mode_win, &ControllerModeWindow::change_mode_signal, this, &ControllerMainWindow::change_mode_slot);
            QObject::connect(takeoff_win, &ControllerTakeoffWindow::take_off_info_signal, this, &ControllerMainWindow::take_off_info_slot);

            // thread
            for (size_t i = 0; i < nodes.size(); i++)
            {
                std::thread update_table_thread(&ControllerMainWindow::update_table_info, this, i);
                update_table_thread.detach();
                threads.push_back(&update_table_thread);
            }
            std::thread update_thread(&ControllerMainWindow::update_info, this);
            update_thread.detach();
        }

        // ros thread
        void ros_thread_func()
        {
            while (ros::ok() && !thread_stop)
            {
                for (size_t i = 0; i < nodes.size(); i++)
                {
                    pubs[i].publish(cmds[i]);
                }
                ros::spinOnce();
                ros::Duration(0.02).sleep();
            }
        }

        // slot functions
        void info_window_slot()
        {
            info_win->win->exec();
        }

        void mode_window_slot()
        {
            mode_win->set_data(data);
            mode_win->win->exec();
        }

        void arm_slot()
        {
            operating_info = "Arming...";
            std::thread arm_thread(&ControllerMainWindow::arm_thread_func, this);
            arm_thread.detach();
        }

        void arm_thread_func()
        {
            string err = "";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                err = data[i]->set_mode("Arm");
                if (err != "")
                {
                    msg_box = new QMessageBox(win);
                    msg_box->setIcon(QMessageBox::Icon::Critical);
                    msg_box->setText("Error");
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
            arm_button->setEnabled(false);
            disarm_button->setEnabled(true);
        }

        void disarm_slot()
        {
            operating_info = "DisArming...";
            std::thread disarm_thread(&ControllerMainWindow::disarm_thread_func, this);
            disarm_thread.detach();
        }

        void disarm_thread_func()
        {
            string err = "";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                err = data[i]->set_mode("DisArm");
                if (err != "")
                {
                    msg_box = new QMessageBox(win);
                    msg_box->setIcon(QMessageBox::Icon::Critical);
                    msg_box->setText("Error");
                    msg_box->setWindowTitle("Error");
                    msg_box->setText(err.c_str());
                    msg_box->exec();
                    operating_info = "";
                    return;
                }
            }
            operating_info = "";
            current_cmd = "DisArm";
            arm_button->setEnabled(true);
            disarm_button->setEnabled(false);
        }

        void takeoff_window_slot()
        {
            if (!takeoff_win->set_data(data))
            {
                return;
            }
            takeoff_win->win->exec();
            takeoff_button->setEnabled(false);
            arm_button->setEnabled(false);
            mode_button->setEnabled(false);
            if (takeoff_win->set_height)
            {
                std::thread take_off_thread(&ControllerMainWindow::take_off_thread_func, this);
                take_off_thread.detach();
            }
        }

        void take_off_thread_func()
        {
            string err = "";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                cmds[i].Mode = px4_cmd::Command::Takeoff;
                cmds[i].Move_frame = px4_cmd::Command::ENU;
                cmds[i].desire_cmd[0] = data[i]->init_x;
                cmds[i].desire_cmd[1] = data[i]->init_y;
                cmds[i].desire_cmd[2] = takeoff_win->takeoff_height;
                cmds[i].yaw_cmd = 0;
                err = data[i]->set_mode("Arm");
                if (err != "")
                {
                    msg_box = new QMessageBox(win);
                    msg_box->setIcon(QMessageBox::Icon::Critical);
                    msg_box->setText("Error");
                    msg_box->setWindowTitle("Error");
                    msg_box->setText("Armed Failure, Please Retry.");
                    msg_box->exec();
                    for (size_t j = 0; j < nodes.size(); j++)
                    {
                        data[i]->set_mode(mavros_msgs::State::MODE_PX4_RTL);
                    }
                    operating_info = "";
                    mode_button->setEnabled(true);
                    takeoff_button->setEnabled(true);
                    arm_button->setEnabled(true);
                    return;
                }
            }
            operating_info = "";
            current_cmd = "Take Off";
            mode_button->setEnabled(true);
            takeoff_button->setEnabled(false);
            arm_button->setEnabled(false);
            disarm_button->setEnabled(true);
            manual_button->setEnabled(true);
            trajectory_button->setEnabled(true);
            external_button->setEnabled(true);
            hover_button->setEnabled(true);
            land_button->setEnabled(true);
            return_button->setEnabled(true);
        }

        void ext_cmd_slot()
        {
            if (!ext_cmd_state)
            {
                msg_box = new QMessageBox(win);
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setText("Error");
                msg_box->setWindowTitle("Error");
                msg_box->setText("Not Detect External Command Topic, Please Check.");
                msg_box->exec();
                return;
            }
            current_cmd = "External Command";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                std::thread ext_cmd_thread(&ControllerMainWindow::ext_cmd_thread_func, this, i);
                ext_cmd_threads.push_back(&ext_cmd_thread);
            }
            for (size_t i = 0; i < nodes.size(); i++)
            {
                ext_cmd_threads[i]->detach();
            }
        }

        void ext_cmd_thread_func(int node_id)
        {
            while (ext_cmd_state && (current_cmd == "External Command"))
            {
                cmds[node_id] = data[node_id]->ext_cmd;
                if (cmds[node_id].Move_mode == px4_cmd::Command::XYZ_POS)
                {
                    cmds[node_id].desire_cmd[0] = cmds[node_id].desire_cmd[0] - data[node_id]->init_x;
                    cmds[node_id].desire_cmd[1] = cmds[node_id].desire_cmd[1] - data[node_id]->init_y;
                    cmds[node_id].desire_cmd[2] = cmds[node_id].desire_cmd[2] - data[node_id]->init_z;
                }
                if (cmds[node_id].Move_mode == px4_cmd::Command::XY_VEL_Z_POS)
                {
                    cmds[node_id].desire_cmd[2] = cmds[node_id].desire_cmd[2] - data[node_id]->init_z;
                }
                cmds[node_id].yaw_cmd = manual_win->cmd_values[0][node_id][3];
            }
            if (!ext_cmd_state)
            {
                cmds[node_id].Mode = px4_cmd::Command::Hover;
            }
        }

        void hover_slot()
        {
            current_cmd = "Hover";
            std::thread hover_thread(&ControllerMainWindow::hover_thread_func, this);
            hover_thread.detach();
        }

        void hover_thread_func()
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
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
            while (!stop)
            {
                tmp = 0;
                for (auto item = threads.begin(); item != threads.end(); item++)
                {
                    if ((*item)->joinable())
                    {
                        continue;
                    }
                    tmp++;
                }
                for (auto item = ext_cmd_threads.begin(); item != ext_cmd_threads.end(); item++)
                {
                    if ((*item)->joinable())
                    {
                        continue;
                    }
                    tmp++;
                }
                for (auto item = data.begin(); item != data.end(); item++)
                {
                    if ((*item)->ros_stop)
                    {
                        continue;
                    }
                    tmp++;
                }
                if (tmp == 0)
                {
                    stop = true;
                }
            }
            win->close();
        }

        void take_off_info_slot()
        {
            operating_info = "Take off and Arming...";
        }

        void manual_cmd_slot()
        {
            manual_win->set_nodes(nodes);
            manual_win->win->exec();
            if (manual_win->exec_state)
            {
                current_cmd = "Manual CMD";
                cmd_values = manual_win->cmd_values;
                std::thread manual_cmd_thread(&ControllerMainWindow::manual_cmd_thread_func, this);
                manual_cmd_thread.detach();
            }
        }

        void manual_cmd_thread_func()
        {
            for (size_t i = 0; i < cmds.size(); i++)
            {
                cmds[i].Mode = px4_cmd::Command::Move;
                cmds[i].Move_mode = manual_win->set_mode;
                cmds[i].Move_frame = manual_win->set_frame;
                cmds[i].desire_cmd[0] = manual_win->cmd_values[0][i][0];
                cmds[i].desire_cmd[1] = manual_win->cmd_values[0][i][1];
                cmds[i].desire_cmd[2] = manual_win->cmd_values[0][i][2];
                if (cmds[i].Move_mode == px4_cmd::Command::XYZ_POS)
                {
                    cmds[i].desire_cmd[0] = cmds[i].desire_cmd[0] - data[i]->init_x;
                    cmds[i].desire_cmd[1] = cmds[i].desire_cmd[1] - data[i]->init_y;
                    cmds[i].desire_cmd[2] = cmds[i].desire_cmd[2] - data[i]->init_z;
                }
                if (cmds[i].Move_mode == px4_cmd::Command::XY_VEL_Z_POS)
                {
                    cmds[i].desire_cmd[2] = cmds[i].desire_cmd[2] - data[i]->init_z;
                }
                cmds[i].yaw_cmd = manual_win->cmd_values[0][i][3];
            }
        }

        void ext_cmd_err_msg_slot()
        {
            msg_box = new QMessageBox(win);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setText("Error");
            msg_box->setWindowTitle("Error");
            msg_box->setText("Disconnection to External Command Topic, Please Check. Automatically Change to Hover Mode.");
            msg_box->exec();
        }

        void update_info()
        {
            bool pre_arm_state = false;
            bool arm_state = false;
            bool ext_cmd_state_single = false;
            QString pre_cmd = "None";
            string current_mode = "";
            string pre_mode = "";
            while (!thread_stop)
            {
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
                arm_state = data[0]->arm_state;
                current_mode = data[0]->state_mode;
                if (arm_state != pre_arm_state || current_mode != pre_mode)
                {
                    if (arm_state)
                    {
                        state_label_1->setText("[Arm State]  Arm");
                        state_label_1->setStyleSheet("color: green; font-size: 14pt");
                        arm_button->setEnabled(false);
                        disarm_button->setEnabled(true);
                        if (!data[0]->land_state)
                        {
                            mode_button->setEnabled(false);
                            manual_button->setEnabled(true);
                            trajectory_button->setEnabled(true);
                            external_button->setEnabled(true);
                            hover_button->setEnabled(true);
                            land_button->setEnabled(true);
                            return_button->setEnabled(true);
                        }
                        if ((current_mode == mavros_msgs::State::MODE_PX4_LAND) && (!data[0]->land_state))
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
                        if ((current_mode == mavros_msgs::State::MODE_PX4_RTL) && (!data[0]->land_state))
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
                    if (data[i]->ext_cmd_state)
                    {
                        ext_cmd_state_single = true;
                        continue;
                    }
                    ext_cmd_state_single = false;
                    break;
                }
                ext_cmd_state = ext_cmd_state_single;

                //sleep
                ros::Duration(0.1).sleep();
            }
        }

        void update_table_info(int node_id)
        {
            bool ext_cmd_state_single = false;
            bool pre_ext_cmd_state_single = false;
            QString cmd_mode = "Loiter";
            QString pre_cmd = "";
            QString frame = "";
            string current_mode = "";
            string pre_mode = "";
            QString null_str = "  -----------  ";
            QStringList table_headers_ext_cmd = {
                "Node",
                "CMD Mode",
                "Frame",
                "CMD 1",
                "CMD 2",
                "CMD 3",
                "CMD 4  [Yaw]",
                "External State",
                "External CMD Topic"
            };
            QStandardItem *item_2 = new QStandardItem();
            QStandardItem *item_3 = new QStandardItem();
            QStandardItem *item_4 = new QStandardItem();
            QStandardItem *item_5 = new QStandardItem();
            QStandardItem *item_6 = new QStandardItem();
            QStandardItem *item_7 = new QStandardItem();
            QStandardItem *item_8 = new QStandardItem();
            item_2->setEditable(false);
            item_3->setEditable(false);
            item_4->setEditable(false);
            item_5->setEditable(false);
            item_6->setEditable(false);
            item_7->setEditable(false);
            item_8->setEditable(false);
            item_2->setTextAlignment(Qt::AlignCenter);
            item_3->setTextAlignment(Qt::AlignCenter);
            item_4->setTextAlignment(Qt::AlignCenter);
            item_5->setTextAlignment(Qt::AlignCenter);
            item_6->setTextAlignment(Qt::AlignCenter);
            item_7->setTextAlignment(Qt::AlignCenter);
            item_8->setTextAlignment(Qt::AlignCenter);
            item_8->setText("Deactive");
            while (!thread_stop)
            {
                // current mode
                current_mode = data[0]->state_mode;
                if (pre_mode != current_mode || pre_cmd != current_cmd)
                {
                    pre_mode = current_mode;
                    pre_cmd = current_cmd;
                    if (current_mode != mavros_msgs::State::MODE_PX4_OFFBOARD)
                    {
                        item_3->setText(null_str);
                        item_4->setText(null_str);
                        item_5->setText(null_str);
                        item_6->setText(null_str);
                        item_7->setText(null_str);
                        table_headers_ext_cmd[3] = "CMD 1";
                        table_headers_ext_cmd[4] = "CMD 2";
                        table_headers_ext_cmd[5] = "CMD 3";
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
                ext_cmd_state_single = data[0]->ext_cmd_state;
                if (pre_ext_cmd_state_single != ext_cmd_state_single)
                {
                    if (ext_cmd_state)
                    {
                        item_8->setText("  Active  ");
                    }
                    else
                    {
                        item_8->setText("  Deactive  ");
                    }
                }

                // table info
                if (current_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    // update headers
                    // update cmd mode info
                    if (current_mode == "Take Off")
                    {
                        cmd_mode = "Take Off";
                        table_headers_ext_cmd[3] = "CMD 1  [x]";
                        table_headers_ext_cmd[4] = "CMD 2  [y]";
                        table_headers_ext_cmd[5] = "CMD 3  [z]";
                    }
                    else
                    {
                        switch (cmds[0].Move_mode)
                        {
                            case px4_cmd::Command::XYZ_POS:
                                cmd_mode = "Postion";
                                table_headers_ext_cmd[3] = "CMD 1  [x]";
                                table_headers_ext_cmd[4] = "CMD 2  [y]";
                                table_headers_ext_cmd[5] = "CMD 3  [z]";
                                break;

                            case px4_cmd::Command::XYZ_REL_POS:
                                cmd_mode = "Rel Position";
                                table_headers_ext_cmd[3] = "CMD 1  [Rel x]";
                                table_headers_ext_cmd[4] = "CMD 2  [Rel y]";
                                table_headers_ext_cmd[5] = "CMD 3  [Rel z]";
                                break;

                            case px4_cmd::Command::XYZ_VEL:
                                cmd_mode = "Velocity";
                                table_headers_ext_cmd[3] = "CMD 1  [vx]";
                                table_headers_ext_cmd[4] = "CMD 2  [vy]";
                                table_headers_ext_cmd[5] = "CMD 3  [vz]";
                                break;

                            case px4_cmd::Command::XY_VEL_Z_POS:
                                cmd_mode = "Velocity with Altitude";
                                table_headers_ext_cmd[3] = "CMD 1  [vx]";
                                table_headers_ext_cmd[4] = "CMD 2  [vy]";
                                table_headers_ext_cmd[5] = "CMD 3  [z]";
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
                    item_2->setText("  " + cmd_mode + "  ");
                    item_3->setText("  " + frame + "  ");
                }

                if (current_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    if (cmds[node_id].Move_mode == px4_cmd::Command::XYZ_POS && (current_cmd == "Manual Command" || current_cmd == "Trajectory Command" || current_cmd == "External Command"))
                    {
                        item_4->setText(("  " + to_string(cmds[node_id].desire_cmd[0] + data[node_id]->init_x) + "  ").c_str());
                        item_5->setText(("  " + to_string(cmds[node_id].desire_cmd[1] + data[node_id]->init_y) + "  ").c_str());
                        item_6->setText(("  " + to_string(cmds[node_id].desire_cmd[2] + data[node_id]->init_z) + "  ").c_str());
                    }
                    else
                    {
                        if (cmds[node_id].Move_mode == px4_cmd::Command::XY_VEL_Z_POS && (current_cmd == "Manual Command" || current_cmd == "Trajectory Command" || current_cmd == "External Command"))
                        {
                            item_4->setText(("  " + to_string(cmds[node_id].desire_cmd[0]) + "  ").c_str());
                            item_5->setText(("  " + to_string(cmds[node_id].desire_cmd[1]) + "  ").c_str());
                            item_6->setText(("  " + to_string(cmds[node_id].desire_cmd[2] + data[node_id]->init_z) + "  ").c_str());
                        }
                        else
                        {
                            item_4->setText(("  " + to_string(cmds[node_id].desire_cmd[0]) + "  ").c_str());
                            item_5->setText(("  " + to_string(cmds[node_id].desire_cmd[1]) + "  ").c_str());
                            item_6->setText(("  " + to_string(cmds[node_id].desire_cmd[2]) + "  ").c_str());
                        }
                    }
                    item_7->setText(("  " + to_string(cmds[node_id].yaw_cmd) + "  ").c_str());
                }
                info_model->setItem(node_id, 1, item_2);
                info_model->setItem(node_id, 2, item_3);
                info_model->setItem(node_id, 3, item_4);
                info_model->setItem(node_id, 4, item_5);
                info_model->setItem(node_id, 5, item_6);
                info_model->setItem(node_id, 6, item_7);
                info_model->setItem(node_id, 7, item_8);
                info_model->setHorizontalHeaderLabels(table_headers_ext_cmd);
                if (ext_cmd_state)
                {
                    info_model->setData(info_model->index(node_id, 7), QBrush(Qt::green), Qt::TextColorRole);
                }
                else
                {
                    info_model->setData(info_model->index(node_id, 7), QBrush(Qt::red), Qt::TextColorRole);
                }
                // sleep
                ros::Duration(0.2).sleep();
            }
        }

        // subsribe function
        void ext_cmd_sub_func(px4_cmd::Command::ConstPtr &msg, int i)
        {
            ext_cmds[i] = *msg;
        }
};
#endif