#ifndef CONTROLLERMAINWINDOW_H
#define CONTROLLERMAINWINDOW_H
#include <QMainWindow>
#include <QApplication>
#include <QDialog>
#include <QMessageBox>
#include <QLabel>
#include <QVector>
#include <QTableView>
#include <QPushButton>
#include <QHeaderView>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QString>
#include <QStringList>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <thread>

#include <ros/ros.h>

#include <gui/controller/controller_infowindow.h>
#include <gui/controller/controller_modewindow.h>
#include <gui/controller/controller_takeoffwindow.h>
#include <print_utility/printf_utility.h>
#include <vehicle_command.h>


using namespace std;

class ControllerMainWindow : public QWidget
{
    public:
        QDialog *win = new QDialog();
        ControllerInfoWindow *info_win = new ControllerInfoWindow(win);
        ControllerModeWindow *mode_win = new ControllerModeWindow(win);
        ControllerTakeoffWindow *takeoff_win = new ControllerTakeoffWindow(win);
        QWidget *parent;
        ControllerMainWindow(QWidget *parent_widget, QStringList nodes_input)
        {
            nodes = nodes_input;
            setup();
        }

    private:
        // settings
        string version = "V1.0.0";
        QString current_cmd = "None";
        double update_time = 0.3;
        vector<string> table_headers_ext_cmd = {"Node", "CMD Mode", "CMD 1", "CMD 2", "CMD 3", "External CMD Topic", "External State"};

        // init vectors
        QVector<vehicle_command *> data;
        QVector<std::thread *> threads;
        QVector<px4_cmd::Command> cmds;
        QVector<ros::Publisher> pubs;
        bool thread_stop = false;
        QStringList nodes;

        //Widgets
        QMessageBox *msg_box;
        QPushButton *mode_button;
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
                ros::Publisher pub = nh.advertise<px4_cmd::Command>((nodes[i] + "/px4_cmd/control_command").toStdString().c_str(), 50);
                pubs.push_back(pub);
            }
            for (size_t i = 0; i < data.size(); i++)
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
                cmds.push_back(cmd);
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
            QLabel *info_label_1 = new QLabel("[ROS State]  Running", win);
            info_label_1->setStyleSheet("color: green; font-size: 16pt");
            QLabel *info_label_2 = new QLabel(("[Vehicle Count]  " + to_string(nodes.size())).c_str(), win);
            info_label_2->setStyleSheet("color: green; font-size: 16pt");
            QLabel *state_label_1 = new QLabel("[Arm State]  DisArm", win);
            state_label_1->setStyleSheet("color: orange; font-size: 16pt");
            QLabel *state_label_2 = new QLabel("[Current CMD]  None", win);
            state_label_2->setStyleSheet("color: green; font-size: 16pt");
            QLabel *label_1 = new QLabel("[Move CMD Mode]", win);
            label_1->setStyleSheet("color: rgb(0,169,190); font-size: 16pt; font-weight: bold");
            label_1->setAlignment(Qt::AlignmentFlag::AlignCenter);
            QLabel *label_2 = new QLabel("[State CMD Mode]", win);
            label_2->setStyleSheet("color: rgb(209,167,0); font-size: 16pt; font-weight: bold");
            label_2->setAlignment(Qt::AlignmentFlag::AlignCenter);

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

            // add buttom table
            QStringList list_ext_cmd;
            info_model = new QStandardItemModel(nodes.size(), table_headers_ext_cmd.size(), win);
            int ext_cmd_count = 0;
            for (auto item = table_headers_ext_cmd.begin(); item != table_headers_ext_cmd.end(); item++)
            {
                list_ext_cmd.append(&(*item->c_str()));
            }
            info_model->setHorizontalHeaderLabels(list_ext_cmd);
            for (auto node_item = nodes.begin(); node_item != nodes.end(); node_item++)
            {
                for (auto item = table_headers_ext_cmd.begin(); item != table_headers_ext_cmd.end(); item++)
                {
                    QStandardItem *item_1 = new QStandardItem();
                    QStandardItem *item_2 = new QStandardItem();
                    QStandardItem *item_3 = new QStandardItem();
                    QStandardItem *item_4 = new QStandardItem();
                    QStandardItem *item_5 = new QStandardItem();
                    QStandardItem *item_6 = new QStandardItem();
                    QStandardItem *item_7 = new QStandardItem();
                    item_1->setEditable(false);
                    item_2->setEditable(false);
                    item_3->setEditable(false);
                    item_4->setEditable(false);
                    item_5->setEditable(false);
                    item_6->setEditable(false);
                    item_7->setEditable(false);
                    item_1->setTextAlignment(Qt::AlignCenter);
                    item_2->setTextAlignment(Qt::AlignCenter);
                    item_3->setTextAlignment(Qt::AlignCenter);
                    item_4->setTextAlignment(Qt::AlignCenter);
                    item_5->setTextAlignment(Qt::AlignCenter);
                    item_6->setTextAlignment(Qt::AlignCenter);
                    item_7->setTextAlignment(Qt::AlignCenter);
                    info_model->setItem(ext_cmd_count, 0, item_1);
                    info_model->setItem(ext_cmd_count, 1, item_2);
                    info_model->setItem(ext_cmd_count, 2, item_3);
                    info_model->setItem(ext_cmd_count, 3, item_4);
                    info_model->setItem(ext_cmd_count, 4, item_5);
                    info_model->setItem(ext_cmd_count, 5, item_6);
                    info_model->setItem(ext_cmd_count, 6, item_7);
                    ext_cmd_count++;
                }
            }
            info_table->setModel(info_model);
            info_table->horizontalHeader()->setStretchLastSection(true);
            info_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
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
            hbox_top->addLayout(vbox_top_1);
            vbox_top_2->addWidget(state_label_1);
            vbox_top_2->addWidget(state_label_2);
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
            hbox_buttom->addLayout(vbox_buttom_2, 4);
            vbox->addLayout(hbox_buttom);
            win->setLayout(vbox);

            // connect
            QObject::connect(about_button, &QPushButton::clicked, this, &ControllerMainWindow::info_window_slot);
            QObject::connect(mode_button, &QPushButton::clicked, this, &ControllerMainWindow::mode_window_slot);
            QObject::connect(takeoff_button, &QPushButton::clicked, this, &ControllerMainWindow::takeoff_window_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &ControllerMainWindow::exit_slot);
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

        void takeoff_window_slot()
        {
            if (!takeoff_win->set_data(data))
            {
                return;
            }
            takeoff_win->win->exec();
            if (takeoff_win->set_height)
            {
                for (size_t i = 0; i < nodes.size(); i++)
                {
                    cmds[i].Mode = px4_cmd::Command::Move;
                    cmds[i].Move_frame = px4_cmd::Command::ENU;
                    if (data[i]->vehicle_name == "plane")
                    {
                        cmds[i].Move_mode = px4_cmd::Command::FixWing_POS;
                    }
                    else
                    {
                        cmds[i].Move_mode = px4_cmd::Command::XYZ_POS;
                    }
                    cmds[i].desire_cmd[0] = data[i]->init_x;
                    cmds[i].desire_cmd[1] = data[i]->init_y;
                    cmds[i].desire_cmd[2] = data[i]->init_z + takeoff_win->takeoff_height;
                    cmds[i].yaw_cmd = 0;
                    data[i]->set_mode("Arm");
                }
            }
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
                /*
                for (auto item = threads.begin(); item != threads.end(); item++)
                {
                    if ((*item)->joinable())
                    {
                        continue;
                    }
                    tmp++;
                }
                */
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
};
#endif