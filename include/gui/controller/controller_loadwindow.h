// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef CONTROLLER_LOADWINDOW_H
#define CONTROLLER_LOADWINDOW_H
#include <QApplication>
#include <QStyle>
#include <QDialog>
#include <QMessageBox>
#include <QLabel>
#include <QIcon>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>
#include <QStringList>
#include <thread>

#include <ros/ros.h>
#include <ros/master.h>
#include <ros/package.h>

using namespace std;

class ControllerLoadWindow : public QDialog
{
    public:
        bool push_button = false;
        QStringList nodes;
        ControllerLoadWindow(QWidget *parent_widget = 0)
        {
            setup();
        }

        ~ControllerLoadWindow()
        {
            thread_stop = true;
        }

    private:
        // settings
        string version = "V2.0.0";
        bool thread_stop = false;

        //Widgets
        QMessageBox *msg_box;
        QHBoxLayout *hbox = new QHBoxLayout();
        QVBoxLayout *vbox = new QVBoxLayout();
        QPushButton *start_button;
        QPushButton *exit_button;
        QLabel *label_1;
        QLabel *label_2;

        void setup()
        {
            // init window
            QIcon *icon = new QIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
            this->setWindowIcon(*icon);
            this->setFixedSize(1080, 130);
            this->setWindowTitle(("PX4 Cmd Simulation Controller [Version: " + version + "]").c_str());
            this->setStyleSheet("background-color: rgb(255,250,250)");
            
            // detect env
            string err = detect_env();
            if (err.length() != 0)
            {
                msg_box = new QMessageBox(this);
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setWindowTitle("Error");
                msg_box->setText(err.c_str());
                exit(msg_box->exec());
            }

            // lables
            label_1 = new QLabel("", this);
            label_2 = new QLabel("", this);
            label_1->setText("Can not Detected ROS Running!");
            label_1->setStyleSheet("color: red; font-size: 11pt");
            label_2->setText("Please Run roslaunch xxx.launch to Continue...");
            label_2->setStyleSheet("color: red; font-size: 11pt");
            vbox->addWidget(label_1);
            vbox->addWidget(label_2);
            vbox->setAlignment(Qt::AlignVCenter);
            vbox->setSpacing(20);
            hbox->addLayout(vbox, 3);

            // start_button
            start_button = new QPushButton("Start  Controller", this);
            start_button->setStyleSheet("background-color: rgb(84,255,159); font-weight: bold; font-size: 16pt");
            start_button->setMinimumHeight(70);
            start_button->setEnabled(false);
            exit_button = new QPushButton("Exit", this);
            exit_button->setStyleSheet("background-color: rgb(255,99,71); font-weight: bold; font-size: 16pt");
            exit_button->setMinimumHeight(70);
            hbox->addWidget(start_button, 2);
            hbox->addSpacing(20);
            hbox->addWidget(exit_button, 2);

            // layout
            this->setLayout(hbox);

            // connect signal and slot
            QObject::connect(start_button, &QPushButton::clicked, this, &ControllerLoadWindow::start_button_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &ControllerLoadWindow::exit_button_slot);

            // thread for detect ros
            std::thread pub_thread(&ControllerLoadWindow::detect_ros, this);
            pub_thread.detach();
        }

        // slot functions
        void start_button_slot()
        {
            push_button = true;
        }
        void exit_button_slot()
        {
            thread_stop = true;
        }

        // detect environment
        string detect_env()
        {
            string error_msg = "";
            string error_header = "This Programme requires ROS enviroment and PX4 installation and px4_cmd ROS Package.\n";
            string res = "";
            res = ros::package::getPath("px4");
            if (res.size() < 1)
            {
                error_msg = error_header + "Please Check PX4 ROS Package Installation.";
                return error_msg;
            }
            res = ros::package::getPath("mavlink_sitl_gazebo");
            if (res.size() < 1)
            {
                error_msg = error_header + "Please Check mavlink_sitl_gazebo ROS Package Installation.";
                return error_msg;
            }
            res = ros::package::getPath("px4_cmd");
            if (res.size() < 1)
            {
                error_msg = error_header + "Please Check px4_cmd ROS Package Installation.\nGithub: https://github.com/Lovely-XPP/PX4_cmd/";
                return error_msg;
            }
            return error_msg;
        }

        // thread update
        void detect_ros()
        {
            ros::Time::init();
            bool ros_state = !ros::master::check();
            bool px4_state = false;
            QString vehicle_nodes = "";
            while ((!px4_state || !push_button) && !thread_stop)
            {
                if (ros_state != ros::master::check())
                {
                    ros_state = ros::master::check();
                    if (!ros_state)
                    {
                        label_1->setText("Not Detecte ROS Running!");
                        label_1->setStyleSheet("color: red; font-size: 11pt");
                        label_2->setText("Please Run roslaunch xxx.launch to Continue...");
                        label_2->setStyleSheet("color: red; font-size: 11pt");
                        start_button->setEnabled(false);
                    }
                    else
                    {
                        sleep(1);
                    }
                }
                if (ros_state)
                {
                    if (px4_state != detect_px4())
                    {
                        px4_state = detect_px4();
                        if (px4_state)
                        {
                            label_1->setText(("Detecte ROS & PX4 Running!\nDetected Nodes Count: " + to_string(nodes.size())).c_str());
                            label_1->setStyleSheet("color: green; font-size: 11pt");
                            label_2->setText("Please Click [Start Controller] Button to Continue...");
                            label_2->setStyleSheet("color: green; font-size: 11pt");
                            start_button->setEnabled(true);
                        }
                        else
                        {
                            label_1->setText("Detecte ROS Running, but Not Detected PX4 Running!");
                            label_1->setStyleSheet("color: orange; font-size: 11pt");
                            label_2->setText("Please Check Launch File and Restart ROS to Continue...");
                            label_2->setStyleSheet("color: orange; font-size: 11pt");
                            start_button->setEnabled(false);
                        }
                    }
                }
                usleep(500000);
            }
            this->close();
        }

        bool detect_px4()
        {
            nodes.clear();
            string tmp;
            ros::V_string ros_nodes;
            ros::master::getNodes(ros_nodes);
            for (auto ros_node : ros_nodes)
            {
                tmp = ros_node;
                if (tmp.find("/mavros") == std::string::npos)
                {
                    continue;
                }
                tmp.erase(tmp.find("/mavros"), 7);
                if (tmp.size() > 0)
                {
                    tmp.erase(0, 1);
                }
                nodes.append(QString::fromStdString(tmp));
            }
            if (nodes.size() > 0)
            {
                return true;
            }
            return false;
        }
};

#endif