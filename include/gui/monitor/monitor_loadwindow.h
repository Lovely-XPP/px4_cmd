#ifndef MONITORLOADWINDOW_H
#define MONITORLOADWINDOW_H
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
#include <print_utility/printf_utility.h>

using namespace std;

class MonitorLoadWindow : public QWidget
{
    public:
        bool push_button = false;
        QStringList nodes;
        QDialog *win = new QDialog();
        QWidget *parent;
        MonitorLoadWindow(QWidget *parent_widget)
        {
            setup();
        }

    private:
        // settings
        string version = "V1.0.2";
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
            win->setWindowIcon(*icon);
            win->setFixedSize(1080, 130);
            win->setWindowTitle(("PX4 Cmd Simulation Monitor [Version: " + version + "]").c_str());
            win->setStyleSheet("background-color: rgb(255,250,250)");
            
            // detect env
            string err = detect_env();
            if (err.length() != 0)
            {
                msg_box = new QMessageBox(win);
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setWindowTitle("Error");
                msg_box->setText(err.c_str());
                exit(msg_box->exec());
            }

            // lables
            label_1 = new QLabel("", win);
            label_2 = new QLabel("", win);
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
            start_button = new QPushButton("Start  Monitor", win);
            start_button->setStyleSheet("background-color: rgb(84,255,159); font-weight: bold; font-size: 16pt");
            start_button->setMinimumHeight(70);
            start_button->setEnabled(false);
            exit_button = new QPushButton("Exit", win);
            exit_button->setStyleSheet("background-color: rgb(255,99,71); font-weight: bold; font-size: 16pt");
            exit_button->setMinimumHeight(70);
            hbox->addWidget(start_button, 2);
            hbox->addSpacing(20);
            hbox->addWidget(exit_button, 2);

            // layout
            win->setLayout(hbox);

            // connect signal and slot
            QObject::connect(start_button, &QPushButton::clicked, this, &MonitorLoadWindow::start_button_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &MonitorLoadWindow::exit_button_slot);

            // thread for detect ros
            std::thread pub_thread(&MonitorLoadWindow::detect_ros, this);
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
            get_cmd_output("which rospack", res);
            strip(res, "\n");
            if (res.length() == 0)
            {
                error_msg = error_header + "Please Check ROS Installation & rospack command.";
                return error_msg;
            }
            res.clear();
            get_cmd_output("rospack list-names", res);
            strip(res, "\n");
            if (res.find("px4") == res.npos)
            {
                error_msg = error_header + "Please Check PX4 ROS Package Installation.";
                return error_msg;
            }
            if (res.find("mavlink_sitl_gazebo") == res.npos)
            {
                error_msg = error_header + "Please Check mavlink_sitl_gazebo ROS Package Installation.";
                return error_msg;
            }
            if (res.find("px4_cmd") == res.npos)
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
                            label_2->setText("Please Click [Start Monitor] Button to Continue...");
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
            win->close();
        }

        bool detect_px4()
        {
            string tmp;
            QString node;
            QStringList nodes_tmp;
            get_cmd_output("rosnode list | grep 'mavros'", tmp);
            node = tmp.c_str();
            nodes_tmp = node.split("/mavros\n");
            for (auto item = nodes_tmp.begin(); item != nodes_tmp.end(); item++)
            {
                (*item).remove(0, 1);
            }
            if (tmp.size() > 0)
            {
                if (nodes.size() == 0)
                {
                    nodes_tmp.removeLast();
                    nodes = nodes_tmp;
                }
                return true;
            }
            return false;
        }
};

#endif