// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef CONTROLLERGENERATEWINDOW_H
#define CONTROLLERGENERATEWINDOW_H
#include <QDialog>
#include <QPushButton>
#include <QCheckBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QStringList>
#include <QMessageBox>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <print_utility/printf_utility.h>

using namespace std;

class ControllerGenerateWindow : public QDialog
{
    public:
        ControllerGenerateWindow(QWidget *parent_widget = 0)
        {
            setup();
        }
        void set_nodes(QStringList input)
        {
            nodes = input;
        }

    private:
        // init widget
        QMessageBox *msg_box;
        QPushButton *py_button;
        QPushButton *cpp_button;
        QCheckBox *pos_box;
        QCheckBox *vel_box;
        QCheckBox *pose_box;
        QCheckBox *angle_rate_box;
        QCheckBox *mode_box;
        QCheckBox *ex_mode_box;

        // init nodes
        QStringList nodes;

        void setup()
        {
            // set this
            this->setFixedSize(680, 220);
            this->setWindowTitle("Take Off Setting");
            this->setStyleSheet("background-color: rgb(255,250,250)");

            // init layouts
            QVBoxLayout *vbox = new QVBoxLayout();
            QHBoxLayout *hbox_button = new QHBoxLayout();
            QHBoxLayout *hbox_cb = new QHBoxLayout();
            QVBoxLayout *vbox_cb_1 = new QVBoxLayout();
            QVBoxLayout *vbox_cb_2 = new QVBoxLayout();
            QVBoxLayout *vbox_cb_3 = new QVBoxLayout();
            QGroupBox *group = new QGroupBox("[Select Parameters]", this);
            group->setAlignment(Qt::AlignHCenter | Qt::AlignBottom);
            group->setStyleSheet("font-size: 16pt;");

            // add buttons
            py_button = new QPushButton("Generate Python Code", this);
            py_button->setStyleSheet("font-size: 16pt");
            cpp_button = new QPushButton("Generate C++ Code", this);
            cpp_button->setStyleSheet("font-size: 16pt");
            py_button->setMinimumHeight(50);
            cpp_button->setMinimumHeight(50);

            // add checkboxes
            pos_box = new QCheckBox("Position", this);
            pos_box->setStyleSheet("font-size: 14pt");
            vel_box = new QCheckBox("Velocity", this);
            vel_box->setStyleSheet("font-size: 14pt");
            pose_box = new QCheckBox("Attitude", this);
            pose_box->setStyleSheet("font-size: 14pt");
            angle_rate_box = new QCheckBox("Angle Rate", this);
            angle_rate_box->setStyleSheet("font-size: 14pt");
            pos_box->setChecked(true);

            // set layouts
            vbox_cb_1->addWidget(pos_box);
            vbox_cb_1->addWidget(vel_box);
            hbox_cb->addLayout(vbox_cb_1);
            vbox_cb_2->addWidget(pose_box);
            vbox_cb_2->addWidget(angle_rate_box);
            hbox_cb->addLayout(vbox_cb_2);
            hbox_cb->addLayout(vbox_cb_3);
            group->setLayout(hbox_cb);
            vbox->addWidget(group);
            vbox->addSpacing(10);
            hbox_button->addWidget(py_button, 1);
            hbox_button->addStretch(1);
            hbox_button->addWidget(cpp_button, 1);
            vbox->addLayout(hbox_button);
            this->setLayout(vbox);

            // connect signals and slots
            QObject::connect(cpp_button, &QPushButton::clicked, this, &ControllerGenerateWindow::generate_cpp_h);
            QObject::connect(py_button, &QPushButton::clicked, this, &ControllerGenerateWindow::generate_python_class);
        }

        // utility functions
        // generate h class include 
        void generate_cpp_h()
        {
            // init var
            QString tab = "    ";
            QString class_name = "vehicle_external_command";
            QString comment = "/**************************************************************************\nThis file is generated by px4_cmd gui controller for external command, \nif you don't know how this file works, please don't edit it.\n**************************************************************************/\n";
            QString class_h = comment + "#ifndef VEHICEL_EXTERNAL_COMMAND_H\n#define VEHICEL_EXTERNAL_COMMAND_H\n";
            QStringList modules = {
                "vector",
                "string",
                "thread",
                "ros/ros.h",
                "px4_cmd/Command.h",
                "tf/LinearMath/Quaternion.h",
                "tf/LinearMath/Transform.h",
                "tf/transform_datatypes.h",
            };
            QStringList addition_args = {
                "#define PI 3.14159265358979323846",
                "using namespace std;"
            };
            QStringList private_vars = {
                "double update_time = 0.1",
                "double init_R",
                "double init_P",
                "double init_Y",
                "px4_cmd::Command external_cmd",
                "tf::Quaternion quat;"
            };
            QStringList public_vars = {
                "double init_x",
                "double init_y",
                "double init_z"
            };
            QStringList public_functions = {
                "void start(string node)",
                "void set_position(double x, double y, double z, int frame)",
                "void set_velocity(double vx, double vy, double vz, int frame)",
                "void set_velocity_with_height(double vx, double vy, double z, int frame)"
            };
            QStringList subscribers = {};
            QStringList subscribers_functions = {};
            QStringList subscribers_msgs = {};
            QStringList subscribers_destinations = {};

            // get generate data
            // position and pose
            if (pos_box->isChecked() || pose_box->isChecked())
            {
                modules.push_back("geometry_msgs/PoseStamped.h");
                subscribers.push_back("pos_pose_sub");
                subscribers_msgs.push_back("geometry_msgs::PoseStamped");
                subscribers_functions.push_back("pos_cb");
                subscribers_destinations.push_back("topic_header + \"local_position/pose\"");
                if (pos_box->isChecked())
                {
                    public_vars.push_back("double position[3]");
                }
                if (pose_box->isChecked())
                {
                    public_vars.push_back("double attitude[3]");
                    private_vars.push_back("double R");
                    private_vars.push_back("double P");
                    private_vars.push_back("double Y");
                }
            }
            // velocity and angle rate
            if (vel_box->isChecked() || angle_rate_box->isChecked())
            {
                modules.push_back("geometry_msgs/TwistStamped.h");
                subscribers.push_back("vel_angle_rate_sub");
                subscribers_msgs.push_back("geometry_msgs::TwistStamped");
                subscribers_functions.push_back("vel_cb");
                subscribers_destinations.push_back("topic_header + \"local_position/velocity_local\"");
                if (vel_box->isChecked())
                {
                    public_vars.push_back("double velocity[3]");
                }
                if (angle_rate_box->isChecked())
                {
                    public_vars.push_back("double angle_rate[3]");
                }
            }

            // generate includes
            for (size_t i = 0; i < modules.size(); i++)
            {
                class_h = class_h + "#include <" + modules[i] + ">\n";
            }
            class_h = class_h + "\n";

            // generate addition args
            for (size_t i = 0; i < addition_args.size(); i++)
            {
                class_h = class_h + addition_args[i] + "\n";
            }
            class_h = class_h + "\n";

            // generate class
            class_h = class_h + "class " + class_name + "\n{\n" + tab + "private:\n";
            // private var
            for (size_t i = 0; i < private_vars.size(); i++)
            {
                class_h = class_h + tab + tab + private_vars[i] + ";\n";
            }
            // add subscribers
            for (size_t i = 0; i < subscribers.size(); i++)
            {
                class_h = class_h + tab + tab + "ros::Subscriber " + subscribers[i] + ";\n";
            }
            // add publisher
            class_h = class_h + tab + tab + "ros::Publisher ext_cmd_pub;\n";
            // add functions
            for (size_t i = 0; i < subscribers_functions.size(); i++)
            {
                class_h = class_h + tab + tab + "void " + subscribers_functions[i] + "(const " + subscribers_msgs[i] + "::ConstPtr &msg);\n";
            }
            class_h = class_h + tab + tab + "void ros_thread_fun();\n\n";
            // public var
            class_h = class_h + tab + "public:\n";
            for (size_t i = 0; i < public_vars.size(); i++)
            {
                class_h = class_h + tab + tab + public_vars[i] + ";\n";
            }
            // add functions
            for (size_t i = 0; i < public_functions.size(); i++)
            {
                class_h = class_h + tab + tab + public_functions[i] + ";\n";
            }
            // class end
            class_h = class_h + "};\n\n";

            // implement start function
            class_h = class_h + "void " + class_name + "::start(string node)\n{\n";
            class_h = class_h + tab + "string node_name = node;\n";
            class_h = class_h + tab + "string topic_header = \"/\" + node_name + \"/mavros/\";\n";
            class_h = class_h + tab + "external_cmd.Mode = px4_cmd::Command::Move;\n";
            class_h = class_h + tab + "external_cmd.Move_frame = px4_cmd::Command::ENU;\n";
            class_h = class_h + tab + "external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;\n";
            class_h = class_h + tab + "int argc = 0;\n";
            class_h = class_h + tab + "char **argv;\n";
            class_h = class_h + tab + "ros::init(argc, argv, \"_ext_cmd\");\n";
            class_h = class_h + tab + "ros::NodeHandle nh(\"/\" + node_name, \"px4_cmd\");\n";
            class_h = class_h + tab + "ros::param::get((\"/\" + node_name + \"/init_x\").c_str(), init_x);\n";
            class_h = class_h + tab + "ros::param::get((\"/\" + node_name + \"/init_y\").c_str(), init_y);\n";
            class_h = class_h + tab + "ros::param::get((\"/\" + node_name + \"/init_z\").c_str(), init_z);\n";
            class_h = class_h + tab + "ros::param::get((\"/\" + node_name + \"/init_R\").c_str(), init_R);\n";
            class_h = class_h + tab + "ros::param::get((\"/\" + node_name + \"/init_P\").c_str(), init_P);\n";
            class_h = class_h + tab + "ros::param::get((\"/\" + node_name + \"/init_Y\").c_str(), init_Y);\n";
            for (size_t i = 0; i < subscribers.size(); i++)
            {
                class_h = class_h + tab + subscribers[i] + " = nh.subscribe<" + subscribers_msgs[i] + ">(" + subscribers_destinations[i] + ", 20, &" + class_name + "::" + subscribers_functions[i] + ", this);\n";
            }
            class_h = class_h + tab + "ext_cmd_pub = nh.advertise<px4_cmd::Command>(\"external_command\", 50);\n";
            class_h = class_h + tab + "while (!ros::ok())\n" + tab + "{\n" + tab + tab + "ros::Duration(update_time).sleep();\n" + tab + "}\n";
            class_h = class_h + tab + "std::thread ros_thread(&" + class_name + "::ros_thread_fun, this);\n";
            class_h = class_h + tab + "ros_thread.detach();\n";
            class_h = class_h + "};\n\n";

            // generate ros thread function
            class_h = class_h + "void " + class_name + "::ros_thread_fun()\n{\n";
            class_h = class_h + tab + "while (ros::ok())\n" + tab + "{\n";
            class_h = class_h + tab + tab + "ext_cmd_pub.publish(external_cmd);\n";
            class_h = class_h + tab + tab + "ros::Duration(update_time).sleep();\n";
            class_h = class_h + tab + tab + "ros::spinOnce();\n";
            class_h = class_h + tab + "}\n};\n\n";

            for (size_t i = 0; i < subscribers_functions.size(); i++)
            {
                class_h = class_h + "void " + class_name + "::" + subscribers_functions[i] + "(const " + subscribers_msgs[i] + "::ConstPtr &msg)" + "\n{\n";
                if (subscribers_functions[i] == "pos_cb")
                {
                    if (pos_box->isChecked())
                    {
                        class_h = class_h + tab + "position[0] = msg->pose.position.x + init_x;\n";
                        class_h = class_h + tab + "position[1] = msg->pose.position.y + init_y;\n";
                        class_h = class_h + tab + "position[2] = msg->pose.position.z + init_z;\n";
                    }
                    if (pose_box->isChecked())
                    {
                        class_h = class_h + tab + "tf::quaternionMsgToTF(msg->pose.orientation, quat);\n";
                        class_h = class_h + tab + "tf::Matrix3x3(quat).getRPY(R, P, Y);\n";
                        class_h = class_h + tab + "attitude[0] = (P + init_P) * 180 / PI;\n";
                        class_h = class_h + tab + "attitude[1] = (R + init_R) * 180 / PI;\n";
                        class_h = class_h + tab + "attitude[2] = (Y + init_Y) * 180 / PI;\n";
                    }
                    class_h = class_h + "};\n\n";
                    continue;
                }
                if (subscribers_functions[i] == "vel_cb")
                {
                    if (vel_box->isChecked())
                    {
                        class_h = class_h + tab + "velocity[0] = msg->twist.linear.x;\n";
                        class_h = class_h + tab + "velocity[1] = msg->twist.linear.y;\n";
                        class_h = class_h + tab + "velocity[2] = msg->twist.linear.z;\n";
                    }
                    if (angle_rate_box->isChecked())
                    {
                        class_h = class_h + tab + "angle_rate[0] = msg->twist.angular.x * 180 / PI;\n";
                        class_h = class_h + tab + "angle_rate[1] = msg->twist.angular.y * 180 / PI;\n";
                        class_h = class_h + tab + "angle_rate[2] = msg->twist.angular.z * 180 / PI;\n";
                    }
                    class_h = class_h + "};\n\n";
                    continue;
                }
            }
            
            // generate set position function
            class_h = class_h + "void " + class_name + "::set_position(double x, double y, double z, int frame)\n{\n";
            class_h = class_h + tab + "external_cmd.Mode = px4_cmd::Command::Move;\n";
            class_h = class_h + tab + "external_cmd.Move_frame = frame;\n";
            class_h = class_h + tab + "external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;\n";
            class_h = class_h + tab + "external_cmd.desire_cmd[0] = x;\n";
            class_h = class_h + tab + "external_cmd.desire_cmd[1] = y;\n";
            class_h = class_h + tab + "external_cmd.desire_cmd[2] = z;\n";
            class_h = class_h + "};\n\n";

            // generate set velocity function
            class_h = class_h + "void " + class_name + "::set_velocity(double vx, double vy, double vz, int frame)\n{\n";
            class_h = class_h + tab + "external_cmd.Mode = px4_cmd::Command::Move;\n";
            class_h = class_h + tab + "external_cmd.Move_frame = frame;\n";
            class_h = class_h + tab + "external_cmd.Move_mode = px4_cmd::Command::XYZ_VEL;\n";
            class_h = class_h + tab + "external_cmd.desire_cmd[0] = vx;\n";
            class_h = class_h + tab + "external_cmd.desire_cmd[1] = vy;\n";
            class_h = class_h + tab + "external_cmd.desire_cmd[2] = vz;\n";
            class_h = class_h + "};\n\n";

            // generate set velocity with height function
            class_h = class_h + "void " + class_name + "::set_velocity_with_height(double vx, double vy, double z, int frame)\n{\n";
            class_h = class_h + tab + "external_cmd.Mode = px4_cmd::Command::Move;\n";
            class_h = class_h + tab + "external_cmd.Move_frame = frame;\n";
            class_h = class_h + tab + "external_cmd.Move_mode = px4_cmd::Command::XY_VEL_Z_POS;\n";
            class_h = class_h + tab + "external_cmd.desire_cmd[0] = vx;\n";
            class_h = class_h + tab + "external_cmd.desire_cmd[1] = vy;\n";
            class_h = class_h + tab + "external_cmd.desire_cmd[2] = z;\n";
            class_h = class_h + "};\n\n";

            // endif for header
            class_h = class_h + "#endif";

            string root_dir;
            get_cmd_output("rospack find px4_cmd", root_dir);
            strip(root_dir, "\n");
            strip(root_dir);
            std::ofstream outputFile(root_dir + "/include/vehicle_external_command.h");
            if (outputFile.is_open())
            {
                outputFile << class_h.toStdString() << endl;
            }
            else
            {
                msg_box = new QMessageBox(this);
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setWindowTitle("Error");
                msg_box->setText(("Fail to save C++ header to \n" + root_dir + "/include/vehicle_external_command.h").c_str());
                msg_box->exec();
                return;
            }
            // successful info
            msg_box = new QMessageBox(this);
            msg_box->setIcon(QMessageBox::Icon::Information);
            msg_box->setWindowTitle("Successfully Save C++ Header File");
            msg_box->setText(("Successfully save C++ header file to \n" + root_dir + "/include/vehicle_external_command.h").c_str());
            msg_box->exec();
        }

        // generate h class include
        void generate_python_class()
        {
            // init var
            QString tab = "    ";
            QString comment = "##############################################################################\n#   This file is generated by px4_cmd gui controller for external command,   #\n#   if you don't know how this file works, please don't edit it.             #\n##############################################################################\n";
            QString python_class = comment + "\n";
            QStringList modules = {
                "import time",
                "import rospy",
                "import threading",
                "from tf.transformations import euler_from_quaternion",
                "from px4_cmd.msg import Command",
            };
            QStringList addition_args = {
                "PI = 3.14159265358979323846"
            };
            QStringList vars = {
                "self.update_time = 0.02",
                "self.init_x = 0",
                "self.init_y = 0",
                "self.init_z = 0",
                "self.init_R = 0",
                "self.init_P = 0",
                "self.init_Y = 0",
                "self.external_cmd = Command()"
            };
            QStringList subscribers_functions = {};
            QStringList subscribers_msgs = {};
            QStringList subscribers_destinations = {};

            // get generate data
            // position and pose
            if (pos_box->isChecked() || pose_box->isChecked())
            {
                modules.push_back("from geometry_msgs.msg import PoseStamped");
                subscribers_msgs.push_back("PoseStamped");
                subscribers_functions.push_back("pos_cb");
                subscribers_destinations.push_back("topic_header + \"local_position/pose\"");
                if (pos_box->isChecked())
                {
                    vars.push_back("self.position = [0.0, 0.0, 0.0]");
                }
                if (pose_box->isChecked())
                {
                    vars.push_back("self.attitude = [0.0, 0.0, 0.0]");
                }
            }
            // velocity and angle rate
            if (vel_box->isChecked() || angle_rate_box->isChecked())
            {
                modules.push_back("from geometry_msgs.msg import TwistStamped");
                subscribers_msgs.push_back("TwistStamped");
                subscribers_functions.push_back("vel_cb");
                subscribers_destinations.push_back("topic_header + \"local_position/velocity_local\"");
                if (vel_box->isChecked())
                {
                    vars.push_back("self.velocity = [0.0, 0.0, 0.0]");
                }
                if (angle_rate_box->isChecked())
                {
                    vars.push_back("self.angle_rate = [0.0, 0.0, 0.0]");
                }
            }

            // generate includes
            for (size_t i = 0; i < modules.size(); i++)
            {
                python_class = python_class + modules[i] + "\n";
            }
            python_class = python_class + "\n";

            // generate addition args
            for (size_t i = 0; i < addition_args.size(); i++)
            {
                python_class = python_class + addition_args[i] + "\n";
            }
            python_class = python_class + "\n";

            // generate class
            python_class = python_class + "class vehicle_external_command:\n";
            
            // add init function
            python_class = python_class + tab + "def __init__(self) -> None:\n";
            for (size_t i = 0; i < vars.size(); i++)
            {
                python_class = python_class + tab + tab + vars[i] + "\n";
            }
            python_class = python_class + tab + tab + "pass\n\n";

            // add start function
            python_class = python_class + tab + "def start(self, node: str) -> None:\n";
            python_class = python_class + tab + tab + "node_name = node\n";
            python_class = python_class + tab + tab + "topic_header = \"/\" + node_name + \"/mavros/\"\n";
            python_class = python_class + tab + tab + "self.external_cmd.Mode = Command.Move\n";
            python_class = python_class + tab + tab + "self.external_cmd.Move_frame = Command.ENU\n";
            python_class = python_class + tab + tab + "self.external_cmd.Move_mode = Command.XYZ_POS\n";
            python_class = python_class + tab + tab + "self.init_x = rospy.get_param(\"/\" + node_name + \"/init_x\")\n";
            python_class = python_class + tab + tab + "self.init_y = rospy.get_param(\"/\" + node_name + \"/init_y\")\n";
            python_class = python_class + tab + tab + "self.init_z = rospy.get_param(\"/\" + node_name + \"/init_z\")\n";
            python_class = python_class + tab + tab + "self.init_R = rospy.get_param(\"/\" + node_name + \"/init_R\")\n";
            python_class = python_class + tab + tab + "self.init_P = rospy.get_param(\"/\" + node_name + \"/init_P\")\n";
            python_class = python_class + tab + tab + "self.init_Y = rospy.get_param(\"/\" + node_name + \"/init_Y\")\n";
            for (size_t i = 0; i < subscribers_msgs.size(); i++)
            {
                python_class = python_class + tab + tab + "rospy.Subscriber(" + subscribers_destinations[i] + ", " + subscribers_msgs[i] + ", self." + subscribers_functions[i] + ", queue_size=20)\n";
            }
            python_class = python_class + tab + tab + "self.ext_cmd_pub = rospy.Publisher(\"/\" + node_name + \"/px4_cmd/external_command\", Command, queue_size=50)\n";
            python_class = python_class + tab + tab + "while rospy.is_shutdown():\n" + tab + tab + tab + "time.sleep(self.update_time)\n";
            python_class = python_class + tab + tab + "progress_1 = threading.Thread(target=self.ros_sub_thread)\n";
            python_class = python_class + tab + tab + "progress_1.start()\n";
            python_class = python_class + tab + tab + "progress_2 = threading.Thread(target=self.ros_pub_thread)\n";
            python_class = python_class + tab + tab + "progress_2.start()\n\n";
        
            // generate ros subscribe thread function
            python_class = python_class + tab + "def ros_sub_thread(self):\n";
            python_class = python_class + tab + tab + "rospy.spin()\n\n";

            // generate ros publish thread function
            python_class = python_class + tab + "def ros_pub_thread(self):\n";
            python_class = python_class + tab + tab + "while not rospy.is_shutdown():\n" + tab + tab + tab + "self.ext_cmd_pub.publish(self.external_cmd)\n" + tab + tab + tab + "time.sleep(self.update_time)\n\n";

            // generate subscriber function
            for (size_t i = 0; i < subscribers_functions.size(); i++)
            {
                python_class = python_class + tab + "def " + subscribers_functions[i] + "(self, msg: " + subscribers_msgs[i] + "):\n";
                if (subscribers_functions[i] == "pos_cb")
                {
                    if (pos_box->isChecked())
                    {
                        python_class = python_class + tab + tab + "self.position[0] = msg.pose.position.x + self.init_x\n";
                        python_class = python_class + tab + tab + "self.position[1] = msg.pose.position.y + self.init_y\n";
                        python_class = python_class + tab + tab + "self.position[2] = msg.pose.position.z + self.init_z\n";
                    }
                    if (pose_box->isChecked())
                    {
                        python_class = python_class + tab + tab + "(R, P, Y) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])\n";
                        python_class = python_class + tab + tab + "self.attitude[0] = (P + self.init_P) * 180 / PI\n";
                        python_class = python_class + tab + tab + "self.attitude[1] = (R + self.init_R) * 180 / PI\n";
                        python_class = python_class + tab + tab + "self.attitude[2] = (Y + self.init_Y) * 180 / PI\n";
                    }
                    python_class = python_class + "\n";
                    continue;
                }
                if (subscribers_functions[i] == "vel_cb")
                {
                    if (vel_box->isChecked())
                    {
                        python_class = python_class + tab + tab + "self.velocity[0] = msg.twist.linear.x\n";
                        python_class = python_class + tab + tab + "self.velocity[1] = msg.twist.linear.y\n";
                        python_class = python_class + tab + tab + "self.velocity[2] = msg.twist.linear.z\n";
                    }
                    if (angle_rate_box->isChecked())
                    {
                        python_class = python_class + tab + tab + "self.angle_rate[0] = msg.twist.angular.x * 180 / PI\n";
                        python_class = python_class + tab + tab + "self.angle_rate[1] = msg.twist.angular.y * 180 / PI\n";
                        python_class = python_class + tab + tab + "self.angle_rate[2] = msg.twist.angular.z * 180 / PI\n";
                    }
                    python_class = python_class + "\n";
                    continue;
                }
            }

            // generate set position function
            python_class = python_class + tab + "def set_position(self, x: float, y: float, z: float, frame: int = Command.ENU):\n";
            python_class = python_class + tab + tab + "self.external_cmd.Mode = Command.Move\n";
            python_class = python_class + tab + tab + "self.external_cmd.Move_frame = frame\n";
            python_class = python_class + tab + tab + "self.external_cmd.Move_mode = Command.XYZ_POS\n";
            python_class = python_class + tab + tab + "self.external_cmd.desire_cmd[0] = x\n";
            python_class = python_class + tab + tab + "self.external_cmd.desire_cmd[1] = y\n";
            python_class = python_class + tab + tab + "self.external_cmd.desire_cmd[2] = z\n";
            python_class = python_class + "\n";

            // generate set velocity function
            python_class = python_class + tab + "def set_velocity(self, vx: float, vy: float, vz: float, frame: int = Command.ENU):\n";
            python_class = python_class + tab + tab + "self.external_cmd.Mode = Command.Move\n";
            python_class = python_class + tab + tab + "self.external_cmd.Move_frame = frame\n";
            python_class = python_class + tab + tab + "self.external_cmd.Move_mode = Command.XYZ_VEL\n";
            python_class = python_class + tab + tab + "self.external_cmd.desire_cmd[0] = vx\n";
            python_class = python_class + tab + tab + "self.external_cmd.desire_cmd[1] = vy\n";
            python_class = python_class + tab + tab + "self.external_cmd.desire_cmd[2] = vz\n";
            python_class = python_class + "\n";

            // generate set velocity with height function
            python_class = python_class + tab + "def set_velocity_with_height(self, vx: float, vy: float, z: float, frame: int = Command.ENU):\n";
            python_class = python_class + tab + tab + "self.external_cmd.Mode = Command.Move\n";
            python_class = python_class + tab + tab + "self.external_cmd.Move_frame = frame\n";
            python_class = python_class + tab + tab + "self.external_cmd.Move_mode = Command.XY_VEL_Z_POS\n";
            python_class = python_class + tab + tab + "self.external_cmd.desire_cmd[0] = vx\n";
            python_class = python_class + tab + tab + "self.external_cmd.desire_cmd[1] = vy\n";
            python_class = python_class + tab + tab + "self.external_cmd.desire_cmd[2] = z\n";
            python_class = python_class + "\n";

            string root_dir;
            get_cmd_output("rospack find px4_cmd", root_dir);
            strip(root_dir, "\n");
            strip(root_dir);
            std::ofstream outputFile(root_dir + "/template/vehicle_external_command.py");
            if (outputFile.is_open())
            {
                outputFile << python_class.toStdString() << endl;
            }
            else
            {
                msg_box = new QMessageBox(this);
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setWindowTitle("Error");
                msg_box->setText(("Fail to save Python class to \n" + root_dir + "/template/vehicle_external_command.py").c_str());
                msg_box->exec();
                return;
            }
            // successful info
            msg_box = new QMessageBox(this);
            msg_box->setIcon(QMessageBox::Icon::Information);
            msg_box->setWindowTitle("Successfully Save Python class File");
            msg_box->setText(("Successfully save Python class file to \n" + root_dir + "/template/vehicle_external_command.py").c_str());
            msg_box->exec();
        }
};

#endif