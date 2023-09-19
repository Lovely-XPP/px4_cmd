#include <ros/ros.h>
#include <QVector>
#include <QString>
#include <QStringList>
#include <math.h>

#include <px4_cmd/Command.h>
#include <print_utility/printf_utility.h>
#include <vehicle_external_command.h>

using namespace std;

string detect_env();
bool detect_px4();

// global var
QStringList nodes;
QVector<vehicle_external_command *> data;

int main(int argc, char *argv[])
{
    // detect env
    string err = detect_env();
    if (err != "")
    {
        Error(err.c_str());
        exit(0);
    }

    // init ros node
    ros::init(argc, argv, "External_Command_Center");

    // detect px4 running to get nodes for starting external command module
    while (!ros::ok() || !detect_px4())
    {
        Warning("Not Detected PX4 Running! Waiting for PX4 running ...");
        sleep(1);
    }

    // init all agents data
    for (auto item = nodes.begin(); item != nodes.end(); item++)
    {
        vehicle_external_command *vec = new vehicle_external_command();
        vec->start((*item).toStdString());
        data.push_back(vec);
    }
    Info(("External Command Start! Vehicle Count: " + to_string(nodes.size())).c_str());
    /************************ Edit  Here ************************/
    double t = 0;
    while (ros::ok())
    {
        for (size_t i = 0; i < nodes.size(); i++)
        {
            data[i]->set_position(data[i]->init_x + 2 * sin(0.4 * t), data[i]->init_y + 2 * cos(0.4 * t), 3, px4_cmd::Command::ENU);
        }
        ros::Duration(0.02).sleep();
        t += 0.02;
    }
    return 0;

    // Example to get initial position for uavs
    data[0]->init_x;
    data[0]->init_y;
    data[0]->init_z;

    // Example to get position (m) / attitude (deg) / velocity (m/s) / angle_rate (deg/s) for uavs
    data[0]->position[0]; // get x position of uav 0
    data[0]->velocity[1]; // get y velocity of uav 0
    data[0]->angle_rate[1]; // get x angle rate of uav 0
    data[0]->attitude[0]; // get x attitude (Row) of uav 0
    data[0]->attitude[1]; // get y attitude (Pitch) of uav 0
    data[0]->attitude[2]; // get z attitude (Yaw) of uav 0

    // Example to set desite position / velocity / velocity with height for uavs
    // set desired position to [1, 1, 1] of uav 0, use Body frame (it is recommanded for using global frame: px4_cmd::Command::ENU)
    data[0]->set_position(1, 1, 1, px4_cmd::Command::BODY);

    // set desired velocity to [1, 1, 1] of uav 0, use global ENU frame
    data[0]->set_velocity(1, 1, 1, px4_cmd::Command::ENU);
    // set desired velocity with height to [1, 1, 1] of uav 0, use global ENU frame 
    data[0]->set_velocity_with_height(1, 1, 1, px4_cmd::Command::ENU);

    return 0;
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
