#include <ros/ros.h>
#include <vector>
#include <math.h>

#include <px4_cmd/Command.h>
#include <px4_cmd/vehicle_external_command.h>

using namespace std;

void detect_px4();

// global var
vector<string> nodes;
vector<vehicle_external_command *> uav;

int main(int argc, char *argv[])
{
    // init ros node
    ros::init(argc, argv, "External_Command_Center");

    // detect px4 running to get nodes for starting external command module
    detect_px4();

    // init all agents uav
    for (auto item = nodes.begin(); item != nodes.end(); item++)
    {
        vehicle_external_command *vec = new vehicle_external_command();
        vec->start(*item);
        uav.push_back(vec);
    }
    std::cout << (("[Info] External Command Start! Vehicle Count: " + to_string(nodes.size()) + ".")) << std::endl;
    
    /************************ Edit  Here ************************/
    double t = 0;
    while (ros::ok())
    {
        for (size_t i = 0; i < nodes.size(); i++)
        {
            uav[i]->set_position(uav[i]->init_x + 2 * sin(0.4 * t), uav[i]->init_y + 2 * cos(0.4 * t), 3, px4_cmd::Command::ENU);
        }
        ros::Duration(0.02).sleep();
        t += 0.02;
    }
    return 0;

    /*
    // Example to get initial position for uavs
    uav[0]->init_x;
    uav[0]->init_y;
    uav[0]->init_z;

    // Example to get position (m) / attitude (deg) / velocity (m/s) / angle_rate (deg/s) for uavs
    uav[0]->position[0]; // get x position of uav 0
    uav[0]->velocity[1]; // get y velocity of uav 0
    uav[0]->angle_rate[1]; // get x angle rate of uav 0
    uav[0]->attitude[0]; // get x attitude (Row) of uav 0
    uav[0]->attitude[1]; // get y attitude (Pitch) of uav 0
    uav[0]->attitude[2]; // get z attitude (Yaw) of uav 0
    uav->quaternion; // get quaternion of uav (tf::Quaternion)
    // get element of quaternion
    uav->quaternion.x();
    uav->quaternion.y();
    uav->quaternion.z();
    uav->quaternion.w();

    // Example to set desite position / velocity / velocity with height for uavs
    // set desired position to [1, 1, 1] of uav 0, use Body frame (it is recommanded for using global frame: px4_cmd::Command::ENU)
    uav[0]->set_position(1, 1, 1, px4_cmd::Command::BODY);

    // set desired velocity to [1, 1, 1] of uav 0, use global ENU frame
    uav[0]->set_velocity(1, 1, 1, px4_cmd::Command::ENU);
    // set desired velocity with height to [1, 1, 1] of uav 0, use global ENU frame
    uav[0]->set_velocity_with_height(1, 1, 1, px4_cmd::Command::ENU);
    */
};

void detect_px4()
{
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
        nodes.emplace_back(tmp);
    }
    if (nodes.size() <= 0)
    {
        throw std::runtime_error("[Error] Can not detect PX4 Running!");
    }
};