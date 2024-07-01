#include <ros/ros.h>
#include <vector>
#include <math.h>

#include <px4_cmd/Command.h>
#include <px4_cmd/vehicle_external_command.h>

using namespace std;

int main(int argc, char *argv[])
{
    // init ros node
    ros::init(argc, argv, "External_Command_Center");

    // init uav
    vehicle_external_command *uav = new vehicle_external_command();
    std::cout << "External Command Start! Vehicle Count: 1." << std::endl;
    
    /************************ Edit  Here ************************/
    double t = 0;
    while (ros::ok())
    {
        uav->set_position(uav->init_x + 2 * sin(0.4 * t), uav->init_y + 2 * cos(0.4 * t), 3, px4_cmd::Command::ENU);
        ros::Duration(0.02).sleep();
        t += 0.02;
    }
    return 0;
    /*
    // Example to get initial position for uav
    uav->init_x;
    uav->init_y;
    uav->init_z;

    // Example to get position (m) / attitude (deg) / velocity (m/s) / angle_rate (deg/s) for uavs
    uav->position[0]; // get x position of uav 0
    uav->velocity[1]; // get y velocity of uav 0
    uav->angle_rate[1]; // get x angle rate of uav 0
    uav->attitude[0]; // get x attitude (Row) of uav 0
    uav->attitude[1]; // get y attitude (Pitch) of uav 0
    uav->attitude[2]; // get z attitude (Yaw) of uav 0
    uav->quaternion; // get quaternion of uav (tf::Quaternion)
    // get element of quaternion
    uav->quaternion.x();
    uav->quaternion.y();
    uav->quaternion.z();
    uav->quaternion.w();

    // Example to set desite position / velocity / velocity with height for uavs
    // set desired position to [1, 1, 1] of uav 0, use Body frame (it is recommanded for using global frame: px4_cmd::Command::ENU)
    uav->set_position(1, 1, 1, px4_cmd::Command::BODY);

    // set desired velocity to [1, 1, 1] of uav 0, use global ENU frame
    uav->set_velocity(1, 1, 1, px4_cmd::Command::ENU);
    // set desired velocity with height to [1, 1, 1] of uav 0, use global ENU frame
    uav->set_velocity_with_height(1, 1, 1, px4_cmd::Command::ENU);
    */
};