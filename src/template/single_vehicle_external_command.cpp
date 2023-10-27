#include <px4_cmd/single_vehicle_external_command.hpp>
#include <vector>
#include <string>
#include <thread>
#include <ros/ros.h>
#include <px4_cmd/Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265358979323846
using namespace std;

void single_vehicle_external_command::start()
{
    string topic_header = "/mavros/";
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = px4_cmd::Command::ENU;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    int argc = 0;
    char **argv;
    ros::init(argc, argv, "ext_cmd");
    ros::NodeHandle nh("~");
    pos_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_header + "local_position/pose", 20, &single_vehicle_external_command::pos_cb, this);
    vel_angle_rate_sub = nh.subscribe<geometry_msgs::TwistStamped>(topic_header + "local_position/velocity_local", 20, &single_vehicle_external_command::vel_cb, this);
    ext_state_sub = nh.subscribe<std_msgs::Bool>("/px4_cmd/ext_on", 20, &single_vehicle_external_command::ext_state_cb, this);
    ext_cmd_pub = nh.advertise<px4_cmd::Command>("/px4_cmd/ext_command", 50);
    while (!ros::ok())
    {
        ros::Duration(update_time).sleep();
    }
    std::thread ros_thread(&single_vehicle_external_command::ros_thread_fun, this);
    ros_thread.detach();
};

void single_vehicle_external_command::ros_thread_fun()
{
    double t = 0;
    while (ros::ok() && !ros_shutdown)
    {
        while (ext_cmd_pub.getNumSubscribers() < 1)
        {
            ROS_INFO("External Command: Waiting for user-define mode!");
            ros::Duration(1).sleep();
            t = 0;
        }
        external_cmd.ext_time = t;
        external_cmd.ext_total_time = total_time;
        ext_cmd_pub.publish(external_cmd);
        t = t + update_time;
        ros::Duration(update_time).sleep();
        ros::spinOnce();
    }
    ros::shutdown();
};

void single_vehicle_external_command::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    position[2] = msg->pose.position.z;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(attitude[0], attitude[1], attitude[2]);
};

void single_vehicle_external_command::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    velocity[0] = msg->twist.linear.x;
    velocity[1] = msg->twist.linear.y;
    velocity[2] = msg->twist.linear.z;
    angle_rate[0] = msg->twist.angular.x * 180 / PI;
    angle_rate[1] = msg->twist.angular.y * 180 / PI;
    angle_rate[2] = msg->twist.angular.z * 180 / PI;
};

void single_vehicle_external_command::ext_state_cb(const std_msgs::Bool::ConstPtr &msg)
{
    ext_on = msg->data;
}

void single_vehicle_external_command::set_position(double x, double y, double z, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    external_cmd.desire_cmd[0] = x;
    external_cmd.desire_cmd[1] = y;
    external_cmd.desire_cmd[2] = z;
};

void single_vehicle_external_command::set_position(double x, double y, double z, double yaw, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    external_cmd.desire_cmd[0] = x;
    external_cmd.desire_cmd[1] = y;
    external_cmd.desire_cmd[2] = z;
    external_cmd.yaw_cmd = yaw;
};

void single_vehicle_external_command::set_velocity(double vx, double vy, double vz, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_VEL;
    external_cmd.desire_cmd[0] = vx;
    external_cmd.desire_cmd[1] = vy;
    external_cmd.desire_cmd[2] = vz;
};

void single_vehicle_external_command::set_velocity(double vx, double vy, double vz, double yaw, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_VEL;
    external_cmd.desire_cmd[0] = vx;
    external_cmd.desire_cmd[1] = vy;
    external_cmd.desire_cmd[2] = vz;
    external_cmd.yaw_cmd = yaw;
};

void single_vehicle_external_command::set_velocity_with_height(double vx, double vy, double z, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XY_VEL_Z_POS;
    external_cmd.desire_cmd[0] = vx;
    external_cmd.desire_cmd[1] = vy;
    external_cmd.desire_cmd[2] = z;
};

void single_vehicle_external_command::set_velocity_with_height(double vx, double vy, double z, double yaw, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XY_VEL_Z_POS;
    external_cmd.desire_cmd[0] = vx;
    external_cmd.desire_cmd[1] = vy;
    external_cmd.desire_cmd[2] = z;
    external_cmd.yaw_cmd = yaw;
};

void single_vehicle_external_command::set_hover()
{
    external_cmd.Mode = px4_cmd::Command::Hover;
    external_cmd.Move_frame = px4_cmd::Command::ENU;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
};

void single_vehicle_external_command::set_hover(double yaw)
{
    external_cmd.Mode = px4_cmd::Command::Hover;
    external_cmd.Move_frame = px4_cmd::Command::ENU;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    external_cmd.yaw_cmd = yaw;
};

void single_vehicle_external_command::shutdown()
{
    ros_shutdown = true;
};