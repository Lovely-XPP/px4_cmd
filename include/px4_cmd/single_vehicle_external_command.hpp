// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef SINGLE_VEHICEL_EXTERNAL_COMMAND_H
#define SINGLE_VEHICEL_EXTERNAL_COMMAND_H
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

/// @brief single vehicle external command API, including vehicle information and sending command.
class single_vehicle_external_command
{
    private:
        /// @brief ros shutdown flag
        bool ros_shutdown = false;
        /// @brief external command
        px4_cmd::Command external_cmd;
        /// @brief subscriber for position and pose
        ros::Subscriber pos_pose_sub;
        /// @brief subscriber for velocity and angle rate
        ros::Subscriber vel_angle_rate_sub;
        /// @brief publish if recive external command
        ros::Publisher ext_cmd_pub;
        /// @brief subscriber for external command state
        ros::Subscriber ext_state_sub;

        /// @brief position and pose subscriber callback function
        /// @param msg message
        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

        /// @brief velocity and angle rate subscriber callback function
        /// @param msg message
        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

        /// @brief external command state subsceiber callback function
        /// @param msg message
        void ext_state_cb(const std_msgs::Bool::ConstPtr &msg);

        /// @brief main thread function
        void ros_thread_fun();

    public:
        /// @brief update time for API
        double update_time = 0.05;
        /// @brief if reciving external command
        bool ext_cmd_state = false;
        /// @brief set external command total time
        double total_time = -1;
        /// @brief vehicle position [x y z], m
        double position[3];
        /// @brief vehicle attitude [roll pitch yaw], rad
        double attitude[3];
        /// @brief vehicle velocity [vx vy vz], m/s
        double velocity[3];
        /// @brief vehicle angle rate [x, y, z], rad/s
        double angle_rate[3];

        /// @brief start API node
        void start();

        /// @brief setting position command in 3 axis for vehicle
        /// @param x desire position in x axis
        /// @param y desire position in y axis
        /// @param z desire position in z axis
        /// @param frame position in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_position(double x, double y, double z, int frame = px4_cmd::Command::ENU);

        /// @brief setting position command in 3 axis for vehicle
        /// @param x desire position in x axis
        /// @param y desire position in y axis
        /// @param z desire position in z axis
        /// @param yaw_cmd desire yaw command
        /// @param frame position in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_position(double x, double y, double z, double yaw_cmd, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 3 axis for vehicle
        /// @param vx desire velocity in x axis
        /// @param vy desire velocity in y axis
        /// @param vz desire velocity in z axis
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity(double vx, double vy, double vz, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 3 axis for vehicle
        /// @param vx desire velocity in x axis
        /// @param vy desire velocity in y axis
        /// @param vz desire velocity in z axis
        /// @param yaw_cmd desire yaw command
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity(double vx, double vy, double vz, double yaw_cmd, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 2 axis with height command for vehicle
        /// @param vx desire velocity in x axis
        /// @param vy desire velocity in y axis
        /// @param z desire height
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity_with_height(double vx, double vy, double z, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 2 axis with height command for vehicle
        /// @param vx desire velocity in x axis
        /// @param vy desire velocity in y axis
        /// @param z desire height
        /// @param yaw_cmd desire yaw command
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity_with_height(double vx, double vy, double z, double yaw_cmd, int frame = px4_cmd::Command::ENU);

        /// @brief setting vehicle to hover mode
        void set_hover();

        /// @brief setting vehicle to hover mode
        /// @param yaw_cmd desire yaw command
        void set_hover(double yaw);

        /// @brief shutdown API node
        void shutdown();
};
#endif
