// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef VEHICEL_EXTERNAL_COMMAND_H
#define VEHICEL_EXTERNAL_COMMAND_H
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <ros/ros.h>
#include <px4_cmd/Command.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <px4_cmd/custom_command.h>

#define PI 3.14159265358979323846
using namespace std;

class vehicle_external_command
{
    private:
        /// @brief ros shutdown flag
        bool ros_shutdown_flag = false;
        /// @brief initial roll angle in rad
        double init_R = 0;
        /// @brief initial pitch angle in rad
        double init_P = 0;
        /// @brief initial yaw angle in rad
        double init_Y = 0;
        /// @brief sync lock, use for Synchronize multiple agents
        bool *sync_lock = nullptr;
        /// @brief external command
        px4_cmd::Command external_cmd;
        /// @brief subscriber for position and pose
        ros::Subscriber pos_pose_sub;
        /// @brief subscriber for velocity and angle rate
        ros::Subscriber vel_angle_rate_sub;
        /// @brief publisher for external command
        ros::Publisher ext_cmd_pub;
        /// @brief subscriber for external command state
        ros::Subscriber ext_state_sub;
        /// mutexes
        /// @brief position callback function mutex
        std::mutex pos_cb_mutex;
        /// @brief velocity callback function mutex
        std::mutex vel_cb_mutex;
        /// @brief change command mutex
        std::mutex change_cmd_mutex;

        /// @brief position and pose subscriber callback function
        /// @param msg message
        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

        /// @brief velocity and angle rate subscriber callback function
        /// @param msg message
        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);

        /// @brief external command state subsceiber callback function
        /// @param msg message
        void ext_state_cb(const std_msgs::Bool::ConstPtr &msg);

        /// @brief check sync lock state
        /// @return if not sync lock or unlock - return false, if sync lock - return true
        bool check_sync_lock();

        /// @brief main thread function
        void ros_thread_fun();

        /// @brief setting position command in 3 axis for vehicle - thread function
        /// @param x desire position in x axis, m
        /// @param y desire position in y axis, m
        /// @param z desire position in z axis, m
        /// @param frame position in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_position_thread_func(double x, double y, double z, int frame = px4_cmd::Command::ENU);

        /// @brief setting position command in 3 axis for vehicle - thread function
        /// @param x desire position in x axis, m
        /// @param y desire position in y axis, m
        /// @param z desire position in z axis, m
        /// @param yaw_cmd desire yaw command, rad
        /// @param frame position in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_position_thread_func(double x, double y, double z, double yaw_cmd, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 3 axis for vehicle - thread function
        /// @param vx desire velocity in x axis, m/s
        /// @param vy desire velocity in y axis, m/s
        /// @param vz desire velocity in z axis, m/s
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity_thread_func(double vx, double vy, double vz, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 3 axis for vehicle - thread function
        /// @param vx desire velocity in x axis, m/s
        /// @param vy desire velocity in y axis, m/s
        /// @param vz desire velocity in z axis, m/s
        /// @param yaw_cmd desire yaw command, rad
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity_thread_func(double vx, double vy, double vz, double yaw_cmd, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 2 axis with height command for vehicle - thread function
        /// @param vx desire velocity in x axis, m/s
        /// @param vy desire velocity in y axis, m/s
        /// @param z desire height, m
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity_with_height_thread_func(double vx, double vy, double z, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 2 axis with height command for vehicle - thread function
        /// @param vx desire velocity in x axis, m/s
        /// @param vy desire velocity in y axis, m/s
        /// @param z desire height, m
        /// @param yaw_cmd desire yaw command, rad
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity_with_height_thread_func(double vx, double vy, double z, double yaw_cmd, int frame = px4_cmd::Command::ENU);

        /// @brief setting custom command - thread function
        /// @param cmd custom command
        void set_custom_command_thread_func(CustomCommand cmd);

        /// @brief setting vehicle to hover mode  - thread function
        void set_hover_thread_func();

        /// @brief setting vehicle to hover mode  - thread function
        /// @param yaw_cmd desire yaw command, rad
        void set_hover_thread_func(double yaw);

    public:
        /// @brief update time for API
        double update_time = 0.02;
        /// @brief if reciving external command
        bool ext_cmd_state = false;
        /// @brief set external command total time
        double total_time = -1;
        /// @brief initial position in x axis, m
        double init_x = 0;
        /// @brief initial position in y axis, m
        double init_y = 0;
        /// @brief initial position in z axis, m
        double init_z = 0;
        /// @brief vehicle position [x y z], m
        double position[3];
        /// @brief vehicle attitude [roll pitch yaw], rad
        double attitude[3];
        /// @brief vehicle velocity [vx vy vz], m/s
        double velocity[3];
        /// @brief vehicle angle rate [x, y, z], rad/s
        double angle_rate[3];
        /// @brief vehicle quaternion
        tf::Quaternion quaternion;

        /// @brief start API node
        /// @param node node name for vehicle, defined in topic name: /{node}/mavros/....
        void start(string node);

        /// @brief start API node - for single vehicle simualtion
        void start();

        /// @brief setting position command in 3 axis for vehicle
        /// @param x desire position in x axis, m
        /// @param y desire position in y axis, m
        /// @param z desire position in z axis, m
        /// @param frame position in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_position(double x, double y, double z, int frame = px4_cmd::Command::ENU);

        /// @brief setting position command in 3 axis for vehicle
        /// @param x desire position in x axis, m
        /// @param y desire position in y axis, m
        /// @param z desire position in z axis, m
        /// @param yaw_cmd desire yaw command, rad
        /// @param frame position in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_position(double x, double y, double z, double yaw_cmd, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 3 axis for vehicle
        /// @param vx desire velocity in x axis, m/s
        /// @param vy desire velocity in y axis, m/s
        /// @param vz desire velocity in z axis, m/s
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity(double vx, double vy, double vz, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 3 axis for vehicle
        /// @param vx desire velocity in x axis, m/s
        /// @param vy desire velocity in y axis, m/s
        /// @param vz desire velocity in z axis, m/s
        /// @param yaw_cmd desire yaw command, rad
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity(double vx, double vy, double vz, double yaw_cmd, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 2 axis with height command for vehicle
        /// @param vx desire velocity in x axis, m/s
        /// @param vy desire velocity in y axis, m/s
        /// @param z desire height, m
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity_with_height(double vx, double vy, double z, int frame = px4_cmd::Command::ENU);

        /// @brief setting velocity command in 2 axis with height command for vehicle
        /// @param vx desire velocity in x axis, m/s
        /// @param vy desire velocity in y axis, m/s
        /// @param z desire height, m
        /// @param yaw_cmd desire yaw command, rad
        /// @param frame velocity in which frame, px4_cmd::Command::ENU (default) / px4_cmd::Command::BODY
        void set_velocity_with_height(double vx, double vy, double z, double yaw_cmd, int frame = px4_cmd::Command::ENU);

        /// @brief setting custom command
        /// @param cmd custom command
        void set_custom_command(CustomCommand cmd);

        /// @brief setting vehicle to hover mode
        void set_hover();

        /// @brief setting vehicle to hover mode
        /// @param yaw_cmd desire yaw command, rad
        void set_hover(double yaw);

        /// @brief set sync lock with bool ptr
        /// @param sync_lock bool ptr
        void set_sync_lock(bool *sync_lock_);

        /// @brief reset sync lock to nullptr
        void reset_sync_lock();

        /// @brief shutdown API node
        void shutdown();
};
#endif