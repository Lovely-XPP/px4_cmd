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

class single_vehicle_external_command
{
    private:
        bool ros_shutdown = false;
        double update_time = 0.1;
        tf::Quaternion quat;
        px4_cmd::Command external_cmd;
        ros::Subscriber pos_pose_sub;
        ros::Subscriber vel_angle_rate_sub;
        ros::Subscriber ext_state_sub;
        ros::Publisher ext_cmd_pub;
        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void ext_state_cb(const std_msgs::Bool::ConstPtr &msg);
        void ros_thread_fun();

    public:
        bool ext_on = false;
        double total_time = -1;
        double position[3];
        double attitude[3];
        double velocity[3];
        double angle_rate[3];
        void start();
        void set_position(double x, double y, double z, int frame);
        void set_position(double x, double y, double z, double yaw_cmd, int frame);
        void set_velocity(double vx, double vy, double vz, int frame);
        void set_velocity(double vx, double vy, double vz, double yaw_cmd, int frame);
        void set_velocity_with_height(double vx, double vy, double z, int frame);
        void set_velocity_with_height(double vx, double vy, double z, double yaw_cmd, int frame);
        void set_hover();
        void set_hover(double yaw);
        void shutdown();
};
#endif
