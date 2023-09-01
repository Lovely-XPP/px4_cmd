#ifndef VEHICEL_COMMAND_H
#define VEHICEL_COMMAND_H
#include <ros/ros.h>
#include <string>
#include <QStringList>
#include <QVector>
#include <QThread>
#include <thread>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_cmd/Command.h>

#define PI 3.14159265358979323846

using namespace std;

class vehicle_command
{
    private:
        // setting
        double update_time = 0.02;
        ros::Subscriber controller_cmd_sub;
        ros::Subscriber current_pos_sub;
        ros::Subscriber current_state_sub;
        ros::Publisher setpoint_raw_local_pub;
        ros::ServiceClient mode_client;
        ros::ServiceClient arming_client;
        tf::Quaternion quat;
        px4_cmd::Command controller_cmd;
        mavros_msgs::State current_state;
        mavros_msgs::PositionTarget pos_setpoint;
        mavros_msgs::SetMode mode_cmd;
        mavros_msgs::CommandBool arm_cmd;
        geometry_msgs::PoseStamped current_pos;
        double R;
        double P;
        double Y;
        double init_x;
        double init_y;
        double init_z;
        double init_R;
        double init_P;
        double init_Y;
        void controller_cmd_cb(const px4_cmd::Command::ConstPtr &msg);
        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void state_cb(const mavros_msgs::State::ConstPtr &msg);
        void ros_thread_fun();

    public:
        QVector<double> x;
        QVector<double> y;
        QVector<double> z;
        QVector<double> vx;
        QVector<double> vy;
        QVector<double> vz;
        QVector<double> pitch;
        QVector<double> roll;
        QVector<double> yaw;
        QStringList sensor_topics;
        std::thread *run_thread;
        string state_mode;
        string node_name;
        string vehicle_name;
        string sensor_name;
        bool thread_stop = false;
        void start(string node);
        string set_mode(string desire_mode);
};

void vehicle_command::start(string node)
{
    node_name = node;
    int argc = 0;
    char **argv;
    string topic_header = "/" + node_name + "/mavros/";
    ros::init(argc, argv, node_name + "_cmd");
    ros::NodeHandle nh;
    ros::param::get(("/" + node_name + "/vehicle").c_str(), vehicle_name);
    ros::param::get(("/" + node_name + "/sensor").c_str(), sensor_name);
    ros::param::get(("/" + node_name + "/init_x").c_str(), init_x);
    ros::param::get(("/" + node_name + "/init_y").c_str(), init_y);
    ros::param::get(("/" + node_name + "/init_z").c_str(), init_z);
    ros::param::get(("/" + node_name + "/init_R").c_str(), init_R);
    ros::param::get(("/" + node_name + "/init_P").c_str(), init_P);
    ros::param::get(("/" + node_name + "/init_Y").c_str(), init_Y);
    while (!ros::ok())
    {
        ros::Duration(update_time).sleep();
    }
    controller_cmd_sub = nh.subscribe<px4_cmd::Command>((node_name + "/px4_cmd/control_command").c_str(), 20, &vehicle_command::controller_cmd_cb, this);
    current_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>((topic_header + "local_position/pose").c_str(), 20, &vehicle_command::pos_cb, this);
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>((topic_header + "setpoint_raw/local").c_str(), 50);
    mode_client = nh.serviceClient<mavros_msgs::SetMode>((topic_header + "set_mode").c_str());
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>((topic_header + "cmd/arming").c_str());
    std::thread ros_thread(&vehicle_command::ros_thread_fun, this);
    ros_thread.detach();
    run_thread = &ros_thread;
}

string vehicle_command::set_mode(string desire_mode)
{
    int error_times = 0;
    string err_msg = "";
    // 解锁
    if (desire_mode == "Arm" || desire_mode == "DisArm")
    {
        // 如果飞行高度超过20cm则不允许DisArm
        if (abs(current_pos.pose.position.z) > 0.2 && desire_mode == "DisArm")
        {
            return "Vehicle is Flying, you can not DisArm!";
        }

        bool desire_arm_cmd = (desire_mode == "Arm") ? true : false;
        while ((current_state.armed != desire_arm_cmd) && error_times < 10)
        {
            arm_cmd.request.value = desire_arm_cmd;
            Info(desire_mode + "ing...");
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                // 执行回调函数
                ros::spinOnce();
                sleep(1);
                if (current_state.armed == desire_arm_cmd)
                {
                    Info(desire_mode + " Command Sent and " + desire_mode + " Successfully!");
                    return "";
                }
                else
                {
                    err_msg = desire_mode + " Command Sent but " + desire_mode + " Failed!";
                }
            }
            else
            {
                err_msg = desire_mode + " Command Sent Failed!";
            }
            error_times++;
        }
        if (error_times == 0)
        {
            err_msg = "Already " + desire_mode + "!";
        }
        return err_msg;
    }
    // 更改模式
    while (current_state.mode != desire_mode && error_times < 10)
    {
        // 处于OFFBOARD模式时，只能改为AUTO模式
        if (current_state.mode == "OFFBOARD")
        {
            if (desire_mode != "AUTO.LAND" && desire_mode != "AUTO.RTL")
            {
                return "You are in OFFBOARD Mode, you can only change to [Auto.Land] or [Auto.RTL] Mode!";
            }
        }
        // 请求更改模式服务
        mode_cmd.request.custom_mode = desire_mode;
        mode_client.call(mode_cmd);
        sleep(1);

        if (mode_cmd.response.mode_sent)
        {
            if (current_state.mode == desire_mode)
            {
                return "";
            }
            else
            {
                err_msg = desire_mode + " Mode Sent but Changed Failed!";
            }
        }
        else
        {
            err_msg = "Mode Sent Failed!";
        }
        error_times++;
    }
    return err_msg;
}

void vehicle_command::ros_thread_fun()
{
    while (ros::ok() && !thread_stop)
    {
        setpoint_raw_local_pub.publish(pos_setpoint);
        ros::Duration(update_time).sleep();
        ros::spinOnce();
    }
}

void vehicle_command::controller_cmd_cb(const px4_cmd::Command::ConstPtr &msg)
{
    controller_cmd = *msg;

    // 设定坐标系
    switch (controller_cmd.Move_frame)
    {
        case px4_cmd::Command::ENU:
        {
            pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            break;
        }

        case px4_cmd::Command::BODY:
        {
            pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            break;
        }
    }

    // 设定输入值
    // Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
    // Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
    // Bit 10 should set to 0, means is not force sp
    //
    if (controller_cmd.Vehicle == px4_cmd::Command::Multicopter)
    {
        if (controller_cmd.Mode == px4_cmd::Command::Hover)
        {
            pos_setpoint.type_mask = 0b100111111000;
            pos_setpoint.position.x = current_pos.pose.position.x;
            pos_setpoint.position.y = current_pos.pose.position.y;
            pos_setpoint.position.z = current_pos.pose.position.z;
            pos_setpoint.header.frame_id = 1;
            pos_setpoint.yaw = controller_cmd.yaw_cmd;
            return;
        }

        switch (controller_cmd.Move_mode)
        {
        case px4_cmd::Command::XYZ_POS:
        {
            pos_setpoint.type_mask = 0b100111111000;
            pos_setpoint.position.x = controller_cmd.desire_cmd[0];
            pos_setpoint.position.y = controller_cmd.desire_cmd[1];
            pos_setpoint.position.z = controller_cmd.desire_cmd[2];
            break;
        }

        case px4_cmd::Command::XY_VEL_Z_POS:
        {
            pos_setpoint.type_mask = 0b100111100011;
            pos_setpoint.velocity.x = controller_cmd.desire_cmd[0];
            pos_setpoint.velocity.y = controller_cmd.desire_cmd[1];
            pos_setpoint.position.z = controller_cmd.desire_cmd[2];
            break;
        }

        case px4_cmd::Command::XYZ_VEL:
        {
            pos_setpoint.type_mask = 0b100111000111;
            pos_setpoint.velocity.x = controller_cmd.desire_cmd[0];
            pos_setpoint.velocity.y = controller_cmd.desire_cmd[1];
            pos_setpoint.velocity.z = controller_cmd.desire_cmd[2];
            break;
        }

        case px4_cmd::Command::XYZ_REL_POS:
        {
            pos_setpoint.type_mask = 0b100111111000;
            pos_setpoint.position.x = controller_cmd.desire_cmd[0];
            pos_setpoint.position.y = controller_cmd.desire_cmd[1];
            pos_setpoint.position.z = controller_cmd.desire_cmd[2];
            break;
        }
        }

        pos_setpoint.header.frame_id = 1;

        pos_setpoint.yaw = controller_cmd.yaw_cmd;
    }
    // 固定翼信息
    if (controller_cmd.Vehicle == px4_cmd::Command::FixWing)
    {
        if (controller_cmd.Mode == px4_cmd::Command::Loiter)
        {
            pos_setpoint.type_mask = 12288;
            pos_setpoint.position.x = current_pos.pose.position.x;
            pos_setpoint.position.y = current_pos.pose.position.y;
            pos_setpoint.position.z = current_pos.pose.position.z;
            pos_setpoint.header.frame_id = 1;
            return;
        }

        switch (controller_cmd.Move_mode)
        {
        case px4_cmd::Command::FixWing_Takeoff:
        {
            pos_setpoint.type_mask = 4096;
            break;
        }

        case px4_cmd::Command::FixWing_Gliding:
        {
            pos_setpoint.type_mask = 0b000100100100;
            break;
        }

        case px4_cmd::Command::FixWing_POS:
        {
            pos_setpoint.type_mask = 12288;
            break;
        }

        case px4_cmd::Command::FixWing_REL_POS:
        {
            pos_setpoint.type_mask = 12288;
            break;
        }
        }
        pos_setpoint.position.x = controller_cmd.desire_cmd[0];
        pos_setpoint.position.y = controller_cmd.desire_cmd[1];
        pos_setpoint.position.z = controller_cmd.desire_cmd[2];
        pos_setpoint.header.frame_id = 1;
    }
}

// 订阅回调返回状态信息
void vehicle_command::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

// 订阅回调返回位置信息
void vehicle_command::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pos = *msg;
}

#endif