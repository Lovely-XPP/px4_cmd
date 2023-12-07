// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <ros/ros.h>

#include <iostream>
#include <string>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_cmd/Command.h>

#include <print_utility/printf_utility.h>
#include <print_utility/handle_cin.h>

using namespace std;

// 初始化信息用于接受设置命令,发送指定位置信息
px4_cmd::Command set_cmd;
mavros_msgs::PositionTarget pos_setpoint;
mavros_msgs::SetMode mode_cmd;
geometry_msgs::PoseStamped current_pos;
string state_mode;
bool arm_state;

// 初始化订阅和广播
ros::Subscriber set_cmd_sub;
ros::Subscriber current_pos_sub;
ros::Subscriber current_state_sub;
ros::Publisher setpoint_raw_local_pub;

// hover
bool hover;
double hover_pos[3] = {0};

// home
double home_position[2] = {0};

// 声明回调函数
void sub_set_cmd_cb(const px4_cmd::Command::ConstPtr &msg);
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg);

int main(int argc, char **argv)
{
    // 节点初始化
    ros::init(argc, argv, "send_command");
    ros::NodeHandle nh;

    // 广播和节点
    set_cmd_sub = nh.subscribe<px4_cmd::Command>("/px4_cmd/control_command", 10, sub_set_cmd_cb);
    current_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 20, state_cb);
    current_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 服务
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 等待节点初始化完成
    sleep(1);
    ros::Rate rate(50.0);

    // 输出标题（提示）
    int sys_res = system("clear");
    print_head("PX4 Command Sender");
    Info("PX4 Command Sender is Running...");

    // 主循环
    while (ros::ok())
    {
        setpoint_raw_local_pub.publish(pos_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

// 订阅回调函数,获取设置的指令信息
void sub_set_cmd_cb(const px4_cmd::Command::ConstPtr &msg)
{
    set_cmd = *msg;
    // reset home position when land
    if (state_mode == mavros_msgs::State::MODE_PX4_LAND && !arm_state && (msg->Move_mode == px4_cmd::Command::XYZ_POS || msg->Move_mode == px4_cmd::Command::XYZ_REL_POS))
    {
        home_position[0] = msg->desire_cmd[0];
        home_position[1] = msg->desire_cmd[1];
    }

    // 设定坐标系
    switch (set_cmd.Move_frame)
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
    pos_setpoint.header.frame_id = 1;
    if (set_cmd.Vehicle == px4_cmd::Command::Multicopter)
    {
        if (set_cmd.Mode == px4_cmd::Command::Hover)
        {
            if (!hover)
            {
                hover_pos[0] = current_pos.pose.position.x;
                hover_pos[1] = current_pos.pose.position.y;
                hover_pos[2] = current_pos.pose.position.z;
                hover = true;
            }
            pos_setpoint.type_mask = 0b100111111000;
            pos_setpoint.position.x = hover_pos[0];
            pos_setpoint.position.y = hover_pos[1];
            pos_setpoint.position.z = hover_pos[2];
            pos_setpoint.yaw = 0;
            return;
        }
        hover = false;

        if (set_cmd.Mode == px4_cmd::Command::Takeoff)
        {
            pos_setpoint.type_mask = 0b100111111000;
            pos_setpoint.position.x = home_position[0];
            pos_setpoint.position.y = home_position[1];
            pos_setpoint.position.z = set_cmd.desire_cmd[2];
            pos_setpoint.yaw = 0;
            return;
        }

        switch (set_cmd.Move_mode)
        {
            case px4_cmd::Command::XYZ_POS:
            {
                pos_setpoint.type_mask = 0b100111111000;
                pos_setpoint.position.x = set_cmd.desire_cmd[0];
                pos_setpoint.position.y = set_cmd.desire_cmd[1];
                pos_setpoint.position.z = set_cmd.desire_cmd[2];
                break;
            }

            case px4_cmd::Command::XY_VEL_Z_POS:
            {
                pos_setpoint.type_mask = 0b100111100011;
                pos_setpoint.velocity.x = set_cmd.desire_cmd[0];
                pos_setpoint.velocity.y = set_cmd.desire_cmd[1];
                pos_setpoint.position.z = set_cmd.desire_cmd[2];
                break;
            }

            case px4_cmd::Command::XYZ_VEL:
            {
                pos_setpoint.type_mask = 0b100111000111;
                pos_setpoint.velocity.x = set_cmd.desire_cmd[0];
                pos_setpoint.velocity.y = set_cmd.desire_cmd[1];
                pos_setpoint.velocity.z = set_cmd.desire_cmd[2];
                break;
            }

            case px4_cmd::Command::XYZ_REL_POS:
            {
                pos_setpoint.type_mask = 0b100111111000;
                pos_setpoint.position.x = set_cmd.desire_cmd[0];
                pos_setpoint.position.y = set_cmd.desire_cmd[1];
                pos_setpoint.position.z = set_cmd.desire_cmd[2];
                break;
            }
        }
        pos_setpoint.yaw = set_cmd.yaw_cmd;
    }
    // 固定翼信息
    if (set_cmd.Vehicle == px4_cmd::Command::FixWing)
    {
        if (set_cmd.Mode == px4_cmd::Command::Loiter)
        {
            if (!hover)
            {
                hover_pos[0] = current_pos.pose.position.x;
                hover_pos[1] = current_pos.pose.position.y;
                hover_pos[2] = current_pos.pose.position.z;
                hover = true;
            }
            pos_setpoint.type_mask = 12288;
            pos_setpoint.position.x = hover_pos[0];
            pos_setpoint.position.y = hover_pos[1];
            pos_setpoint.position.z = hover_pos[2];
            return;
        }
        hover = false;

        if (set_cmd.Mode == px4_cmd::Command::Takeoff)
        {
            pos_setpoint.type_mask = 4096;
            pos_setpoint.position.x = set_cmd.desire_cmd[0];
            pos_setpoint.position.y = set_cmd.desire_cmd[1];
            pos_setpoint.position.z = set_cmd.desire_cmd[2];
            return;
        }

        if (set_cmd.Mode == px4_cmd::Command::Gliding)
        {
            pos_setpoint.type_mask = 0b000100100100;
        }

        switch (set_cmd.Move_mode)
        {
            case px4_cmd::Command::XYZ_POS:
            {
                pos_setpoint.type_mask = 0b100111111000;
                break;
            }

            case px4_cmd::Command::XYZ_REL_POS:
            {
                pos_setpoint.type_mask = 0b100111111000;
                break;
            }
        }
        pos_setpoint.position.x = set_cmd.desire_cmd[0];
        pos_setpoint.position.y = set_cmd.desire_cmd[1];
        pos_setpoint.position.z = set_cmd.desire_cmd[2];
    }
}

// 订阅回调返回状态信息
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pos = *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    state_mode = msg->mode;
    arm_state = msg->armed;
}