// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <ros/ros.h>

#include <iostream>
#include <string>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <px4_cmd/Command.h>

#include <print_utility/printf_utility.h>
#include <print_utility/handle_cin.h>

#include <px4_cmd/custom_command.h>

using namespace std;

// 初始化信息用于接受设置命令,发送指定位置信息
ros::NodeHandle nh;
px4_cmd::Command set_cmd;
mavros_msgs::PositionTarget pos_setpoint;
mavros_msgs::GlobalPositionTarget pos_setpoint_global;
mavros_msgs::AttitudeTarget attitude_setpoint;
mavros_msgs::SetMode mode_cmd;
geometry_msgs::PoseStamped current_pos;
string state_mode;
bool arm_state;
string node_name = "";

// 初始化外部自定义指令
int current_custom_mode = 0;
CustomCommand cmd;
std::string error;

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

    // 获取 robot 名
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    for (auto topic = topics.begin(); topic != topics.end(); topic++)
    {
        auto position = topic->name.find("/mavros");
        if (position != std::string::npos)
        {
            if (position != 0)
            {
                node_name = topic->name.substr(0, position);
            }
            break;
        }
    }

    // 广播和节点
    set_cmd_sub = nh.subscribe<px4_cmd::Command>(node_name + "/px4_cmd/control_command", 10, sub_set_cmd_cb);
    current_state_sub = nh.subscribe<mavros_msgs::State>(node_name + "/mavros/state", 20, state_cb);
    current_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(node_name + "/mavros/local_position/pose", 10, pos_cb);
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>(node_name + "/mavros/setpoint_raw/local", 10);

    // 服务
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>(node_name + "/mavros/set_mode");

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

    // custom command
    while (state_mode == mavros_msgs::State::MODE_PX4_OFFBOARD && msg->Move_mode == px4_cmd::Command::Custom_Command)
    {
        // convert origin message to custom command
        px4_msg_to_custom_command(set_cmd, cmd);

        // check command
        if (!check_custom_command(cmd, set_cmd.Vehicle, error))
        {
            // if not pass check, set mode to hover
            if (set_cmd.Vehicle == px4_cmd::Command::FixWing)
            {
                set_cmd.Mode = px4_cmd::Command::Loiter;
            }
            else
            {
                set_cmd.Mode = px4_cmd::Command::Hover;
            }
            std::cout << error << std::endl;
            break;
        }

        // get frame
        int frame;
        switch (set_cmd.Move_frame)
        {
            case px4_cmd::Command::ENU:
            {
                frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                break;
            }

            case px4_cmd::Command::BODY:
            {
                frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
                break;
            }

            case px4_cmd::Command::GLOBAL:
            {
                frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
            }
        }

        // generate message
        switch (cmd.mode)
        {
            case CommandMode::TargetLocal:
            {
                // frame
                pos_setpoint.coordinate_frame = frame;
                pos_setpoint.header.frame_id = frame;

                // position
                pos_setpoint.position.x = cmd.position[0];
                pos_setpoint.position.y = cmd.position[1];
                pos_setpoint.position.z = cmd.position[2];
                pos_setpoint.type_mask += isnan(cmd.position[0]) ? mavros_msgs::PositionTarget::IGNORE_PX : 0;
                pos_setpoint.type_mask += isnan(cmd.position[1]) ? mavros_msgs::PositionTarget::IGNORE_PY : 0;
                pos_setpoint.type_mask += isnan(cmd.position[2]) ? mavros_msgs::PositionTarget::IGNORE_PZ : 0;

                // fixwing only support position setpoint
                if (set_cmd.Vehicle == px4_cmd::Command::FixWing)
                {
                    pos_setpoint.type_mask = cmd.fw_mode;
                    break;
                }

                // velocity
                pos_setpoint.velocity.x = cmd.velocity[0];
                pos_setpoint.velocity.y = cmd.velocity[1];
                pos_setpoint.velocity.z = cmd.velocity[2];
                pos_setpoint.type_mask += isnan(cmd.velocity[0]) ? mavros_msgs::PositionTarget::IGNORE_VX : 0;
                pos_setpoint.type_mask += isnan(cmd.velocity[1]) ? mavros_msgs::PositionTarget::IGNORE_VY : 0;
                pos_setpoint.type_mask += isnan(cmd.velocity[2]) ? mavros_msgs::PositionTarget::IGNORE_VZ : 0;

                // accelerate
                pos_setpoint.acceleration_or_force.x = cmd.accelerate[0];
                pos_setpoint.acceleration_or_force.y = cmd.accelerate[1];
                pos_setpoint.acceleration_or_force.z = cmd.accelerate[2];
                pos_setpoint.type_mask += isnan(cmd.accelerate[0]) ? mavros_msgs::PositionTarget::IGNORE_AFX : 0;
                pos_setpoint.type_mask += isnan(cmd.accelerate[1]) ? mavros_msgs::PositionTarget::IGNORE_AFY : 0;
                pos_setpoint.type_mask += isnan(cmd.accelerate[2]) ? mavros_msgs::PositionTarget::IGNORE_AFZ : 0;

                // Force flag
                pos_setpoint.type_mask += cmd.force_flag ? mavros_msgs::PositionTarget::FORCE : 0;

                // yaw and yaw rate
                pos_setpoint.yaw = cmd.yaw;
                pos_setpoint.yaw_rate = cmd.yaw_rate;
                pos_setpoint.type_mask += isnan(cmd.yaw) ? mavros_msgs::PositionTarget::IGNORE_YAW : 0;
                pos_setpoint.type_mask += isnan(cmd.yaw_rate) ? mavros_msgs::PositionTarget::IGNORE_YAW_RATE : 0;
                break;
            }

            case CommandMode::TargetGlobal:
            {
                // frame
                pos_setpoint_global.coordinate_frame = frame;
                pos_setpoint_global.header.frame_id = frame;

                // position
                pos_setpoint_global.latitude = cmd.position[0];
                pos_setpoint_global.longitude = cmd.position[1];
                pos_setpoint_global.altitude = cmd.position[2];
                pos_setpoint_global.type_mask += isnan(cmd.position[0]) ? mavros_msgs::GlobalPositionTarget::IGNORE_LATITUDE : 0;
                pos_setpoint_global.type_mask += isnan(cmd.position[1]) ? mavros_msgs::GlobalPositionTarget::IGNORE_LONGITUDE : 0;
                pos_setpoint_global.type_mask += isnan(cmd.position[2]) ? mavros_msgs::GlobalPositionTarget::IGNORE_ALTITUDE : 0;

                // fixwing only support position setpoint
                if (set_cmd.Vehicle == px4_cmd::Command::FixWing)
                {
                    pos_setpoint_global.type_mask = cmd.fw_mode;
                    break;
                }

                // velocity
                pos_setpoint_global.velocity.x = cmd.velocity[0];
                pos_setpoint_global.velocity.y = cmd.velocity[1];
                pos_setpoint_global.velocity.z = cmd.velocity[2];
                pos_setpoint_global.type_mask += isnan(cmd.velocity[0]) ? mavros_msgs::GlobalPositionTarget::IGNORE_VX : 0;
                pos_setpoint_global.type_mask += isnan(cmd.velocity[1]) ? mavros_msgs::GlobalPositionTarget::IGNORE_VY : 0;
                pos_setpoint_global.type_mask += isnan(cmd.velocity[2]) ? mavros_msgs::GlobalPositionTarget::IGNORE_VZ : 0;

                // accelerate
                pos_setpoint_global.acceleration_or_force.x = cmd.accelerate[0];
                pos_setpoint_global.acceleration_or_force.y = cmd.accelerate[1];
                pos_setpoint_global.acceleration_or_force.z = cmd.accelerate[2];
                pos_setpoint_global.type_mask += isnan(cmd.accelerate[0]) ? mavros_msgs::GlobalPositionTarget::IGNORE_AFX : 0;
                pos_setpoint_global.type_mask += isnan(cmd.accelerate[1]) ? mavros_msgs::GlobalPositionTarget::IGNORE_AFY : 0;
                pos_setpoint_global.type_mask += isnan(cmd.accelerate[2]) ? mavros_msgs::GlobalPositionTarget::IGNORE_AFZ : 0;

                // Force flag
                pos_setpoint_global.type_mask += cmd.force_flag ? mavros_msgs::GlobalPositionTarget::FORCE : 0;

                // yaw and yaw rate
                pos_setpoint_global.yaw = cmd.yaw;
                pos_setpoint_global.yaw_rate = cmd.yaw_rate;
                pos_setpoint_global.type_mask += isnan(cmd.yaw) ? mavros_msgs::GlobalPositionTarget::IGNORE_YAW : 0;
                pos_setpoint_global.type_mask += isnan(cmd.yaw_rate) ? mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE : 0;
                break;
            }

            case CommandMode::TargetAttitude:
            {
                // attitude
                attitude_setpoint.orientation.x = cmd.attitude[0];
                attitude_setpoint.orientation.y = cmd.attitude[1];
                attitude_setpoint.orientation.z = cmd.attitude[2];
                attitude_setpoint.orientation.w = cmd.attitude[3];
                attitude_setpoint.type_mask += isnan(cmd.attitude[0]) ? mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE : 0;

                // attitude rate
                attitude_setpoint.body_rate.x = cmd.attitude_rate[0];
                attitude_setpoint.body_rate.y = cmd.attitude_rate[1];
                attitude_setpoint.body_rate.z = cmd.attitude_rate[2];
                attitude_setpoint.type_mask += isnan(cmd.attitude_rate[0]) ? mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE : 0;
                attitude_setpoint.type_mask += isnan(cmd.attitude_rate[1]) ? mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE : 0;
                attitude_setpoint.type_mask += isnan(cmd.attitude_rate[2]) ? mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE : 0;

                // thrust
                attitude_setpoint.thrust = cmd.thrust;
                attitude_setpoint.type_mask += isnan(cmd.thrust) ? mavros_msgs::AttitudeTarget::IGNORE_THRUST : 0;
                break;
            }
        }

        // update publisher
        if (current_custom_mode != cmd.mode)
        {
            current_custom_mode = cmd.mode;
            setpoint_raw_local_pub.shutdown();
            switch (cmd.mode)
            {
                case CommandMode::TargetLocal:
                {
                    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>((node_name +"mavros/setpoint_raw/local").c_str(), 50);
                    break;
                }

                case CommandMode::TargetGlobal:
                {
                    setpoint_raw_local_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>((node_name +"mavros/setpoint_raw/global").c_str(), 50);
                    break;
                }

                case CommandMode::TargetAttitude:
                {
                    setpoint_raw_local_pub = nh.advertise<mavros_msgs::AttitudeTarget>((node_name +"mavros/setpoint_raw/attitude").c_str(), 50);
                    break;
                }
            }
        }

        // send command
        switch (cmd.mode)
        {
            case CommandMode::TargetLocal:
            {
                setpoint_raw_local_pub.publish(pos_setpoint);
                break;
            }

            case CommandMode::TargetGlobal:
            {
                setpoint_raw_local_pub.publish(pos_setpoint_global);
                break;
            }

            case CommandMode::TargetAttitude:
            {
                setpoint_raw_local_pub.publish(attitude_setpoint);
                break;
            }
        }

        // custom command end
        return;
    }

    // non-custom command mode be local mode
    if (current_custom_mode != CommandMode::TargetLocal)
    {
        setpoint_raw_local_pub.shutdown();
        setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>((node_name + "/mavros/setpoint_raw/local").c_str(), 50);
    }
    current_custom_mode = CommandMode::TargetLocal;

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