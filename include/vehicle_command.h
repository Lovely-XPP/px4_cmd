// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef VEHICEL_COMMAND_H
#define VEHICEL_COMMAND_H
#include <ros/ros.h>
#include <string>
#include <cmath>
#include <QStringList>
#include <QVector>
#include <QThread>
#include <thread>
#include <mutex>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_cmd/Command.h>
#include <px4_cmd/custom_command.h>

#define PI 3.14159265358979323846

using namespace std;

class vehicle_command
{
    private:
        // setting
        ros::NodeHandle nh;
        ros::Subscriber controller_cmd_sub;
        ros::Subscriber current_pos_sub;
        ros::Subscriber current_state_sub;
        ros::Subscriber current_extend_state_sub;
        ros::Subscriber ext_cmd_sub;
        ros::Publisher setpoint_raw_pub;
        ros::Publisher ext_cmd_state_pub;
        ros::ServiceClient mode_client;
        ros::ServiceClient arming_client;
        tf::Quaternion quat;
        mavros_msgs::State current_state;
        mavros_msgs::PositionTarget pos_setpoint;
        mavros_msgs::GlobalPositionTarget pos_setpoint_global;
        mavros_msgs::AttitudeTarget attitude_setpoint;
        mavros_msgs::SetMode mode_cmd;
        mavros_msgs::CommandBool arm_cmd;
        std_msgs::Bool ext_cmd_state_msg;
        std::mutex cmd_mutex;
        double R;
        double P;
        double Y;
        double init_R;
        double init_P;
        double init_Y;
        int desire_time = 0;
        bool hover = false;
        int current_custom_mode = 0;
        std::string topic_header;

        /// @brief controller command subscribe call back function
        /// @param msg controller command message 
        void controller_cmd_cb(const px4_cmd::Command::ConstPtr &msg);

        /// @brief vehicle pose subscribe call back function
        /// @param msg controller command message
        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

        /// @brief vehicle state subscribe call back function
        /// @param msg controller command message 
        void state_cb(const mavros_msgs::State::ConstPtr &msg);

        /// @brief external command subscribe call back function
        /// @param msg external command message 
        void ext_cmd_cb(const px4_cmd::Command::ConstPtr &msg);

        /// @brief vehicle extended state subscribe call back function
        /// @param msg mavros extended state message
        void extend_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg);

        /// @brief ROS node thread function 
        void ros_thread_fun();

    public:
        double update_time = 0.02;
        std::thread *run_thread;
        string state_mode;
        string node_name;
        string vehicle_name;
        int vehicle_type = px4_cmd::Command::Multicopter;
        px4_cmd::Command controller_cmd;
        px4_cmd::Command ext_cmd;
        bool arm_state = false;
        bool land_state = true;
        bool ext_cmd_pub_state = false;
        bool ext_cmd_sub_state = false;
        bool thread_stop = false;
        bool ros_stop = false;
        bool achieve_desire = false;
        double sigma = 0.1;
        double x;
        double y;
        double z;
        double init_x;
        double init_y;
        double init_z;
        vector<double> home_position = {0, 0};
        vector<double> hover_pos = {0, 0, 0};

        /// @brief start vehicle control
        /// @param node node name for vehicle
        void start(string node);

        /// @brief set desire mode for vehicle and return execute state
        /// @param desire_mode desire mode for vehicle
        /// @return error string, if success, return empty string
        string set_mode(string desire_mode);
};

void vehicle_command::start(string node)
{
    node_name = node;
    pos_setpoint.header.frame_id = 1;
    pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    int argc = 0;
    char **argv;
    topic_header = "/" + node_name + "/mavros/";
    ros::init(argc, argv, "px4_cmd/" + node_name + "_cmd");
    ros::param::get(("/" + node_name + "/vehicle").c_str(), vehicle_name);
    ros::param::get(("/" + node_name + "/init_x").c_str(), init_x);
    ros::param::get(("/" + node_name + "/init_y").c_str(), init_y);
    ros::param::get(("/" + node_name + "/init_z").c_str(), init_z);
    ros::param::get(("/" + node_name + "/init_R").c_str(), init_R);
    ros::param::get(("/" + node_name + "/init_P").c_str(), init_P);
    ros::param::get(("/" + node_name + "/init_Y").c_str(), init_Y);
    if (vehicle_name == "plane")
    {
        vehicle_type = px4_cmd::Command::FixWing;
    }

    while (!ros::ok())
    {
        usleep(floor(1000000 * update_time));
    }
    controller_cmd_sub = nh.subscribe<px4_cmd::Command>((node_name + "/px4_cmd/control_command").c_str(), 20, &vehicle_command::controller_cmd_cb, this);
    current_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>((topic_header + "local_position/pose").c_str(), 20, &vehicle_command::pos_cb, this);
    current_state_sub = nh.subscribe<mavros_msgs::State>((topic_header + "state").c_str(), 20, &vehicle_command::state_cb, this);
    ext_cmd_sub = nh.subscribe<px4_cmd::Command>((node_name + "/px4_cmd/external_command").c_str(), 50, &vehicle_command::ext_cmd_cb, this);
    current_extend_state_sub = nh.subscribe<mavros_msgs::ExtendedState>((topic_header + "extended_state").c_str(), 20, &vehicle_command::extend_state_cb, this);
    setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>((topic_header + "setpoint_raw/local").c_str(), 50);
    ext_cmd_state_pub = nh.advertise<std_msgs::Bool>("/" + node_name + "/px4_cmd/ext_cmd_state", 20);
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
        if (!land_state && desire_mode == "DisArm")
        {
            return "Vehicle is Flying, you can not DisArm!";
        }

        bool desire_arm_cmd = (desire_mode == "Arm") ? true : false;
        while (error_times < 10)
        {
            ros::spinOnce();
            usleep(200000);
            if (current_state.armed == desire_arm_cmd)
            {
                return "";
            }
            arm_cmd.request.value = desire_arm_cmd;
            if (arming_client.call(arm_cmd))
            {
                // 执行回调函数
                ros::spinOnce();
                usleep(200000);
                if (current_state.armed == desire_arm_cmd)
                {
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
        err_msg = "";
        // 处于OFFBOARD模式时，只能改为AUTO模式
        if (current_state.mode == "OFFBOARD")
        {
            if (desire_mode != "AUTO.LAND" && desire_mode != "AUTO.RTL")
            {
                return "In OFFBOARD Mode, you can only change to [Auto.Land] or [Auto.RTL] Mode!";
            }
        }

        // 请求更改模式服务
        mode_cmd.request.custom_mode = desire_mode;
        mode_client.call(mode_cmd);
        ros::spinOnce();
        usleep(200000);
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
    if (current_state.mode == desire_mode)
    {
        return "Current Mode is already [" + desire_mode + "]";
    }
    return err_msg;
}

void vehicle_command::ros_thread_fun()
{
    sleep(1);
    ros::spinOnce();
    bool sub_state = true;
    while (ros::ok() && !thread_stop && sub_state)
    {
        if (current_pos_sub.getNumPublishers() > 0)
        {
            sub_state = true;
        }
        else
        {
            sub_state = false;
        }
        if (ext_cmd_sub.getNumPublishers() > 0)
        {
            ext_cmd_pub_state = true;
        }
        else
        {
            ext_cmd_pub_state = false;
        }
        setpoint_raw_pub.publish(pos_setpoint);
        ext_cmd_state_msg.data = ext_cmd_sub_state;
        ext_cmd_state_pub.publish(ext_cmd_state_msg);
        usleep(floor(1000000 * update_time));
        ros::spinOnce();
    }
    ros::shutdown();
    ros_stop = true;
}

void vehicle_command::controller_cmd_cb(const px4_cmd::Command::ConstPtr &msg)
{
    controller_cmd = *msg;
    std::string error;
    CustomCommand cmd;
    cmd_mutex.lock();

    // custom command
    while (state_mode == mavros_msgs::State::MODE_PX4_OFFBOARD && msg->Move_mode == px4_cmd::Command::Custom_Command)
    {
        // set hover to false
        hover = false;
        
        // convert origin message to custom command
        px4_msg_to_custom_command(controller_cmd, cmd);

        // check command
        if (!check_custom_command(cmd, vehicle_type ,error))
        {
            // if not pass check, set mode to hover
            if (controller_cmd.Vehicle == px4_cmd::Command::FixWing)
            {
                controller_cmd.Mode = px4_cmd::Command::Loiter;
            }
            else
            {
                controller_cmd.Mode = px4_cmd::Command::Hover;
            }
            std::cout << error << std::endl;
            break;
        }

        // get frame
        int frame;
        switch (controller_cmd.Move_frame)
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
                // init typemask
                pos_setpoint.type_mask = 0;

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
                if (controller_cmd.Vehicle == px4_cmd::Command::FixWing)
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
                // init typemask
                pos_setpoint_global.type_mask = 0;

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
                if (controller_cmd.Vehicle == px4_cmd::Command::FixWing)
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
                // init typemask
                attitude_setpoint.type_mask = 0;

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
            setpoint_raw_pub.shutdown();
            switch (cmd.mode)
            {
                case CommandMode::TargetLocal:
                {
                    setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>((topic_header + "setpoint_raw/local").c_str(), 50);
                    break;
                }

                case CommandMode::TargetGlobal:
                {
                    setpoint_raw_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>((topic_header + "setpoint_raw/global").c_str(), 50);
                    break;
                }

                case CommandMode::TargetAttitude:
                {
                    setpoint_raw_pub = nh.advertise<mavros_msgs::AttitudeTarget>((topic_header + "setpoint_raw/attitude").c_str(), 50);
                    break;
                }
            }
        }

        // send command
        switch (cmd.mode)
        {
            case CommandMode::TargetLocal:
            {
                setpoint_raw_pub.publish(pos_setpoint);
                break;
            }

            case CommandMode::TargetGlobal:
            {
                setpoint_raw_pub.publish(pos_setpoint_global);
                break;
            }

            case CommandMode::TargetAttitude:
            {
                setpoint_raw_pub.publish(attitude_setpoint);
                break;
            }
        }

        // custom command end
        cmd_mutex.unlock();
        return;
    }
    
    // non-custom command mode be local mode
    if (current_custom_mode != CommandMode::TargetLocal)
    {
        setpoint_raw_pub.shutdown();
        setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>((topic_header + "setpoint_raw/local").c_str(), 50);
    }
    current_custom_mode = CommandMode::TargetLocal;

    // judge if achieve desire cmd
    double dx = x - controller_cmd.desire_cmd[0];
    double dy = y - controller_cmd.desire_cmd[1];
    double dz = z - controller_cmd.desire_cmd[2];
    if (msg->Mode == px4_cmd::Command::Move && state_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
    {
        if (abs(dx) < sigma && abs(dy) < sigma && abs(dz) < sigma)
        {
            desire_time++;
        }
        else
        {
            achieve_desire = false;
            desire_time = 0;
        }
        if (desire_time >= 5)
        {
            achieve_desire = true;
        }
    }
    else
    {
        desire_time = 0;
    }
    // reset home position when land
    if (state_mode == mavros_msgs::State::MODE_PX4_LAND && !arm_state && (msg->Move_mode == px4_cmd::Command::XYZ_POS || msg->Move_mode == px4_cmd::Command::XYZ_REL_POS))
    {
        home_position[0] = msg->desire_cmd[0];
        home_position[1] = msg->desire_cmd[1];
    }
    // 设定坐标系
    switch (controller_cmd.Move_frame)
    {
        case px4_cmd::Command::ENU:
        {
            pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            pos_setpoint.header.frame_id = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            break;
        }

        case px4_cmd::Command::BODY:
        {
            pos_setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            pos_setpoint.header.frame_id = mavros_msgs::PositionTarget::FRAME_BODY_NED;
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
            if (!hover)
            {
                hover_pos[0] = x;
                hover_pos[1] = y;
                hover_pos[2] = z;
                hover = true;
            }
            pos_setpoint.type_mask = 0b100111111000;
            pos_setpoint.position.x = hover_pos[0];
            pos_setpoint.position.y = hover_pos[1];
            pos_setpoint.position.z = hover_pos[2];
            cmd_mutex.unlock();
            return;
        }
        hover = false;

        if (controller_cmd.Mode == px4_cmd::Command::Takeoff)
        {
            pos_setpoint.type_mask = 0b100111111000;
            pos_setpoint.position.x = home_position[0];
            pos_setpoint.position.y = home_position[1];
            pos_setpoint.position.z = controller_cmd.desire_cmd[2];
            pos_setpoint.yaw = 0;
            cmd_mutex.unlock();
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

        pos_setpoint.yaw = controller_cmd.yaw_cmd;
    }
    // 固定翼信息
    if (controller_cmd.Vehicle == px4_cmd::Command::FixWing)
    {
        if (controller_cmd.Mode == px4_cmd::Command::Loiter)
        {
            if (!hover)
            {
                hover_pos[0] = x;
                hover_pos[1] = y;
                hover_pos[2] = z;
                hover = true;
            }
            pos_setpoint.type_mask = 12288;
            pos_setpoint.position.x = hover_pos[0];
            pos_setpoint.position.y = hover_pos[1];
            pos_setpoint.position.z = hover_pos[2];
            cmd_mutex.unlock();
            return;
        }
        hover = false;

        if (controller_cmd.Mode == px4_cmd::Command::Takeoff)
        {
            pos_setpoint.type_mask = 4096;
            pos_setpoint.position.x = home_position[0];
            pos_setpoint.position.y = home_position[1];
            pos_setpoint.position.z = controller_cmd.desire_cmd[2];
            cmd_mutex.unlock();
            return;
        }

        if (controller_cmd.Mode == px4_cmd::Command::Gliding)
        {
            pos_setpoint.type_mask = 0b000100100100;
        }

        switch (controller_cmd.Move_mode)
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
        pos_setpoint.position.x = controller_cmd.desire_cmd[0];
        pos_setpoint.position.y = controller_cmd.desire_cmd[1];
        pos_setpoint.position.z = controller_cmd.desire_cmd[2];
    }
    cmd_mutex.unlock();
}

void vehicle_command::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
    state_mode = msg->mode;
    arm_state = msg->armed;
}

void vehicle_command::extend_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
    if (msg->landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
    {
        land_state = true;
    }
    else
    {
        land_state = false;
    }
}

void vehicle_command::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;
}

void vehicle_command::ext_cmd_cb(const px4_cmd::Command::ConstPtr &msg)
{
    ext_cmd = *msg;
}
#endif