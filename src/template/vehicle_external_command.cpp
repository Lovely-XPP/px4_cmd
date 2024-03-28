// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <px4_cmd/vehicle_external_command.hpp>

#define PI 3.14159265358979323846
using namespace std;

void vehicle_external_command::start(string node)
{
    string node_name = node;
    string topic_header = "/" + node_name + "/mavros/";
    // handle null string
    if (node_name == "")
    {
        topic_header = "/mavros/";
    }
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    int argc = 0;
    char **argv;
    ros::init(argc, argv, "ext_cmd");
    ros::NodeHandle nh("/" + node_name, "px4_cmd");
    ros::param::get(("/" + node_name + "/init_x").c_str(), init_x);
    ros::param::get(("/" + node_name + "/init_y").c_str(), init_y);
    ros::param::get(("/" + node_name + "/init_z").c_str(), init_z);
    ros::param::get(("/" + node_name + "/init_R").c_str(), init_R);
    ros::param::get(("/" + node_name + "/init_P").c_str(), init_P);
    ros::param::get(("/" + node_name + "/init_Y").c_str(), init_Y);
    pos_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_header + "local_position/pose", 20, &vehicle_external_command::pos_cb, this);
    vel_angle_rate_sub = nh.subscribe<geometry_msgs::TwistStamped>(topic_header + "local_position/velocity_local", 20, &vehicle_external_command::vel_cb, this);
    ext_cmd_pub = nh.advertise<px4_cmd::Command>("external_command", 50);
    ext_state_sub = nh.subscribe<std_msgs::Bool>("/" + node_name + "px4_cmd/ext_cmd_state", 20, &vehicle_external_command::ext_state_cb, this);
    while (!ros::ok())
    {
        usleep(floor(1000000 * update_time));
    }
    std::thread ros_thread(&vehicle_external_command::ros_thread_fun, this);
    ros_thread.detach();
};

void vehicle_external_command::start()
{
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    string node_name = "";
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
    start(node_name);
};

void vehicle_external_command::ros_thread_fun()
{
    // time counter
    double t = 0;
    // ros main loop
    while (ros::ok() && !ros_shutdown_flag)
    {
        while (ext_cmd_pub.getNumSubscribers() < 1)
        {
            ROS_INFO("External Command: Waiting for user-define mode!");
            sleep(1);
            t = 0;
        }
        external_cmd.ext_time = t;
        external_cmd.ext_total_time = total_time;
        ext_cmd_pub.publish(external_cmd);
        t = t + update_time;
        usleep(floor(1000000 * update_time));
        ros::spinOnce();
    }
    // exit with shutdown
    ros::shutdown();
};

void vehicle_external_command::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    position[0] = msg->pose.position.x + init_x;
    position[1] = msg->pose.position.y + init_y;
    position[2] = msg->pose.position.z + init_z;
    double R;
    double P;
    double Y;
    tf::quaternionMsgToTF(msg->pose.orientation, quaternion);
    tf::Matrix3x3(quaternion).getRPY(R, P, Y);
    attitude[0] = P + init_P;
    attitude[1] = R + init_R;
    attitude[2] = Y + init_Y;
};

void vehicle_external_command::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    velocity[0] = msg->twist.linear.x;
    velocity[1] = msg->twist.linear.y;
    velocity[2] = msg->twist.linear.z;
    angle_rate[0] = msg->twist.angular.x * 180 / PI;
    angle_rate[1] = msg->twist.angular.y * 180 / PI;
    angle_rate[2] = msg->twist.angular.z * 180 / PI;
};

void vehicle_external_command::ext_state_cb(const std_msgs::Bool::ConstPtr &msg)
{
    ext_cmd_state = msg->data;
};

void vehicle_external_command::set_position(double x, double y, double z, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    external_cmd.desire_cmd[0] = x;
    external_cmd.desire_cmd[1] = y;
    external_cmd.desire_cmd[2] = z;
};

void vehicle_external_command::set_position(double x, double y, double z, double yaw, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    external_cmd.desire_cmd[0] = x;
    external_cmd.desire_cmd[1] = y;
    external_cmd.desire_cmd[2] = z;
    external_cmd.yaw_cmd = yaw - init_Y;
};

void vehicle_external_command::set_velocity(double vx, double vy, double vz, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_VEL;
    external_cmd.desire_cmd[0] = vx;
    external_cmd.desire_cmd[1] = vy;
    external_cmd.desire_cmd[2] = vz;
};

void vehicle_external_command::set_velocity(double vx, double vy, double vz, double yaw, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_VEL;
    external_cmd.desire_cmd[0] = vx;
    external_cmd.desire_cmd[1] = vy;
    external_cmd.desire_cmd[2] = vz;
    external_cmd.yaw_cmd = yaw - init_Y;
};

void vehicle_external_command::set_velocity_with_height(double vx, double vy, double z, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XY_VEL_Z_POS;
    external_cmd.desire_cmd[0] = vx;
    external_cmd.desire_cmd[1] = vy;
    external_cmd.desire_cmd[2] = z;
};

void vehicle_external_command::set_velocity_with_height(double vx, double vy, double z, double yaw, int frame)
{
    external_cmd.Mode = px4_cmd::Command::Move;
    external_cmd.Move_frame = frame;
    external_cmd.Move_mode = px4_cmd::Command::XY_VEL_Z_POS;
    external_cmd.desire_cmd[0] = vx;
    external_cmd.desire_cmd[1] = vy;
    external_cmd.desire_cmd[2] = z;
    external_cmd.yaw_cmd = yaw - init_Y;
};

void vehicle_external_command::set_custom_command(const CustomCommand cmd)
{
    custom_command_to_px4_msg(cmd, external_cmd);
}

void vehicle_external_command::set_hover()
{
    external_cmd.Mode = px4_cmd::Command::Hover;
    external_cmd.Move_frame = px4_cmd::Command::ENU;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
};

void vehicle_external_command::set_hover(double yaw)
{
    external_cmd.Mode = px4_cmd::Command::Hover;
    external_cmd.Move_frame = px4_cmd::Command::ENU;
    external_cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    external_cmd.yaw_cmd = yaw;
};

void vehicle_external_command::shutdown()
{
    ros_shutdown_flag = true;
};