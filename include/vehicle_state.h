// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef VEHICEL_STATE_H
#define VEHICEL_STATE_H
#include <ros/ros.h>
#include <string>
#include <QStringList>
#include <QVector>
#include <QThread>
#include <thread>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#define PI 3.14159265358979323846

using namespace std;

class vehicle_state
{
    private:
        // setting
        ros::Subscriber pos_sub;
        ros::Subscriber vel_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber state_sub;
        tf::Quaternion quat;
        double R;
        double P;
        double Y;
        double init_x;
        double init_y;
        double init_z;
        double init_R;
        double init_P;
        double init_Y;
        double t_count;
        void get_sensor_topic();
        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void state_cb(const mavros_msgs::State::ConstPtr &msg);
        void ros_thread_fun();

    public:
        double update_time = 0.02;
        QVector<double> t;
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
        string state_mode;
        string node_name;
        string vehicle_name;
        string sensor_name;
        bool ros_stop = false;
        bool thread_stop = false;
        void get_state(string node);
};

void vehicle_state::get_state(string node)
{
    node_name = node;
    t_count = 0;
    int argc = 0;
    char **argv;
    string topic_header = "/" + node_name + "/mavros/";
    ros::init(argc, argv, "px4_cmd/" + node_name + "_state");
    ros::NodeHandle nh;
    ros::param::get(("/" + node_name + "/vehicle").c_str(), vehicle_name);
    ros::param::get(("/" + node_name + "/sensor").c_str(), sensor_name);
    ros::param::get(("/" + node_name + "/init_x").c_str(), init_x);
    ros::param::get(("/" + node_name + "/init_y").c_str(), init_y);
    ros::param::get(("/" + node_name + "/init_z").c_str(), init_z);
    ros::param::get(("/" + node_name + "/init_R").c_str(), init_R);
    ros::param::get(("/" + node_name + "/init_P").c_str(), init_P);
    ros::param::get(("/" + node_name + "/init_Y").c_str(), init_Y);
    get_sensor_topic();
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>((topic_header + "local_position/pose").c_str(), 20, &vehicle_state::pos_cb, this);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>((topic_header + "local_position/velocity_local").c_str(), 20, &vehicle_state::vel_cb, this);
    state_sub = nh.subscribe<mavros_msgs::State>((topic_header + "state").c_str(), 20, &vehicle_state::state_cb, this);
    while (!ros::ok())
    {
        usleep(floor(1000000 * update_time));
    }
    std::thread ros_thread(&vehicle_state::ros_thread_fun, this);
    ros_thread.detach();
}

void vehicle_state::ros_thread_fun()
{
    sleep(1);
    while (ros::ok() && !thread_stop && pos_sub.getNumPublishers() > 0)
    {
        usleep(floor(1000000 * update_time));
        ros::spinOnce();
    }
    ros::shutdown();
    ros_stop = true;
}

void vehicle_state::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    // 将旋转矩阵转换为欧拉角
    tf::Matrix3x3(quat).getRPY(R, P, Y);
    t.push_back(t_count);
    x.push_back(msg->pose.position.x + init_x);
    y.push_back(msg->pose.position.y + init_y);
    z.push_back(msg->pose.position.z + init_z);
    pitch.push_back(P + init_P);
    roll.push_back(R + init_R);
    yaw.push_back(Y + init_Y);
    t_count++;
}

void vehicle_state::vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vx.push_back(msg->twist.linear.x);
    vy.push_back(msg->twist.linear.y);
    vz.push_back(msg->twist.linear.z);
}

void vehicle_state::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    state_mode = msg->mode;
}

void vehicle_state::get_sensor_topic()
{
    QString node = node_name.c_str();
    QString sensor_type = sensor_name.c_str();
    sensor_type = sensor_type.toLower();
    if (sensor_type.toStdString().find("camera") == std::string::npos)
    {
        sensor_topics.push_back("None");
        return;
    }
    sensor_type = sensor_type.split(" ")[0];
    sensor_type = "/" + node + "/" + sensor_type + "_cam";
    if (sensor_type.toStdString().find("stereo") != std::string::npos)
    {
        sensor_topics.push_back(sensor_type + "/left/image_raw");
        sensor_topics.push_back(sensor_type + "/right/image_raw");
    }
    else
    {
        sensor_topics.push_back(sensor_type + "/image_raw");
        if (sensor_type.toStdString().find("depth") != std::string::npos)
        {
            sensor_topics.push_back(sensor_type + "/depth/image_raw");
        }
    }
}

#endif