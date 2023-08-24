#include <ros/ros.h>
#include <string>
#include <QVector>
#include <thread>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.14159265358979323846

class vehicle
{
private:
    ros::Rate cmd_rate(10.0);
    ros::Subscriber pos_sub;
    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

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
    vehicle(string node_name);
};

vehicle::vehicle(string node_name)
{
    ros::init(argc, argv, node_name + "_info");
    ros::NodeHandle nh;
    state_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, state_cb);
}

vehicle::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    x.push_back(*msg->x);
    y.push_back(*msg->y);
}