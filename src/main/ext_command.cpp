#include <ros/ros.h>

#include <iostream>
#include <string>
#include <math.h>

#include <std_msgs/Bool.h>
#include <px4_cmd/Command.h>

#include <print_utility/printf_utility.h>
#include <print_utility/handle_cin.h>

using namespace std;

void ext_on_cb(const std_msgs::Bool::ConstPtr &msg);

px4_cmd::Command cmd;
bool ext_on = false;

// 运行总时间
float run_time = 60.0;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Ext_cmd");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Subscriber ext_on_sub;
    ext_on_sub = nh.subscribe<std_msgs::Bool>("/px4_cmd/ext_on", 20, ext_on_cb);

    ros::Publisher cmd_pub;
    cmd_pub = nh.advertise<px4_cmd::Command>("/px4_cmd/ext_command", 20);

    // 设置外部命令的基本形式
    cmd.Mode = px4_cmd::Command::Move;              // 模式为移动 Move
    cmd.Move_frame = px4_cmd::Command::ENU;         // 坐标系取绝对坐标系
    cmd.Move_mode = px4_cmd::Command::XYZ_POS;      // 移动指令类型
    cmd.ext_total_time = run_time;
    cmd.ext_time = 0;

    // 初始化计时
    float t = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();
        // 开启外部模式后开始运行指定命令
        if (ext_on)
        {
            cmd.desire_cmd[0] = 5*sin(t);
            cmd.desire_cmd[1] = 5*cos(t);
            cmd.desire_cmd[2] = 5;
            cmd.yaw_cmd = 0;
            t += 0.05;
            cmd.ext_time = t;
            cout << "[Info - Ext cmd - Current Time/Value]  " << fixed << setprecision(2) << t << " / [" << cmd.desire_cmd[0] << ", " << cmd.desire_cmd[1] << ", " << cmd.desire_cmd[2] << "]" << endl;
            ros::spinOnce();
            rate.sleep();
        }
        else
        {
            // 如果初始没有开启外部模式，则输出信息
            if (t == 0)
            {
                ROS_INFO("External Command: Waiting for user-define mode!");
                if (ext_on_sub.getNumPublishers() < 1)
                {
                    ROS_WARN("External Command: No Publisher!");
                }
                sleep(2);
            }
            // 如果中途话题通信截止，则退出
            else
            {
                break;
            }
        }
        
        // 如果超出指定时间，则退出
        if (t > run_time)
        {
            break;
        }

        // 发布指令
        cmd_pub.publish(cmd);
    }
}

// 订阅回调函数,获取设置的指令信息
void ext_on_cb(const std_msgs::Bool::ConstPtr &msg)
{
    ext_on = msg->data;
}