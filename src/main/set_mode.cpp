// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <ros/ros.h>

#include <iostream>
#include <string>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

#include <print_utility/printf_utility.h>
#include <print_utility/handle_cin.h>

using namespace std;

// 订阅信息
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);

/* 主函数 */
int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "set_mode");
    ros::NodeHandle nh("~");

    // 获取 robot 名
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

    // 订阅
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(node_name + "/mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(node_name + "/mavros/local_position/pose", 10, pos_cb);

    // 服务
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>(node_name + "/mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(node_name + "/mavros/cmd/arming");

    //设置频率
    ros::Rate rate(50.0);

    // 设定指令初始化
    mavros_msgs::SetMode mode_cmd;
    mavros_msgs::CommandBool arm_cmd;

    // 用户输入
    int switch_mode = 0;

    // 获取输入对应的理想模式
    string desire_mode = "";

    // 统计更改模式错误次数
    int error_times = 0;

    // 定义列表储存所有模式
    std::vector<string> mode_list = {
        "Refresh Status",//刷新状态
        "MANUAL",       //手动
        "OFFBOARD",     //外部控制
        "STABILIZED",   //自稳
        "POSCTL",       //位置控制
        "ALTCTL",       //高度控制
        "AUTO.LAND",    //自动降落
        "AUTO.RTL",     //自动返航
        "Arm",          //解除锁定
        "DisArm",       //锁定
        "Exit"          //退出
    };

    // 等待节点初始化完成
    sleep(2);
    for (int i = 0; i < 50; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // 主循环
    while (ros::ok())
    {
        // 清屏及初始化
        int sys_res = system("clear");
        cout << POINTER;
        error_times = 0;

        // 输出标题及选项
        print_title("PX4 Mode Control", mode_list);
        // 输出当前模式
        cout << "Current Mode: [" << GREEN << current_state.mode << WHITE << "]  [";
        if (current_state.armed)
        {
            cout << GREEN << "Arm" << WHITE << "]" << endl;
        }
        else
        {
            cout << YELLOW << "DisArm" << WHITE << "]" << endl;
        }
        // 输出当前位置
        cout << "Current Position [ENU]:" << endl;
        cout << setprecision(2) << "   x: " << fixed << current_pos.pose.position.x << " m" << endl;
        cout << setprecision(2) << "   y: " << fixed << current_pos.pose.position.y << " m" << endl;
        cout << setprecision(2) << "   z: " << fixed << current_pos.pose.position.z << " m" << endl;

        // 获取输入
        cout << WHITE << "\n" << "Input Mode Number: ";
        cin >> switch_mode;

        // 判断输入正确性
        if (!handle_cin() || switch_mode >= mode_list.size() || switch_mode < 0)
        {
            cout << "\n" << endl;
            cout << NO_POINTER;
            Error("Please Input int 0 ~ " + to_string(mode_list.size() - 1));
            sleep(2);
            cout << POINTER;
            continue;
        }
        // 判断退出
        if (switch_mode == mode_list.size()-1)
        {
            return 0;
        }

        // 回调函数,更新状态
        ros::spinOnce();
        rate.sleep();

        if (switch_mode == 0)
        {
            continue;
        }

        // 获取理想模式
        desire_mode = mode_list[switch_mode];

        // 解锁
        if (desire_mode == "Arm" || desire_mode == "DisArm")
        {
            // 如果飞行高度超过20cm则不允许DisArm
            if (abs(current_pos.pose.position.z) > 0.2 && desire_mode == "DisArm")
            {
                cout << "\n";
                Error("Vehicle is Flying, you can not DisArm!");
                sleep(2);
                continue;
            }
            
            bool desire_arm_cmd = (desire_mode == "Arm") ? true : false;
            while ((current_state.armed != desire_arm_cmd) && error_times < 10)
            {
                arm_cmd.request.value = desire_arm_cmd;
                Info(desire_mode + "ing...");
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    //执行回调函数
                    ros::spinOnce();
                    sleep(1);
                    if (current_state.armed == desire_arm_cmd)
                    {
                        Info(desire_mode + " Command Sent and " + desire_mode + " Successfully!");
                        break;
                    }
                    else
                    {
                        Warning(desire_mode + " Command Sent but " + desire_mode + " Failed!");
                    }
                }
                else
                {
                    Error(desire_mode + " Command Sent Failed!");
                }
                error_times++;
            }
            if (error_times == 0)
            {
                Info("Already " + desire_mode + "!");
            }
        }
        // 更改模式
        else
        {
            while (current_state.mode != desire_mode && error_times < 10)
            {
                // 处于OFFBOARD模式时，只能改为AUTO模式
                if (current_state.mode == "OFFBOARD")
                {
                    if (desire_mode != "AUTO.LAND" && desire_mode != "AUTO.RTL")
                    {
                        cout << "\n";
                        Error("You are in OFFBOARD Mode, you can only change to [Auto.Land] or [Auto.RTL] Mode!");
                        error_times++;
                        break;
                    }
                }
                // 请求更改模式服务
                mode_cmd.request.custom_mode = desire_mode;
                mode_client.call(mode_cmd);
                Info("Changing Mode to " + desire_mode + "...");
                sleep(1);

                if (mode_cmd.response.mode_sent)
                {
                    //执行回调函数,更新状态
                    ros::spinOnce();
                    rate.sleep();
                    if (current_state.mode == desire_mode)
                    {
                        Info(desire_mode + " Mode Sent and Changed Successfully!");
                    }
                    else
                    {
                        Warning(desire_mode + " Mode Sent but Changed Failed!");
                    }
                }
                else
                {
                    Error(desire_mode + " Mode Sent Failed!");
                }
                error_times++;
            }
            // 如果没进循环则说明目前已经是指定的mode
            if (!error_times)
            {
                Info("Already Set to " + desire_mode + " Mode!");
            }
        }        
        // 如果达到10错误次则提示
        if (error_times == 10)
        {
            Error("Error times too much, please retry!");
        }

        sleep(2);
    }
    return 0;
}


// 订阅回调返回状态信息
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
// 订阅回调返回位置信息
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pos = *msg;
}
