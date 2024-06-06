// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <ros/ros.h>

#include <thread>
#include <iostream>
#include <string>
#include <stdio.h>
#include <termio.h>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_cmd/Command.h>
#include <std_msgs/Bool.h>

#include <print_utility/printf_utility.h>
#include <print_utility/handle_cin.h>

using namespace std;

// 定义pi值
#define PI 3.14159265358979323846

// 发布消息初始化
ros::Publisher cmd_pub;
ros::Publisher ext_cmd_state_pub;
ros::Subscriber state_sub;
ros::Subscriber cmd_sub;

// 订阅信息
geometry_msgs::PoseStamped current_state;
std_msgs::Bool ext_cmd_state_msg;
px4_cmd::Command external_cmd;

// 声明回调函数
void state_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void external_cmd_cb(const px4_cmd::Command::ConstPtr &msg);

// 初始化命令
px4_cmd::Command cmd;
bool ext_cmd_state = false;

// 定义列表储存所有模式
std::vector<string> command_list = {
    "Idle",             // 怠速
    "Takeoff",          // 起飞到指定高度
    "Move",             // 移动
    "Loiter",           // 盘旋
    "Trajectory",       // 航点轨迹控制
    "External Command", // 外部命令输入
    "Refresh Status",   // 刷新状态
    "Exit"              // 退出
};

// 坐标系
std::vector<string> frame_list = {
    "ENU",  // 惯性坐标系，东北天
    "Body", // 机体坐标系
    "Exit"  // 退出
};

// 指令方式
std::vector<string> move_list = {
    "Position (XYZ)",             // 三位置
    "Relative Position (XYZ)",    // 三相对位置
    "Exit"                        // 退出
};

// 航点输入方式
std::vector<string> trajectory_list = {
    "Position (XYZ)",          // 三位置
    "Relative Position (XYZ)", // 三相对位置
    "Exit"                     // 退出
};


/* 初始化变量 */
// 用户输入
int switch_cmd = 0;
int switch_frame = 0;
int switch_cmd_mode = 0;
int switch_trajectory_mode = 0;
string confirm_exec = "0";
bool correct = false;

// 轨迹模式专用
// 判断是否继续增加航点
bool trajectory_next = false;
char next_point = '0';
char confirm_trajectory = '0';
float err_x = 0.0;
float err_y = 0.0;
float err_z = 0.0;
// 当前输入的轨迹航点
std::vector<float> trajectory_point = {0, 0, 0, 0};
// 初始化用轨迹航点列表
std::vector<vector<float>> init_trajectory_points = {trajectory_point};
// 轨迹航点列表
std::vector<vector<float>> trajectory_points = {trajectory_point};

// 外部命令模式专用
std::vector<string> null_string;
bool ext_exit = false;

// 初始化命令信息
float desire_cmd_value[3];
float yaw_value = 0.0;


// 子函数声明
void print_current_cmd(px4_cmd::Command cmd, string topic, bool topic_state);
void pub_thread_fun();
void print_trajectory_info(int mode, vector<float> point, std::vector<vector<float>> points, int start);
bool input_cmd(string msg1, string msg2, string msg3, int other_msg, ...);
void judge_esc_thread_func();


/*     主函数      */
int main(int argc, char **argv)
{
    // 声明载具
    cmd.Vehicle = px4_cmd::Command::FixWing;
    
    // 节点初始化
    ros::init(argc, argv, "set_command");
    ros::NodeHandle nh;
    ros::Rate cmd_rate(10.0);
    
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

    // 外部命令默认话题名，支持通过命令行参数输入
    std::string topic_name = node_name + "/px4_cmd/external_command";
    bool get_topic = nh.getParam("cmd_topic", topic_name);

    // 订阅
    state_sub = nh.subscribe<geometry_msgs::PoseStamped>(node_name + "/mavros/local_position/pose", 10, state_cb);
    cmd_sub = nh.subscribe<px4_cmd::Command>(topic_name, 20, external_cmd_cb);

    // 广播初始化
    cmd_pub = nh.advertise<px4_cmd::Command>(node_name + "/px4_cmd/control_command", 10);
    ext_cmd_state_pub = nh.advertise<std_msgs::Bool>(node_name + "/px4_cmd/ext_cmd_state", 20);

    // 命令信息
    desire_cmd_value[0] = 0.0;
    desire_cmd_value[1] = 0.0;
    desire_cmd_value[2] = 0.0;
    cmd.Mode = px4_cmd::Command::Idle;
    cmd.Move_frame = px4_cmd::Command::ENU;
    cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    cmd.desire_cmd[0] = desire_cmd_value[0];
    cmd.desire_cmd[1] = desire_cmd_value[1];
    cmd.desire_cmd[2] = desire_cmd_value[2];
    cmd.yaw_cmd = yaw_value;
    cmd.header.frame_id = 1;

    // 开启广播线程
    sleep(1);
    std::thread pub_thread(pub_thread_fun);
    pub_thread.detach();

    // 主循环
    while (ros::ok())
    {
        // 清屏及初始化
        int sys_res = system("clear");
        cout << POINTER;

        // 输出标题及选项
        print_title("PX4 Offboard Command", command_list);
        print_current_cmd(cmd, topic_name, cmd_sub.getNumPublishers() >= 1);

        // 获取用户输入
        cout << "\n" << "Input Command Number: ";
        cin >> switch_cmd;
        correct = handle_cin();

        // 判断输入正确性
        if (!correct || switch_cmd >= command_list.size() || switch_cmd < 0)
        {
            cout << "\n" << NO_POINTER;
            Error("Please Input int 0 ~ " + to_string(command_list.size() - 1));
            sleep(2);
            cout << POINTER;
            continue;
        }

        if (switch_cmd == (command_list.size() - 1))
        {
            return 0;
        }

        switch (switch_cmd)
        {
            // 待机模式
            case px4_cmd::Command::Idle:
            {
                cmd.Mode = switch_cmd;
                cmd.Move_frame = px4_cmd::Command::ENU;
                cmd.Move_mode = px4_cmd::Command::XYZ_POS;
                desire_cmd_value[0] = current_state.pose.position.x;
                desire_cmd_value[1] = current_state.pose.position.y;
                desire_cmd_value[2] = current_state.pose.position.z;
                cmd.desire_cmd[0] = desire_cmd_value[0];
                cmd.desire_cmd[1] = desire_cmd_value[1];
                cmd.desire_cmd[2] = desire_cmd_value[2];
                break;
            }

            // 起飞模式
            case px4_cmd::Command::Takeoff:
            {
                // 用户指定起飞高度
                if (!input_cmd("X Position [m]: ", "Y Position [m]: ", "Z Position [m]: ", 0))
                {
                    continue;
                }
                cmd.Mode = switch_cmd;
                cmd.Move_frame = px4_cmd::Command::ENU;
                cmd.Move_mode = px4_cmd::Command::Takeoff;
                cmd.desire_cmd[0] = desire_cmd_value[0];
                cmd.desire_cmd[1] = desire_cmd_value[1];
                cmd.desire_cmd[2] = desire_cmd_value[2];
                break;
            }

            // Move 模式
            case px4_cmd::Command::Move:
            {
                // 输入坐标系
                correct = false;
                while (!correct)
                {
                    sys_res = system("clear");
                    print_title("PX4 Command Center", frame_list);
                    cout << WHITE << "Input frame id: ";
                    cin >> switch_frame;
                    correct = handle_cin();
                    // 判断输入正确性
                    if (!correct || switch_frame >= frame_list.size() || switch_frame < 0)
                    {
                        cout << "\n" << NO_POINTER;
                        Error("Please Input int 0 ~ " + to_string(frame_list.size() - 1));
                        sleep(2);
                        cout << POINTER;
                        continue;
                    }
                }
                
                if (switch_frame == (frame_list.size() - 1))
                {
                    cmd.Mode = px4_cmd::Command::Loiter;
                    continue;
                }


                // 输入Move模式类型
                sys_res = system("clear");
                print_title("PX4 Command Center", move_list);
                cout << WHITE << "Input Move Mode Number: ";
                cin >> switch_cmd_mode;
                correct = handle_cin();
                // 判断输入正确性
                if (!correct || switch_cmd_mode >= move_list.size() || switch_cmd_mode < 0)
                {
                    cout << "\n" << NO_POINTER;
                    Error("Please Input int 0 ~ " + to_string(move_list.size() - 1));
                    sleep(2);
                    continue;
                }

                // 输入相应模式的值
                // 绝对位置
                if (switch_cmd_mode == px4_cmd::Command::XYZ_POS)
                {
                    if (!input_cmd("X Position [m]: ", "Y Position [m]: ", "Z Position [m]: ", 0))
                    {
                        continue;
                    }
                }
                // 相对位置
                if (switch_cmd_mode == px4_cmd::Command::XYZ_REL_POS)
                {
                    if (!input_cmd("X Relative Position [m]: ", "Y Relative Position [m]: ", "Z Relative Position [m]: ", 0))
                    {
                        continue;
                    }
                }
                    
                // 修改命令
                cmd.Mode = switch_cmd;
                cmd.Move_frame = switch_frame;
                cmd.Move_mode = switch_cmd_mode;

                // 更改具体指令
                if (switch_cmd_mode == px4_cmd::Command::XYZ_REL_POS)
                {
                    // 相对位置指令需要加上当前的位置得到绝对位置
                    cmd.desire_cmd[0] = desire_cmd_value[0] + current_state.pose.position.x;
                    cmd.desire_cmd[1] = desire_cmd_value[1] + current_state.pose.position.y;
                    cmd.desire_cmd[2] = desire_cmd_value[2] + current_state.pose.position.z;
                }
                else
                {
                    cmd.desire_cmd[0] = desire_cmd_value[0];
                    cmd.desire_cmd[1] = desire_cmd_value[1];
                    cmd.desire_cmd[2] = desire_cmd_value[2];
                }
                break;
            }

            // 盘旋模式
            case px4_cmd::Command::Loiter:
            {
                cmd.Mode = switch_cmd;
                break;
            }

            // 轨迹模式
            case px4_cmd::Command::Trajectory:
            {
                // 初始化
                trajectory_next = true;
                trajectory_points = init_trajectory_points;
                // 输入模式：相对位置/绝对位置
                sys_res = system("clear");
                print_title("PX4 Trajectory Center", trajectory_list);
                cout << YELLOW << "Tip: Trajectory Only Support Frame [" << GREEN << "ENU" << YELLOW << "]" << endl;
                cout << WHITE << "\n" << "Input Trajectory Mode Number: ";
                cin >> switch_trajectory_mode;
                correct = handle_cin();
                // 判断输入正确性
                if (!correct || switch_trajectory_mode >= trajectory_list.size() || switch_trajectory_mode < 0)
                {
                    cout << "\n" << NO_POINTER;
                    Error("Please Input int 0 ~ " + to_string(trajectory_list.size() - 1));
                    sleep(2);
                    cout << POINTER;
                    continue;
                }
                // 循环输入航点
                while (trajectory_next)
                {
                    correct = false;
                    while (!correct)
                    {
                        // 清空并显示轨迹航点输入模式
                        sys_res = system("clear");
                        print_head("PX4 Trajectory Center");
                        print_trajectory_info(switch_trajectory_mode, trajectory_point, trajectory_points, 1);
                        cout << "\n\n"
                            << "######################### Point " << trajectory_points.size() << " #########################" << endl;
                        //判断模式对应输入(相对位置/绝对位置)
                        if (!switch_trajectory_mode)
                        {
                            cout << "\n" << "X Position [m]: ";
                            cin >> trajectory_point[0];
                            cout << "\n" << "Y Position [m]: ";
                            cin >> trajectory_point[1];
                            cout << "\n" << "Z Position [m]: ";
                            cin >> trajectory_point[2];
                        }
                        else
                        {
                            cout << "\n" << "X Relative Position [m]: ";
                            cin >> trajectory_point[0];
                            cout << "\n" << "Y Relative Position [m]: ";
                            cin >> trajectory_point[1];
                            cout << "\n" << "Z Relative Position [m]: ";
                            cin >> trajectory_point[2];
                        }
                        //航点等待时间
                        cout << "\n" << "Wait Time [s,int]: ";
                        cin >> trajectory_point[3];
                        correct = handle_cin();
                        if (!correct)
                        {
                            cout << "\n" << NO_POINTER;
                            Error("Input illegal, Please input number!");
                            sleep(2);
                            cout << POINTER;
                            continue;
                        }
                        trajectory_point[3] = (int) abs(trajectory_point[3]);

                    }
                    
                    //存入总向量
                    trajectory_points.push_back(trajectory_point);

                    //用户输入是否继续增加航点
                    cout << "\n" << YELLOW << "Add Next Point? (0 -> exit, else -> continue): " << WHITE;
                    cin >> next_point;
                    if (next_point == '0')
                    {
                        break;
                    }
                }
                // 删除初始化的第一个点
                trajectory_points.erase(trajectory_points.begin());

                //输出标题
                sys_res = system("clear");
                print_head("PX4 Trajectory Center");
                print_trajectory_info(switch_trajectory_mode, trajectory_point, trajectory_points, 0);
                //用户确认航点
                cout << "\n" << YELLOW << "Confirm to Execute? (0 -> exit, else -> continue): " << WHITE;
                cin >> confirm_trajectory;
                if (confirm_trajectory == '0')
                {
                    break;
                }

                //开始执行
                sys_res = system("clear");
                print_head("PX4 Trajectory Center");
                print_trajectory_info(switch_trajectory_mode, trajectory_point, trajectory_points, 0);
                cout << "\n" << endl;
                Info("Trajectory Mode is Running...");
                cmd.Mode = switch_cmd;
                cmd.Move_frame = px4_cmd::Command::ENU;
                cmd.Move_mode = px4_cmd::Command::XYZ_POS;
                for (int i = 0; i < trajectory_points.size(); i++)
                {
                    Info("Flying to Point " + to_string(i+1));
                    if (!switch_trajectory_mode)
                    {
                        cmd.desire_cmd[0] = trajectory_points[i][0];
                        cmd.desire_cmd[1] = trajectory_points[i][1];
                        cmd.desire_cmd[2] = trajectory_points[i][2];
                    }
                    else
                    {
                        cmd.desire_cmd[0] = trajectory_points[i][0] + current_state.pose.position.x;
                        cmd.desire_cmd[1] = trajectory_points[i][1] + current_state.pose.position.y;
                        cmd.desire_cmd[2] = trajectory_points[i][2] + current_state.pose.position.z;
                    }
                    // 判断误差，误差小则认为到达航点附近
                    err_x = 1;
                    err_y = 1;
                    err_z = 1;
                    while (err_x > 0.1 || err_y > 0.1 || err_z > 0.1)
                    {
                        ros::spinOnce();
                        err_x = abs(current_state.pose.position.x - cmd.desire_cmd[0]);
                        err_y = abs(current_state.pose.position.y - cmd.desire_cmd[1]);
                        err_z = abs(current_state.pose.position.z - cmd.desire_cmd[2]);
                        sleep(1);
                    }
                    Info("Arrive at Point " + to_string(i + 1) + ", Wait " + to_string((int)trajectory_points[i][4]) + " s");
                    // 等待规定时间
                    sleep((int)trajectory_points[i][4]);
                }
                Info("Trajectory Executed Done, Automatically changed to Loiter Command!");
                cmd.Mode = px4_cmd::Command::Loiter;
                sleep(2);
                break;
            }

            // 外部命令模式
            case px4_cmd::Command::User_define:
            {
                std::thread judge_esc_thread(judge_esc_thread_func);
                judge_esc_thread.detach();
                while (true) // 按ESC退出
                {
                    ext_cmd_state = true;
                    ext_cmd_state_msg.data = ext_cmd_state;
                    ext_cmd_state_pub.publish(ext_cmd_state_msg);
                    ros::spinOnce();
                    cmd_rate.sleep();
                    if (cmd_sub.getNumPublishers() < 1)
                    {
                        sys_res = system("clear");
                        print_title("PX4 External Command", null_string);
                        cout << RED << "[ERROR] External Cmd Topic Disconneted!" << WHITE << endl;
                        cout << "\nPress [ESC] to exit." << endl;
                        cmd.Mode = px4_cmd::Command::Loiter;
                        cmd.Move_frame = px4_cmd::Command::ENU;
                        switch_cmd = px4_cmd::Command::Loiter;
                        ext_cmd_state = false;
                        ext_cmd_state_msg.data = ext_cmd_state;
                        ext_cmd_state_pub.publish(ext_cmd_state_msg);
                        ros::Duration(1).sleep();
                        while (!ext_exit)
                        {
                            ros::Duration(0.2).sleep();
                        }
                        ext_exit = false;
                        break;
                    }
                    if (ext_exit)
                    {
                        sys_res = system("clear");
                        print_title("PX4 External Command", null_string);
                        cout << YELLOW << "[INFO] User Terminate External Cmd!" << WHITE << endl;
                        ros::Duration(1).sleep();
                        cmd.Mode = px4_cmd::Command::Loiter;
                        cmd.Move_frame = px4_cmd::Command::ENU;
                        switch_cmd = px4_cmd::Command::Loiter;
                        ext_cmd_state = false;
                        ext_cmd_state_msg.data = ext_cmd_state;
                        ext_cmd_state_pub.publish(ext_cmd_state_msg);
                        ext_exit = false;
                        break;
                    }
                    if (external_cmd.Move_mode == px4_cmd::Command::Custom_Command)
                    {
                        cmd.Mode = external_cmd.Mode;
                        cmd.Move_frame = external_cmd.Move_frame;
                        cmd.Move_mode = external_cmd.Move_mode;
                        for (size_t i = 0; i < 20; i++)
                        {
                            cmd.custom_cmd[i] = external_cmd.custom_cmd[i];
                        }
                    }
                    else
                    {
                        cmd.Mode = external_cmd.Mode;
                        cmd.Move_frame = external_cmd.Move_frame;
                        cmd.Move_mode = external_cmd.Move_mode;
                        cmd.desire_cmd[0] = external_cmd.desire_cmd[0];
                        cmd.desire_cmd[1] = external_cmd.desire_cmd[1];
                        cmd.desire_cmd[2] = external_cmd.desire_cmd[2];
                        cmd.yaw_cmd = external_cmd.yaw_cmd;
                    }
                    sys_res = system("clear");
                    print_title("PX4 External Command", null_string);
                    cout << "Exit: Press [ESC]" << endl;
                    cout << "Time: " << fixed << setprecision(2) << external_cmd.ext_time << "/" << external_cmd.ext_total_time << endl;
                    // custom command output info
                    if (external_cmd.Move_mode == px4_cmd::Command::Custom_Command)
                    {
                        int id = 0;
                        std::cout << "Custom Command: " << std::endl;
                        // position
                        std::cout << "[Position Local]: ";
                        for (size_t i = 0; i < 3; i++)
                        {
                            std::cout << cmd.custom_cmd[i + id] << " ";
                        }
                        std::cout << std::endl;

                        // velocity
                        id += 3;
                        std::cout << "[Velocity]: ";
                        for (size_t i = 0; i < 3; i++)
                        {
                            std::cout << cmd.custom_cmd[i + id] << " ";
                        }
                        std::cout << std::endl;

                        // accelerate
                        id += 3;
                        std::cout << "[accelerate]: ";
                        for (size_t i = 0; i < 3; i++)
                        {
                            std::cout << cmd.custom_cmd[i + id] << " ";
                        }
                        std::cout << std::endl;

                        // Force flag
                        id += 3;
                        std::cout << "[Force Flag]: " << (bool)cmd.custom_cmd[id] << std::endl;

                        // Yaw
                        id += 1;
                        std::cout << "[Yaw]: " << cmd.custom_cmd[id] << std::endl;

                        // Yaw rate
                        id += 1;
                        std::cout << "[Yaw Rate]: " << cmd.custom_cmd[id] << std::endl;

                        // Attitude
                        id += 1;
                        std::cout << "[Attitude (quaternion)]: ";
                        for (size_t i = 0; i < 4; i++)
                        {
                            std::cout << cmd.custom_cmd[i + id] << " ";
                        }
                        std::cout << std::endl;

                        // Attidute Rate
                        id += 4;
                        std::cout << "[Attitude Rate]: ";
                        for (size_t i = 0; i < 3; i++)
                        {
                            std::cout << cmd.custom_cmd[i + id] << " ";
                        }
                        std::cout << std::endl;

                        // Thrust
                        id += 3;
                        std::cout << "[Thrust]: " << cmd.custom_cmd[id] << std::endl;
                        continue;
                    }
                    print_current_cmd(cmd, "", false);
                }
                break;
            }
            // 刷新
            default:
            {
                break;
            }
        }
    }
    return 0;
}

// 打印当前命令
void print_current_cmd(px4_cmd::Command cmd, string topic, bool topic_state)
{

    if (topic != "")
    {
        cout << WHITE << "Outside Command Topic: [" << YELLOW << topic << WHITE << "]";
        if (topic_state)
        {
                cout << " [" << GREEN << "Active" << WHITE << "]" << endl;
        }
        else
        {
                cout << " [" << RED << "Deactive" << WHITE << "]" << endl;
        }
    }
    cout << WHITE << "Current Command: [" << GREEN << command_list[cmd.Mode] << WHITE << "]    " << endl;
    cout << WHITE << "Frame: [" << GREEN << frame_list[cmd.Move_frame] << WHITE << "]" << endl;
    cout << WHITE << "Mode: [" << GREEN << move_list[cmd.Move_mode] << WHITE << "]" << endl;
    if (cmd.Mode != px4_cmd::Command::Hover)
    {
        cout << WHITE << "Value: " << fixed << setprecision(2) << cmd.desire_cmd[0]
             << "  " << cmd.desire_cmd[1] << "  " << cmd.desire_cmd[2] << "    ";
        cout << WHITE << "Yaw: " << fixed << setprecision(2) << cmd.yaw_cmd << endl;
    }
}

// 广播线程
void pub_thread_fun()
{
    ros::Rate rate(50.0);
    while (ros::ok())
    {
        cmd_pub.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
}

// 订阅回调返回状态信息
void state_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_state = *msg;
}

void external_cmd_cb(const px4_cmd::Command::ConstPtr &msg)
{
    external_cmd = *msg;
}

// 轨迹标题输出
void print_trajectory_info(int mode, vector<float>  point, std::vector<vector<float>> points, int start)
{
    //打印模式
    cout << WHITE << "Trajectory Mode: [" << GREEN << trajectory_list[mode] << WHITE << "]\n" << endl;
    //打印已输入航点
    cout << WHITE << "--------------------- Current Points ----------------------\n"
         << "ID\t X [m]\t Y [m] \t Z [m] \t\t Wait [s]";
    for (int i = start; i < points.size(); i++)
    {
        cout << WHITE << "\n" << (start == 0 ? i + 1 : i);
        for (int j = 0; j < (point.size() - 1); j++)
        {
            cout << WHITE << "\t " << setprecision(2) << fixed << points[i][j];
        }
        cout << WHITE << "\t\t " << setprecision(2) << fixed << points[i][point.size()-1];
    }
}


// 输入指令
bool input_cmd(string msg1, string msg2, string msg3, int other_msg, ...)
{
    // 判断是否有其他信息输入，有的话则先输出
    if (other_msg > 0)
    {
        va_list arguments;
        va_start(arguments, other_msg);
        for (int x = 0; x < other_msg; x++)
        {
            string msg_arg = va_arg(arguments, char*);
            cout << msg_arg;
        }
        va_end(arguments);
    }

    bool exec = true;
    bool correct = false;
    while (!correct)
    {
        cout << "\n" << msg1;
        cin >> desire_cmd_value[0];
        cout << "\n" << msg2;
        cin >> desire_cmd_value[1];
        cout << "\n" << msg3;
        cin >> desire_cmd_value[2];
        correct = handle_cin();
        if (!correct)
        {
            cout << "\n" << NO_POINTER;
            Error("Input illegal, Please input number!");
            sleep(2);
            cout << POINTER;
        }
    }
    // 用户确认
    cout << "\n" << YELLOW << "Confirm to Execute? (0 -> exit, else -> continue): " << WHITE;
    cin >> confirm_exec;
    if (confirm_exec.compare("0") == 0)
    {
        cmd.Mode = px4_cmd::Command::Loiter;
        exec = false;
    }
    return exec;
}

void judge_esc_thread_func()
{
    int in;
    while (true)
    {
        struct termios new_settings;
        struct termios stored_settings;
        tcgetattr(0, &stored_settings); // 读取原始配置信息
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON); // 屏蔽整行缓存，不需要回车，输入单个字符即可输出
        new_settings.c_cc[VTIME] = 0;
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0, TCSANOW, &new_settings); // 设置新的配置信息

        in = getchar();

        tcsetattr(0, TCSANOW, &stored_settings); // 恢复原始配置信息

        if (in == 27)
        {
            ext_exit = true;
            break;
        }
    }
}