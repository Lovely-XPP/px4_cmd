#include <ros/ros.h>

#include <thread>
#include <iostream>
#include <string>
#include <cstdarg>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_cmd/Command.h>

#include <utility/printf_utility.h>
#include <utility/handle_cin.h>

using namespace std;

// 定义pi值
#define PI 3.14159265358979323846

// 发布消息初始化
ros::Publisher cmd_pub;
ros::Subscriber state_sub;

// 订阅信息
geometry_msgs::PoseStamped current_state;
void state_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

// 初始化命令
px4_cmd::Command cmd;

// 定义列表储存所有模式
std::vector<string> command_list = {
    "Idle",      // 怠速
    "Takeoff",   // 起飞到指定高度
    "Move",      // 移动
    "Hover",     // 悬停
    "Trajectory",// 航点轨迹控制
    "Exit"       // 退出
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
    "Velocity (XY) + Height (Z)", // 定高两速度
    "Velocity (XYZ)",             // 三速度
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
std::vector<float> trajectory_point = {0, 0, 0, 0, 0};
// 初始化用轨迹航点列表
std::vector<vector<float>> init_trajectory_points = {trajectory_point};
// 轨迹航点列表
std::vector<vector<float>> trajectory_points = {trajectory_point};

// 初始化命令信息
float desire_cmd_value[3];
float yaw_value = 0.0;


// 子函数声明
void print_current_cmd(px4_cmd::Command cmd, float cmd_value[3]);
void pub_thread_fun();
void print_trajectory_info(int mode, vector<float> point, std::vector<vector<float>> points, int start);
bool input_cmd(string msg1, string msg2, string msg3, int other_msg, ...);


/*     主函数      */
int main(int argc, char **argv)
{
    // 声明载具
    cmd.Vehicle = px4_cmd::Command::Multicopter;

    // 节点初始化
    ros::init(argc, argv, "set_command");
    ros::NodeHandle nh;

    // 订阅
    state_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, state_cb);

    // 广播初始化
    cmd_pub = nh.advertise<px4_cmd::Command>("/px4_cmd/control_command", 10);

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

    // 开启广播线程
    sleep(1);
    std::thread pub_thread(pub_thread_fun);
    pub_thread.detach();

    // 主循环
    while (ros::ok())
    {
        // 清屏及初始化
        system("clear");
        cout << POINTER;

        // 输出标题及选项
        print_title("PX4 Offboard Command", command_list);
        print_current_cmd(cmd, desire_cmd_value);

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

        // 更改模式
        cmd.Mode = switch_cmd;

        switch (switch_cmd)
        {
            // 待机模式
            case px4_cmd::Command::Idle:
            {
                cmd.Move_frame = px4_cmd::Command::ENU;
                cmd.Move_mode = px4_cmd::Command::XYZ_POS;
                desire_cmd_value[0] = current_state.pose.position.x;
                desire_cmd_value[1] = current_state.pose.position.y;
                desire_cmd_value[2] = current_state.pose.position.z;
                cmd.desire_cmd[0] = desire_cmd_value[0];
                cmd.desire_cmd[1] = desire_cmd_value[1];
                cmd.desire_cmd[2] = desire_cmd_value[2];
                cmd.yaw_cmd = 0.0;
                break;
            }

            // 起飞模式
            case px4_cmd::Command::Takeoff:
            {
                // 用户指定起飞高度
                cout << "\n" << "Set Takeoff Height [m]: ";
                cin >> desire_cmd_value[2];
                desire_cmd_value[2] = abs(desire_cmd_value[2]);
                cmd.Move_frame = px4_cmd::Command::ENU;
                cmd.Move_mode = px4_cmd::Command::XYZ_POS;
                desire_cmd_value[0] = current_state.pose.position.x;
                desire_cmd_value[1] = current_state.pose.position.y;
                cmd.desire_cmd[0] = desire_cmd_value[0];
                cmd.desire_cmd[1] = desire_cmd_value[1];
                cmd.desire_cmd[2] = desire_cmd_value[2];
                cmd.yaw_cmd = 0.0;
                break;
            }

            // Move 模式
            case px4_cmd::Command::Move:
            {
                // 输入坐标系
                correct = false;
                while (!correct)
                {
                    system("clear");
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
                
                // 最后一个选项为返回
                if (switch_frame == (frame_list.size() - 1))
                {
                    cmd.Mode = px4_cmd::Command::Hover;
                    continue;
                }

                // 机体坐标系仅支持速度控制
                if (switch_frame == px4_cmd::Command::BODY)
                {
                    system("clear");
                    print_head("PX4 Command Center");
                    string msg = string(YELLOW) + "Tip: Body Frame Only Support [" + GREEN + "Velocity (XYZ)" + YELLOW + "] Control!" + WHITE + "\n";
                    if (!input_cmd("X Velocity [m/s]: ", "Y Velocity [m/s]: ", "Z Velocity [m/s]: ", 1, msg))
                    {
                        continue;
                    }
                    // 修改命令
                    cmd.Move_frame = switch_frame;
                    cmd.Move_mode = px4_cmd::Command::XYZ_VEL;
                    cmd.desire_cmd[0] = desire_cmd_value[0];
                    cmd.desire_cmd[1] = desire_cmd_value[1];
                    cmd.desire_cmd[2] = desire_cmd_value[2];
                    cmd.yaw_cmd = yaw_value / 180 * PI;
                    break;
                }

                // 输入命令类型
                system("clear");
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
                switch (switch_cmd_mode)
                {
                    case px4_cmd::Command::XYZ_POS:
                    {
                        if (!input_cmd("X Position [m]: ", "Y Position [m]: ", "Z Position [m]: ", 0))
                        {
                            continue;
                        }
                        break;
                    }

                    case px4_cmd::Command::XY_VEL_Z_POS:
                    {
                        if (!input_cmd("X Velocity [m/s]: ", "Y Velocity [m/s]: ", "Z Position [m]: ", 0))
                        {
                            continue;
                        }
                        break;
                    }

                    case px4_cmd::Command::XYZ_VEL:
                    {
                        if (!input_cmd("X Velocity [m/s]: ", "Y Velocity [m/s]: ", "Z Velocity [m/s]: ", 0))
                        {
                            continue;
                        }
                        break;
                    }

                    case px4_cmd::Command::XYZ_REL_POS:
                    {
                        if (!input_cmd("X Relative Position [m]: ", "Y Relative Position [m]: ", "Z Relative Position [m]: ", 0))
                        {
                            continue;
                        }
                        break;
                    }
                }

                // 修改命令
                cmd.Move_frame = switch_frame;
                cmd.Move_mode = switch_cmd_mode;
                // 相对位置指令需要加上当前的位置得到绝对位置
                if (switch_cmd_mode == px4_cmd::Command::XYZ_REL_POS)
                {
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

                cmd.yaw_cmd = yaw_value / 180.0 * PI;
                break;
            }

            // 悬停模式
            case px4_cmd::Command::Hover:
            {
                break;
            }

            // 轨迹模式
            case px4_cmd::Command::Trajectory:
            {
                // 初始化
                trajectory_next = true;
                trajectory_points = init_trajectory_points;
                // 输入模式：相对位置/绝对位置
                system("clear");
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
                        system("clear");
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
                        //偏航角指令
                        cout << "\n" << "Yaw Command [deg]: ";
                        cin >> trajectory_point[3];
                        //航点等待时间
                        cout << "\n" << "Wait Time [s,int]: ";
                        cin >> trajectory_point[4];
                        correct = handle_cin();
                        if (!correct)
                        {
                            cout << "\n" << NO_POINTER;
                            Error("Input illegal, Please input number!");
                            sleep(2);
                            cout << POINTER;
                            continue;
                        }
                        trajectory_point[4] = (int) abs(trajectory_point[4]);

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
                system("clear");
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
                system("clear");
                print_head("PX4 Trajectory Center");
                print_trajectory_info(switch_trajectory_mode, trajectory_point, trajectory_points, 0);
                cout << "\n" << endl;
                Info("Trajectory Mode is Running...");
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
                    cmd.yaw_cmd = trajectory_points[i][3] / 180 * PI;
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
                Info("Trajectory Executed Done, Automatically changed to Hover Command!");
                cmd.Mode = px4_cmd::Command::Hover;
                sleep(2);
            }
        }
    }
    return 0;
}

// 打印当前命令
void print_current_cmd(px4_cmd::Command cmd, float cmd_value[3])
{
    cout << WHITE << "Current Command: [" << GREEN << command_list[cmd.Mode] << WHITE << "]    ";
    cout << WHITE << "Frame: [" << GREEN << frame_list[cmd.Move_frame] << WHITE << "]" << endl;
    cout << WHITE << "Mode: [" << GREEN << move_list[cmd.Move_mode] << WHITE << "]" << endl;
    if (cmd.Mode != px4_cmd::Command::Hover)
    {
        cout << WHITE << "Value: " << fixed << setprecision(2) << cmd_value[0]
             << "  " << cmd_value[1] << "  " << cmd_value[2] << "    ";
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

// 轨迹标题输出
void print_trajectory_info(int mode, vector<float>  point, std::vector<vector<float>> points, int start)
{
    //打印模式
    cout << WHITE << "Trajectory Mode: [" << GREEN << trajectory_list[mode] << WHITE << "]\n" << endl;
    //打印已输入航点
    cout << WHITE << "--------------------- Current Points ----------------------\n"
         << "ID\t X [m]\t Y [m] \t Z [m] \t Yaw [deg] \t Wait [s]";
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
        // yaw指令输入
        cout << "\n" << "Yaw Command [deg]: ";
        cin >> yaw_value;
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
        cmd.Mode = px4_cmd::Command::Hover;
        exec = false;
    }
    return exec;
}