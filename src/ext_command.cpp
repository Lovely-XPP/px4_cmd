#include <ros/ros.h>

#include <iostream>
#include <string>
#include <math.h>

#include <px4_cmd/Command.h>
#include <utility/printf_utility.h>

using namespace std;

px4_cmd::Command cmd;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Ext_cmd");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Publisher cmd_pub;
    cmd_pub = nh.advertise<px4_cmd::Command>("/px4_cmd/ext_command", 20);

    cmd.Mode = px4_cmd::Command::Move;
    cmd.Move_frame = px4_cmd::Command::ENU;
    cmd.Move_mode = px4_cmd::Command::XYZ_POS;
    float t = 0.0;
    while(ros::ok())
    {
        while (t < 1000)
        {
            cmd.desire_cmd[0] = 5*sin(t);
            cmd.desire_cmd[1] = 5*cos(t);
            cmd.desire_cmd[2] = 5;
            cmd.yaw_cmd = 0;
            t += 0.05;
            system("clear");
            cout << "[Current Time]  " << fixed << setprecision(2) << t << endl;
            cout << "[Current Value] " << fixed << setprecision(2) << cmd.desire_cmd[0] << " " << cmd.desire_cmd[1] << " " << cmd.desire_cmd[2] << endl;
            cmd_pub.publish(cmd);
            ros::spinOnce();
            rate.sleep();
        }
    }
}