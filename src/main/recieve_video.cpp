// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ctime>

#include <sensor_msgs/Image.h>

#include <print_utility/printf_utility.h>
#include <print_utility/handle_cin.h>

// declare subscribe function
void image_raw_sub(const sensor_msgs::Image::ConstPtr &msg);

// Vedio setting
static const std::string OPENCV_WINDOW = "Camera Image";
cv::VideoWriter video_w;
int encode_type = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
double fps = 30.0;
std::string filename = "";
bool first = true;
bool save_video = true;

// Save path with time
char s[30];
time_t now = time(NULL);
auto tim = *(localtime(&now));
auto tmp = std::strftime(s, 30, "Fly_%Y_%b_%d_%H_%M_%S.avi", localtime(&now));
char *cwd = getenv("HOME");
string save_path = string(cwd) + string("/") + string(s);

// Parameters init
std::string topic_name = "";
bool show_video = true;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recieve_video");
    ros::NodeHandle nh("~");

    bool get_topic = nh.getParam("camera_topic", topic_name);
    bool get_show = nh.getParam("show_video", show_video);
    if (!get_topic)
    {
        ROS_ERROR("Recive_Camera: Not Camera Topic Input!");
        return 1;
    }
    if (!get_show)
    {
        show_video = true;
    }

    ros::Subscriber img_sub;

    img_sub = nh.subscribe<sensor_msgs::Image>(topic_name, 30, image_raw_sub);
    ROS_WARN("Video Recieve Programme is Running, Image Topic: [ %s ]", topic_name.c_str());
    ROS_WARN("Video save to: %s", save_path.c_str());

    bool first_recieve = false;

    while (ros::ok())
    {
        ros::spinOnce();
        if (img_sub.getNumPublishers() < 1)
        {
            ROS_WARN("Recive_Camera: Can not Connet to Camera Topic, Retrying ...");
            first_recieve = false;
            sleep(1);
        }
        else
        {
            if (!first_recieve)
            {
                ROS_WARN("Recive_Camera: Connet to Camera Topic Successfully!");
                first_recieve = true;
            }
        }
        if (!save_video)
        {
            break;
        }
    }
    video_w.release();
    return 0;
}

// 订阅回调返回图像信息
void image_raw_sub(const sensor_msgs::Image::ConstPtr &msg)
{
    // 第一次则建立writer
    if (first)
    {
        first = false;
        video_w.open(save_path, encode_type, fps, cv::Size(msg->width, msg->height), true);
    }
    // 一旦结束则不
    if (!save_video)
    {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::Image current_state = *msg;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    if (show_video)
    {
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    }
    if (cv::waitKey(1) == 27 || !ros::ok())
    {
        video_w.release();
        save_video = false;
        Warning("Video Reciever has been terminated!");
        cv::destroyWindow(OPENCV_WINDOW);
    }
    video_w.write(cv_ptr->image);
}
