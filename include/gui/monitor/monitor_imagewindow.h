#ifndef MONITORIMAGEWINDOW_H
#define MONITORIMAGEWINDOW_H
#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <string>
#include <vector>
#include <cstdio>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class MonitorImageWindow : public QWidget
{
    public:
        QWidget *parent;
        QDialog *win = new QDialog();
        // topic name
        string topic_name;

        MonitorImageWindow(QWidget *parent_widget, const string &topic_name_input, const int &id)
        {
            topic_name = topic_name_input;
            node_id = id;
            setup();
        }

        ~MonitorImageWindow()
        {
            stop_flag = true;
            save_image = false;
            sleep(1);
        }

        void start()
        {
            std::thread start_thread(&MonitorImageWindow::start_thread_func, this);
            start_thread.detach();
        }

    private:
        // init 
        int node_id;
        string output_file;
        ros::NodeHandle nh;
        ros::Subscriber img_sub;
        bool stop_flag = false;
        bool save_image = false;
        cv::VideoWriter video_w;
        int encode_type = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        double fps = 20.0;

        // init widgets
        QVBoxLayout *vbox = new QVBoxLayout();
        QHBoxLayout *hbox = new QHBoxLayout();
        QPushButton *browse_button = new QPushButton(win);
        QPushButton *save_image_button = new QPushButton(win);
        QLineEdit *txt_dir = new QLineEdit(win);
        QLabel *image_show = new QLabel(win);
        QLabel *dir_tip = new QLabel(win);
        QMessageBox *msg_box = new QMessageBox(win);

        void setup()
        {
            // set win
            win->setFixedSize(700, 450);
            win->setWindowTitle(topic_name.c_str());
            win->setStyleSheet("background-color: rgb(255,250,250)");

            // set buttons
            browse_button->setMinimumWidth(80);
            browse_button->setText("Browse");
            save_image_button->setMinimumWidth(80);
            save_image_button->setText("Save Video");

            // show selected path
            dir_tip->setText("Video Save Path: ");
            txt_dir->setStyleSheet("background-color: white");
            txt_dir->setReadOnly(true);

            // show image
            image_show->setAlignment(Qt::AlignCenter);

            // set layout
            hbox->addWidget(dir_tip);
            hbox->addWidget(txt_dir);
            hbox->addWidget(browse_button);
            hbox->addWidget(save_image_button);
            vbox->addLayout(hbox, 1);
            vbox->addWidget(image_show, 8);
            win->setLayout(vbox);

            // Signal connect slot
            QObject::connect(browse_button, &QPushButton::clicked, this, &MonitorImageWindow::browse_file_slot);
            QObject::connect(save_image_button, &QPushButton::clicked, this, &MonitorImageWindow::save_image_slot);
        }

        void stop()
        {
            stop_flag = true;
        }

        // slot functions
        void browse_file_slot()
        {
            char *cwd = getenv("HOME");
            string topic = topic_name;
            topic.erase(0, 1);
            QString file_name = topic.c_str();
            file_name.replace("/", "_");
            QString default_save_path = QString(cwd) + "/" + file_name + ".mp4";
            QString qfilename = QFileDialog::getSaveFileName(win, "Select Saved Video File", default_save_path, "MPEG File (*.mp4)");
            string filename = qfilename.toStdString();
            if (!filename.compare(""))
            {
                txt_dir->setText("");
                output_file = "";
                return;
            }
            txt_dir->setText(filename.c_str());
            output_file = filename;
        }

        void save_image_slot()
        {
            if (output_file == "")
            {
                msg_box = new QMessageBox();
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setWindowTitle("Error");
                msg_box->setText("Please Browse Output Directory.");
                msg_box->exec();
                return;
            }
            if (save_image_button->text() == "Save Video")
            {
                save_image = true;
                save_image_button->setText("Stop Saving Video");
                browse_button->setEnabled(false);
            }
            else
            {
                save_image = false;
                save_image_button->setText("Save Video");
                browse_button->setEnabled(true);
            }
        }

        void start_thread_func()
        {
            // setup ros
            int argc = 0;
            char **argv;
            ros::init(argc, argv, "px4_cmd/monitor/topic_info/" + to_string(node_id));
            img_sub = nh.subscribe<sensor_msgs::Image>(topic_name, 20, &MonitorImageWindow::topic_data_cb, this);
            // judge if subscribe
            ros::spinOnce();
            usleep(500000);
            ros::spinOnce();
            if (img_sub.getNumPublishers() < 1)
            {
                msg_box = new QMessageBox(win);
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setWindowTitle("Error");
                msg_box->setText(("Can Not Recieve Topic Data: " + topic_name).c_str());
                msg_box->exec();
                return;
            }
            while (ros::ok() && img_sub.getNumPublishers() > 0)
            {
                ros::spinOnce();
                usleep(200000);
                if (stop_flag)
                {
                    break;
                }
            }
            nh.shutdown();
        }

        // subscibe callback funciton
        void topic_data_cb(const sensor_msgs::Image::ConstPtr &msg)
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            if (cv_ptr->image.channels() == 3)
            {
                cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
            }
            cv::Mat img = cv_ptr->image;
            QImage qimg = cvMat2QImage(img);
            QPixmap temp_pixmap = QPixmap::fromImage(qimg);
            QPixmap fit_pixmap = temp_pixmap.scaled(image_show->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            image_show->setPixmap(fit_pixmap);
            if (save_image)
            {
                if (!video_w.isOpened())
                {
                    std::remove(output_file.c_str());
                    video_w.open(output_file, encode_type, fps, cv::Size(img.cols, img.rows), true);
                }
            }
            if (!save_image && video_w.isOpened())
            {
                video_w.release();
            }
        }

        // utility function
        QImage cvMat2QImage(const cv::Mat &mat) // Mat转换成QImage
        {
            // 注意,使用opencv Mat的buf给QImage赋值时,务必带入参数mat.step;因为step是计算生成的每一行元素的字节数,它是4字节对齐的;
            // 否则如果输入图片的分辨率宽度不是4的整数倍,那么QImage会出现显示错乱的问题.
            if (mat.type() == CV_8UC1) // 1通道
            {
                const uchar *pSrc = (const uchar *)mat.data;
                QImage image = QImage(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
                return image;
            }
            else if (mat.type() == CV_8UC3) // 3通道
            {
                const uchar *pSrc = (const uchar *)mat.data;
                QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888); // 格式R,G,B,对应0,1,2
                return image.rgbSwapped();                                               // rgbSwapped是为了将BGR格式转换为RGB格式
            }
            else if (mat.type() == CV_8UC4) // 4通道
            {
                const uchar *pSrc = (const uchar *)mat.data;
                // Create QImage with same dimensions as input Mat
                QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32); // 格式B,G,R,A对应0,1,2,3
                return image.copy();
            }
            else if (mat.type() == CV_16SC3)
            {
                cv::Mat normalize_mat;
                normalize(mat, normalize_mat, 0, 255, cv::NORM_MINMAX, -1);
                normalize_mat.convertTo(normalize_mat, CV_8U);
                const uchar *pSrc = (const uchar *)normalize_mat.data;
                // Create QImage with same dimensions as input Mat
                QImage image(pSrc, normalize_mat.cols, normalize_mat.rows, normalize_mat.step, QImage::Format_RGB888);
                return image.rgbSwapped();
            }
            else
            {
                return QImage();
            }
        }
};
#endif