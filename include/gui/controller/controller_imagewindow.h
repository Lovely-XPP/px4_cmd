// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef CONTROLLER_IMAGEWINDOW_H
#define CONTROLLER_IMAGEWINDOW_H
#include <QDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
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
#include <vehicle_state.h>

using namespace std;

Q_DECLARE_METATYPE(cv::Mat);

class ControllerImageWindow : public QDialog
{
    Q_OBJECT
    public:
        ControllerImageWindow(ros::NodeHandle &nh_, const QVector<vehicle_state *> &uavs, QWidget *parent_widget = 0)
        {
            this->setParent(parent_widget);
            this->setAttribute(Qt::WA_DeleteOnClose);
            qRegisterMetaType<cv::Mat>("cv::Mat");
            qRegisterMetaType<cv::Mat>("cv::Mat&");
            nodes = uavs;
            nh = nh_;
            setup();
        }

        ~ControllerImageWindow()
        {
            stop_flag = true;
            save_image = false;
            usleep(200000);
        }

    signals:
        void update_image_signal(QVariant data);

    private:
        // init
        QVector<vehicle_state *> nodes;
        string topic_name;
        string output_file;
        ros::NodeHandle nh;
        ros::Subscriber img_sub;
        bool stop_flag = false;
        bool save_image = false;
        cv::VideoWriter video_w;
        int encode_type = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        double fps = 20.0;

        // init widgets
        QComboBox *uav_select = new QComboBox();
        QComboBox *topic_select = new QComboBox();
        QVBoxLayout *vbox = new QVBoxLayout();
        QHBoxLayout *hbox_topic = new QHBoxLayout();
        QHBoxLayout *hbox_save = new QHBoxLayout();
        QPushButton *browse_button = new QPushButton(this);
        QPushButton *save_image_button = new QPushButton(this);
        QLineEdit *txt_dir = new QLineEdit(this);
        QLabel *image_show = new QLabel(this);
        QLabel *dir_tip = new QLabel(this);
        QLabel *topic_show = new QLabel(this);
        QLabel *topic_show_2 = new QLabel(this);
        QMessageBox *msg_box = new QMessageBox(this);

        void setup()
        {
            // set this
            this->setFixedSize(700, 450);
            this->setWindowTitle("Image Topic Viewer");
            this->setStyleSheet("background-color: rgb(255,250,250)");

            // set combobox
            topic_select->setStyleSheet("QComboBox {background-color: rgb(91, 155, 213); color: black};");
            uav_select->setStyleSheet("background-color: rgb(91, 155, 213)");
            topic_show->setText("Select Topic:");
            topic_show->setMaximumWidth(100);
            topic_show_2->setText("/");
            topic_show_2->setMaximumWidth(10);
            for (size_t i = 0; i < nodes.size(); i++)
            {
                uav_select->addItem(QString::fromStdString(nodes[i]->node_name));
            }
            uav_select->setCurrentIndex(0);
            uav_select->setMaximumWidth(200);
            topic_select->setEnabled(false);
            save_image_button->setEnabled(false);
            browse_button->setEnabled(false);
            topic_select->addItem("None Avaliable Image Sensor");
            if (nodes[0]->sensor_topics.size() > 0)
            {
                topic_select->setStyleSheet("QComboBox {background-color: rgb(91, 155, 213); color: black};");
                topic_select->clear();
                for (size_t i = 0; i < nodes[0]->sensor_topics.size(); i++)
                {
                    topic_select->addItem(nodes[0]->sensor_topics[i]);
                }
                topic_select->setEnabled(true);
                browse_button->setEnabled(true);
                save_image_button->setEnabled(true);
                topic_name = "/" + uav_select->currentText().toStdString() + "/" + topic_select->currentText().toStdString();
                std::thread start_thread(&ControllerImageWindow::start_thread_func, this);
                start_thread.detach();
            }
            else
            {
                topic_select->setStyleSheet("QComboBox {background-color: rgb(91, 155, 213); color: red}");
            }

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
            hbox_topic->addWidget(topic_show);
            hbox_topic->addWidget(uav_select);
            hbox_topic->addWidget(topic_show_2);
            hbox_topic->addWidget(topic_select);
            vbox->addLayout(hbox_topic, 1);
            hbox_save->addWidget(dir_tip);
            hbox_save->addWidget(txt_dir);
            hbox_save->addWidget(browse_button);
            hbox_save->addWidget(save_image_button);
            vbox->addLayout(hbox_save, 1);
            vbox->addWidget(image_show, 7);
            this->setLayout(vbox);

            // Signal connect slot
            QObject::connect(browse_button, &QPushButton::clicked, this, &ControllerImageWindow::browse_file_slot);
            QObject::connect(save_image_button, &QPushButton::clicked, this, &ControllerImageWindow::save_image_slot);
            QObject::connect(uav_select, &QComboBox::currentTextChanged, this, &ControllerImageWindow::change_uav_slot);
            QObject::connect(topic_select, &QComboBox::currentTextChanged, this, &ControllerImageWindow::change_topic_name_slot);
            QObject::connect(this, &ControllerImageWindow::update_image_signal, this, &ControllerImageWindow::update_image_slot);
        }

        void stop()
        {
            stop_flag = true;
        }

        // slot functions
        void change_uav_slot()
        {
            int select_id = uav_select->currentIndex();
            save_image = false;
            stop_flag = true;
            topic_select->blockSignals(true);
            topic_select->clear();
            topic_select->setEnabled(false);
            topic_select->addItem("None Avaliable Image Sensor");
            browse_button->setEnabled(false);
            save_image_button->setEnabled(false);
            if (nodes[select_id]->sensor_topics.size() > 0)
            {
                topic_select->clear();
                topic_select->setStyleSheet("QComboBox {background-color: rgb(91, 155, 213); color: black}");
                for (size_t i = 0; i < nodes[select_id]->sensor_topics.size(); i++)
                {
                    topic_select->addItem(nodes[select_id]->sensor_topics[i]);
                }
                topic_select->setEnabled(true);
                browse_button->setEnabled(true);
                save_image_button->setEnabled(true);
            }
            else
            {
                topic_select->setStyleSheet("QComboBox {background-color: rgb(91, 155, 213); color: red}");
            }
            topic_select->blockSignals(false);
            if (topic_select->isEnabled())
            {
                topic_select->setCurrentIndex(0);
                change_topic_name_slot();
            }
        }

        void change_topic_name_slot()
        {
            release_video_strean();
            save_image = false;
            stop_flag = true;
            topic_name = "/" + uav_select->currentText().toStdString() + "/" + topic_select->currentText().toStdString();
            save_image_button->setText("Save Video");
            save_image_button->setEnabled(true);
            browse_button->setEnabled(true);
            stop_flag = false;
            std::thread start_thread(&ControllerImageWindow::start_thread_func, this);
            start_thread.detach();
        }

        void browse_file_slot()
        {
            char *cwd = getenv("HOME");
            string topic = topic_name;
            topic.erase(0, 1);
            QString file_name = topic.c_str();
            file_name.replace("/", "_");
            QString default_save_path = QString(cwd) + "/" + file_name + ".mp4";
            QString qfilename = QFileDialog::getSaveFileName(this, "Select Saved Video File", default_save_path, "MPEG File (*.mp4)");
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
            img_sub = nh.subscribe<sensor_msgs::Image>(topic_name, 20, &ControllerImageWindow::topic_data_cb, this);
            // judge if subscribe
            ros::spinOnce();
            usleep(500000);
            ros::spinOnce();
            if (img_sub.getNumPublishers() < 1)
            {
                topic_error_msg_box_slot();
                save_image_button->setText("Save Video");
                browse_button->setEnabled(false);
                save_image_button->setEnabled(false);
                img_sub.shutdown();
                return;
            }
            while (ros::ok() && img_sub.getNumPublishers() > 0)
            {
                ros::spinOnce();
                usleep(200000);
                if (stop_flag)
                {
                    save_image_button->setText("Save Video");
                    browse_button->setEnabled(false);
                    save_image_button->setEnabled(false);
                    break;
                }
            }
            img_sub.shutdown();
        }

        void topic_error_msg_box_slot()
        {
            save_image = false;
            stop_flag = true;
            msg_box = new QMessageBox(this);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            msg_box->setText(("Can Not Recieve Topic Data: " + topic_name).c_str());
            msg_box->exec();
        }

        // subscibe callback funciton
        void topic_data_cb(const sensor_msgs::Image::ConstPtr &msg)
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            if (cv_ptr->image.channels() == 3)
            {
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            }
            cv::Mat img = cv_ptr->image;
            QVariant img_data;
            img_data.setValue(img);
            emit update_image_signal(img_data);
            if (save_image)
            {
                if (!video_w.isOpened())
                {
                    std::remove(output_file.c_str());
                    video_w.open(output_file, encode_type, fps, cv::Size(img.cols, img.rows), true);
                }
                video_w.write(img);
            }
            else
            {
                release_video_strean();
            }
        }

        void update_image_slot(QVariant data)
        {
            cv::Mat img = data.value<cv::Mat>();
            QImage qimg = cvMat2QImage(img);
            QPixmap temp_pixmap = QPixmap::fromImage(qimg);
            QPixmap fit_pixmap = temp_pixmap.scaled(image_show->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            if (!stop_flag)
            {
                image_show->setPixmap(fit_pixmap);
            }
            else
            {
                image_show->clear();
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

        void release_video_strean()
        {
            if (video_w.isOpened())
            {
                video_w.release();
            }
        }
};
#endif