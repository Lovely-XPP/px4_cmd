#ifndef GENERATORSENSORSWINDOW_H
#define GENERATORSENSORSWINDOW_H
#include <QDialog>
#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextBrowser>
#include <QGroupBox>
#include <QFileDialog>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <json/json.h>

using namespace std;

// structure
struct sensor_data
{
    string name = "";
    vector<string> position = {"0.0", "0.0", "0.0"};
    vector<string> pose = {"0.0", "0.0", "0.0"};
    string samples = "20";
    string resolution = "1";
    string update_rate = "30";
    string width = "640";
    string height = "480";
    string min_angle = "-0.0001";
    string max_angle = "0";
    string distance = "0.07";
};

class GeneratorSensorsWindow : public QWidget
{
    public:
        QDialog *win = new QDialog();
        // sensors data list
        vector<sensor_data *> sensors_save_data;
        vector<sensor_data *> sensors_data;
        GeneratorSensorsWindow(QWidget *parent_widget)
        {
            setup();
        }

    private:
        // init data
        sensor_data lidar;
        sensor_data depth_cam;
        sensor_data rgb_cam;
        sensor_data stereo_cam;
        sensor_data realsense_cam;
        vector<sensor_data> load_data;
        vector<string> sensors_types = {"--- Select Sensor Type ---", "Lidar", "Depth Camera", "RGB Camera", "Stereo Camera", "Realsense Camera"};

        // init widgets
        QMessageBox *msg_box;
        QVBoxLayout *vbox = new QVBoxLayout();
        QHBoxLayout *hbox = new QHBoxLayout();
        QComboBox *box_select;
        QPushButton *export_button ;
        QPushButton *load_button;
        QPushButton *save_button;
        QLineEdit *pos_txt_1;
        QLineEdit *pos_txt_2;
        QLineEdit *pos_txt_3;
        QLineEdit *pose_txt_1;
        QLineEdit *pose_txt_2;
        QLineEdit *pose_txt_3;
        QLineEdit *lidar_samples;
        QLineEdit *lidar_resolution;
        QLineEdit *lidar_min_angle;
        QLineEdit *lidar_max_angle;
        QLineEdit *cam_width;
        QLineEdit *cam_height;
        QLineEdit *stereo_cam_distance;
        QLineEdit *update_rate;

        // ui setting
        int textedit_max_height = 30;

        void setup()
        {
            // init data
            if (sensors_save_data.size() > 0)
            {
                sensors_data = sensors_save_data;
            }
            else
            {
                lidar.name = "Lidar";
                depth_cam.name = "Depth Camera";
                rgb_cam.name = "RGB Camera";
                stereo_cam.name = "Stereo Camera";
                realsense_cam.name = "Realsense Camera";
                sensors_data.push_back(&lidar);
                sensors_data.push_back(&depth_cam);
                sensors_data.push_back(&rgb_cam);
                sensors_data.push_back(&stereo_cam);
                sensors_data.push_back(&realsense_cam);
                sensors_save_data = sensors_data;
            }
            // set win
            win->setFixedSize(600, 600);
            win->setWindowTitle("PX4 Cmd Launch File Generator");
            win->setStyleSheet("background-color: rgb(255,250,250)");
            // layout setting
            hbox->setSpacing(10);
            hbox->setAlignment(Qt::AlignHCenter);
            vbox->setAlignment(Qt::AlignTop);
            vbox->setContentsMargins(50, 35, 50, 35);
            vbox->setSpacing(36);
            // select box
            QLabel *label_select = new QLabel("Sensor Type", win);
            box_select = new QComboBox(win);
            box_select->setMinimumWidth(400);
            box_select->setStyleSheet("background-color: rgb(155,205,155)");
            for (auto item = sensors_types.begin(); item != sensors_types.end(); item++)
            {
                box_select->addItem(&(*item->c_str()));
            }
            hbox->addWidget(label_select);
            hbox->addWidget(box_select);
            vbox->addLayout(hbox);
            // save box
            QHBoxLayout *save_box = new QHBoxLayout();
            save_box->setAlignment(Qt::AlignHCenter);
            // load button
            load_button = new QPushButton("Load", win);
            load_button->setStyleSheet("background-color: rgb(159,231,167); font-weight: bold; font-size: 16pt");
            load_button->setMinimumSize(150, 40);
            // export button
            export_button = new QPushButton("Export", win);
            export_button->setStyleSheet("background-color: rgb(176,208,238); font-weight: bold; font-size: 16pt");
            export_button->setMinimumSize(150, 40);
            // save button
            save_button = new QPushButton("Save", win);
            save_button->setStyleSheet("background-color: rgb(180,180,241); font-weight: bold; font-size: 16pt");
            save_button->setMinimumSize(150, 40);
            save_box->addWidget(load_button);
            save_box->addStretch(1);
            save_box->addWidget(save_button);
            save_box->addStretch(1);
            save_box->addWidget(export_button);
            vbox->addStretch(7);
            vbox->addLayout(save_box);

            // set layout
            win->setLayout(vbox);

            // connet sigal and slot function
            QObject::connect(box_select, &QComboBox::currentTextChanged, this, &GeneratorSensorsWindow::update_win);
            QObject::connect(save_button, &QPushButton::clicked, this, &GeneratorSensorsWindow::save_slot);
            QObject::connect(export_button, &QPushButton::clicked, this, &GeneratorSensorsWindow::export_slot);
            QObject::connect(load_button, &QPushButton::clicked, this, &GeneratorSensorsWindow::load_slot);
        }

        void update_win()
        {
            // get select text
            string select_text = box_select->currentText().toStdString();
            // clear window
            clear_win(win->layout(), 1);
            if (select_text != sensors_types[0])
            {
                // get select text data
                sensor_data *select_data;
                for (auto item = sensors_data.begin(); item != sensors_data.end(); item++)
                {
                    if ((*item)->name == select_text)
                    {
                        select_data = *item;
                        break;
                    }
                }
                QHBoxLayout *pos_pose_box = new QHBoxLayout();
                QVBoxLayout *pos_vbox = new QVBoxLayout();
                QHBoxLayout *pos_hbox_1 = new QHBoxLayout();
                QHBoxLayout *pos_hbox_2 = new QHBoxLayout();
                QHBoxLayout *pos_hbox_3 = new QHBoxLayout();
                QVBoxLayout *pose_vbox = new QVBoxLayout();
                QHBoxLayout *pose_hbox_1 = new QHBoxLayout();
                QHBoxLayout *pose_hbox_2 = new QHBoxLayout();
                QHBoxLayout *pose_hbox_3 = new QHBoxLayout();
                pos_vbox->setSpacing(10);
                pose_vbox->setSpacing(10);
                QGroupBox *pos_box = new QGroupBox("[Position (m)]", win);
                QGroupBox *pose_box = new QGroupBox("[Pose (degree)]", win);
                QLabel *pos_label_1 = new QLabel("x", win);
                QLabel *pos_label_2 = new QLabel("y", win);
                QLabel *pos_label_3 = new QLabel("z", win);
                QLabel *pose_label_1 = new QLabel("R", win);
                QLabel *pose_label_2 = new QLabel("P", win);
                QLabel *pose_label_3 = new QLabel("Y", win);
                pos_hbox_1->addWidget(pos_label_1);
                pos_hbox_2->addWidget(pos_label_2);
                pos_hbox_3->addWidget(pos_label_3);
                pose_hbox_1->addWidget(pose_label_1);
                pose_hbox_2->addWidget(pose_label_2);
                pose_hbox_3->addWidget(pose_label_3);
                pos_txt_1 = new QLineEdit(select_data->position[0].c_str(), win);
                pos_txt_2 = new QLineEdit(select_data->position[1].c_str(), win);
                pos_txt_3 = new QLineEdit(select_data->position[2].c_str(), win);
                pose_txt_1 = new QLineEdit(select_data->pose[0].c_str(), win);
                pose_txt_2 = new QLineEdit(select_data->pose[1].c_str(), win);
                pose_txt_3 = new QLineEdit(select_data->pose[2].c_str(), win);
                pos_txt_1->setStyleSheet("background-color: white");
                pos_txt_2->setStyleSheet("background-color: white");
                pos_txt_3->setStyleSheet("background-color: white");
                pos_txt_1->setMaximumHeight(textedit_max_height);
                pos_txt_2->setMaximumHeight(textedit_max_height);
                pos_txt_3->setMaximumHeight(textedit_max_height);
                pose_txt_1->setStyleSheet("background-color: white");
                pose_txt_2->setStyleSheet("background-color: white");
                pose_txt_3->setStyleSheet("background-color: white");
                pose_txt_1->setMaximumHeight(textedit_max_height);
                pose_txt_2->setMaximumHeight(textedit_max_height);
                pose_txt_3->setMaximumHeight(textedit_max_height);
                pos_hbox_1->addWidget(pos_txt_1);
                pos_hbox_2->addWidget(pos_txt_2);
                pos_hbox_3->addWidget(pos_txt_3);
                pose_hbox_1->addWidget(pose_txt_1);
                pose_hbox_2->addWidget(pose_txt_2);
                pose_hbox_3->addWidget(pose_txt_3);
                pos_vbox->addLayout(pos_hbox_1);
                pos_vbox->addLayout(pos_hbox_2);
                pos_vbox->addLayout(pos_hbox_3);
                pose_vbox->addLayout(pose_hbox_1);
                pose_vbox->addLayout(pose_hbox_2);
                pose_vbox->addLayout(pose_hbox_3);
                pos_box->setLayout(pos_vbox);
                pose_box->setLayout(pose_vbox);

                pos_pose_box->addWidget(pos_box);
                pos_pose_box->addWidget(pose_box);
                vbox->addLayout(pos_pose_box);

                // lidar
                if (select_text == sensors_types[1])
                {
                    QHBoxLayout *sample_box = new QHBoxLayout();
                    QHBoxLayout *angle_box = new QHBoxLayout();
                    QLabel *lidar_samples_label = new QLabel("[Samples]", win);
                    QLabel *lidar_resolution_label = new QLabel("[Resolution]", win);
                    QLabel *lidar_min_angle_label = new QLabel("[Min Angle]", win);
                    QLabel *lidar_max_angle_label = new QLabel("[Max Angle]", win);
                    lidar_samples = new QLineEdit(select_data->samples.c_str(), win);
                    lidar_resolution = new QLineEdit(select_data->resolution.c_str(), win);
                    lidar_min_angle = new QLineEdit(select_data->min_angle.c_str(), win);
                    lidar_max_angle = new QLineEdit(select_data->max_angle.c_str(), win);
                    lidar_samples->setMaximumHeight(textedit_max_height);
                    lidar_samples->setStyleSheet("background-color: white");
                    lidar_resolution->setMaximumHeight(textedit_max_height);
                    lidar_resolution->setStyleSheet("background-color: white");
                    lidar_min_angle->setMaximumHeight(textedit_max_height);
                    lidar_min_angle->setStyleSheet("background-color: white");
                    lidar_max_angle->setMaximumHeight(textedit_max_height);
                    lidar_max_angle->setStyleSheet("background-color: white");
                    // set layout
                    sample_box->addWidget(lidar_samples_label);
                    sample_box->addWidget(lidar_samples);
                    sample_box->addWidget(lidar_resolution_label);
                    sample_box->addWidget(lidar_resolution);
                    angle_box->addWidget(lidar_min_angle_label);
                    angle_box->addWidget(lidar_min_angle);
                    angle_box->addWidget(lidar_max_angle_label);
                    angle_box->addWidget(lidar_max_angle);
                    vbox->addLayout(sample_box);
                    vbox->addLayout(angle_box);
                    // connet sigal and slot function
                    QObject::connect(lidar_samples, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
                    QObject::connect(lidar_resolution, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
                    QObject::connect(lidar_min_angle, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
                    QObject::connect(lidar_max_angle, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
                }
                // cameras
                else
                {
                    QHBoxLayout *cam_box = new QHBoxLayout();
                    QLabel *cam_resolution = new QLabel("[Resolution]", win);
                    QLabel *cam_width_label = new QLabel("Width", win);
                    QLabel *cam_height_label = new QLabel("Height", win);
                    cam_width = new QLineEdit(select_data->width.c_str(), win);
                    cam_height = new QLineEdit(select_data->height.c_str(), win);
                    cam_width->setMaximumHeight(textedit_max_height);
                    cam_height->setMaximumHeight(textedit_max_height);
                    cam_width->setStyleSheet("background-color: white");
                    cam_height->setStyleSheet("background-color: white");
                    cam_box->addWidget(cam_resolution);
                    cam_box->addWidget(cam_width_label);
                    cam_box->addWidget(cam_width);
                    cam_box->addWidget(cam_height_label);
                    cam_box->addWidget(cam_height);
                    vbox->addLayout(cam_box);
                    // connet sigal and slot function
                    QObject::connect(cam_height, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
                    QObject::connect(cam_width, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
                    if (select_text == "Stereo Camera")
                    {
                        QHBoxLayout *dis_box = new QHBoxLayout();
                        QLabel *dis_label = new QLabel("[Distance (m)]");
                        stereo_cam_distance = new QLineEdit(select_data->distance.c_str(), win);
                        stereo_cam_distance->setMaximumHeight(textedit_max_height);
                        stereo_cam_distance->setStyleSheet("background-color: white");
                        dis_box->addWidget(dis_label);
                        dis_box->addWidget(stereo_cam_distance);
                        vbox->addLayout(dis_box);
                        // connet sigal and slot function
                        QObject::connect(stereo_cam_distance, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
                    }
                }
                // update rate box
                QHBoxLayout *update_box = new QHBoxLayout();
                QLabel *update_label = new QLabel("[Update Rate]", win);
                update_rate = new QLineEdit(select_data->update_rate.c_str(), win);
                update_rate->setMaximumHeight(textedit_max_height);
                update_rate->setStyleSheet("background-color: white");
                update_box->addWidget(update_label);
                update_box->addWidget(update_rate);
                vbox->addLayout(update_box);
                QObject::connect(update_rate, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
                if (select_text == "Stereo Camera")
                {
                    vbox->addStretch(3);
                }
                else
                {
                    vbox->addStretch(4);
                }
            }
            else
            {
                vbox->addStretch(7);
            }
            // save box
            QHBoxLayout *save_box = new QHBoxLayout();
            save_box->setAlignment(Qt::AlignHCenter);
            // load button
            load_button = new QPushButton("Load", win);
            load_button->setStyleSheet("background-color: rgb(159,231,167); font-weight: bold; font-size: 16pt");
            load_button->setMinimumSize(150, 40);
            // export button
            export_button = new QPushButton("Export", win);
            export_button->setStyleSheet("background-color: rgb(176,208,238); font-weight: bold; font-size: 16pt");
            export_button->setMinimumSize(150, 40);
            // save button
            save_button = new QPushButton("Save", win);
            save_button->setStyleSheet("background-color: rgb(180,180,241); font-weight: bold; font-size: 16pt");
            save_button->setMinimumSize(150, 40);
            save_box->addWidget(load_button);
            save_box->addStretch(1);
            save_box->addWidget(save_button);
            save_box->addStretch(1);
            save_box->addWidget(export_button);
            vbox->addStretch(7);
            vbox->addLayout(save_box);
            
            // connet sigal and slot function
            QObject::connect(pos_txt_1, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
            QObject::connect(pos_txt_2, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
            QObject::connect(pos_txt_3, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
            QObject::connect(pose_txt_1, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
            QObject::connect(pose_txt_2, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
            QObject::connect(pose_txt_3, &QLineEdit::textChanged, this, &GeneratorSensorsWindow::update_data);
            QObject::connect(save_button, &QPushButton::clicked, this, &GeneratorSensorsWindow::save_slot);
            QObject::connect(export_button, &QPushButton::clicked, this, &GeneratorSensorsWindow::export_slot);
            QObject::connect(load_button, &QPushButton::clicked, this, &GeneratorSensorsWindow::load_slot);
        }

        void clear_win(QLayout *layout, int last_index = 0, int first_index = 0)
        {
            int num = layout->count();
            num = num - 1;
            int origin_first_index = num;
            for (int i = num; i >= 0; i--)
            {
                if (i > (origin_first_index - first_index))
                {
                    continue;
                }
                if (i < last_index)
                {
                    return;
                }
                QLayoutItem *item = layout->itemAt(i);
                if (item->layout())
                {
                    clear_win(item->layout());
                }
                if (item->widget())
                {
                    item->widget()->deleteLater();
                }
                if (item->spacerItem())
                {
                    layout->removeItem(item);
                }
            }
        }

        // slot functions
        void update_data()
        {
            string select_text = box_select->currentText().toStdString();
            if (select_text == sensors_types[0])
            {
                return;
            }
            // select data
            sensor_data *select_data;
            for (auto item = sensors_data.begin(); item != sensors_data.end(); item++)
            {
                if ((*item)->name == select_text)
                {
                    select_data = *item;
                    break;
                }
            }
            // get data
            string x = pos_txt_1->text().toStdString();
            string y = pos_txt_2->text().toStdString();
            string z = pos_txt_3->text().toStdString();
            string R = pose_txt_1->text().toStdString();
            string P = pose_txt_2->text().toStdString();
            string Y = pose_txt_3->text().toStdString();
            // check data
            if (!check_input_data(x, 0) || 
                !check_input_data(y, 0) || 
                !check_input_data(z, 0) || 
                !check_input_data(R, 0) || 
                !check_input_data(P, 0) || 
                !check_input_data(Y, 0))
            {
                recover_data(select_data);
                return;
            }
            // lidar data
            if (select_text == sensors_types[1])
            {
                // get data
                string lidar_resolution_data = lidar_resolution->text().toStdString();
                string lidar_samples_data = lidar_samples->text().toStdString();
                string lidar_min_angle_data = lidar_min_angle->text().toStdString();
                string lidar_max_angle_data = lidar_max_angle->text().toStdString();
                string lidar_update_rate_data = update_rate->text().toStdString();
                // check data
                if (!check_input_data(lidar_resolution_data, 0) || 
                    !check_input_data(lidar_samples_data, 1) || 
                    !check_input_data(lidar_min_angle_data, 0) || 
                    !check_input_data(lidar_max_angle_data, 0) || 
                    !check_input_data(lidar_update_rate_data, 1))
                {
                    recover_data(select_data);
                    return;
                }
                // save data
                select_data->resolution = lidar_resolution_data;
                select_data->samples = lidar_samples_data;
                select_data->min_angle = lidar_min_angle_data;
                select_data->max_angle = lidar_max_angle_data;
                select_data->update_rate = lidar_update_rate_data;
            }
            else
            {
                // get data
                string cam_width_data = cam_width->text().toStdString();
                string cam_height_data = cam_width->text().toStdString();
                string cam_update_rate_data = update_rate->text().toStdString();
                // check data
                if (!check_input_data(cam_width_data, 1) || 
                    !check_input_data(cam_height_data, 1) || 
                    !check_input_data(cam_update_rate_data, 1))
                {
                    recover_data(select_data);
                    return;
                }
                // save data
                select_data->width = cam_width_data;
                select_data->height = cam_height_data;
                select_data->update_rate = cam_update_rate_data;
                // Stereo Camera have additional distance parameter
                if (select_text == "Stereo Camera")
                {
                    // get data
                    string distance_data = stereo_cam_distance->text().toStdString();
                    // check data
                    if (!check_input_data(distance_data, 1))
                    {
                        recover_data(select_data);
                        return;
                    }
                    // save data
                    select_data->distance = distance_data;
                }
            }
            // save position data
            select_data->position[0] = x;
            select_data->position[1] = y;
            select_data->position[2] = z;
            // save pose data
            select_data->pose[0] = R;
            select_data->pose[1] = P;
            select_data->pose[2] = Y;
        }

        void save_slot()
        {
            sensors_save_data = sensors_data;
            msg_box = new QMessageBox(win);
            msg_box->setIcon(QMessageBox::Icon::Information);
            msg_box->setText("Info");
            msg_box->setWindowTitle("Info");
            msg_box->setText("The Sensors Configuration is Successfully Saved.");
            msg_box->exec();
        }

        void export_slot()
        {
            // get save filename
            QString qfilename = QFileDialog::getSaveFileName(win, "Select Saved Sensors Config File Dir", "", "Json Files (*.json)");
            QStringList qfilename_split = qfilename.split(".");
            string filename = qfilename_split[0].toStdString();
            if (!filename.compare(""))
            {
                return;
            }
            filename = filename + ".json";
            // creat json object
            Json::Value root;
            int num = 0;
            for (auto item = sensors_data.begin(); item < sensors_data.end(); item++)
            {
                root[num]["name"] = (*item)->name;
                root[num]["x"] = (*item)->position[0];
                root[num]["y"] = (*item)->position[1];
                root[num]["z"] = (*item)->position[2];
                root[num]["R"] = (*item)->pose[0];
                root[num]["P"] = (*item)->pose[1];
                root[num]["Y"] = (*item)->pose[2];
                root[num]["samples"] = (*item)->samples;
                root[num]["resolution"] = (*item)->resolution;
                root[num]["height"] = (*item)->height;
                root[num]["width"] = (*item)->width;
                root[num]["min_angle"] = (*item)->min_angle;
                root[num]["max_angle"] = (*item)->max_angle;
                root[num]["distance"] = (*item)->distance;
                root[num]["update_rate"] = (*item)->update_rate;
                num++;
            }
            // output json file
            Json::StyledStreamWriter streamWriter;
            ofstream outFile(filename);
            streamWriter.write(outFile, root);
            outFile.close();
            // show info
            msg_box = new QMessageBox();
            msg_box->setIcon(QMessageBox::Icon::Information);
            msg_box->setText("Info");
            msg_box->setWindowTitle("Info");
            msg_box->setText(("The Sensors Configuration Json File is Successfully Saved at " + filename).c_str());
            msg_box->exec();
        }

        void load_slot()
        {
            // show info
            msg_box = new QMessageBox();
            msg_box->setIcon(QMessageBox::Icon::Information);
            msg_box->setText("Info");
            msg_box->setWindowTitle("Info");
            msg_box->setText("Load Sensors Configuration Files Will Replace Origin Configuration.");
            msg_box->exec();
            // get saved filename
            QString qfilename = QFileDialog::getOpenFileName(win, "Select Sensors Config File", "", "Json Files (*.json)");
            QStringList qfilename_split = qfilename.split(".");
            string filename = qfilename_split[0].toStdString();
            filename = filename + ".json";
            // read json file
            Json::Reader json_reader;
            Json::Value root;
            sensor_data load_single_data;
            // err msg
            msg_box = new QMessageBox();
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            // open file
            ifstream infile(filename.c_str(), ios::binary);
            if (!infile.is_open())
            {
                msg_box->setText("Cannot find the Sensors Configuration Json File.");
                msg_box->exec();
                return;
            }
            // parse file
            if (json_reader.parse(infile, root))
            {
                if (!root.isArray() || (root.size() != (sensors_types.size() - 1)))
                {
                    msg_box->setText("The Sensors Configuration Json File is Bad, please check the file.");
                    msg_box->exec();
                    return;
                }
                for (unsigned int i = 0; i < root.size(); i++)
                {
                    string name = root[i]["name"].asString();
                    if ((std::find(sensors_types.begin(), sensors_types.end(), name) == sensors_types.end()))
                    {
                        msg_box->setText("The Sensors Configuration Json File is Bad, please check the file.");
                        msg_box->exec();
                        return;
                    }
                    string x = root[i]["x"].asString();
                    string y = root[i]["y"].asString();
                    string z = root[i]["z"].asString();
                    string R = root[i]["R"].asString();
                    string P = root[i]["P"].asString();
                    string Y = root[i]["Y"].asString();
                    string distance = root[i]["distance"].asString();
                    string width = root[i]["width"].asString();
                    string height = root[i]["height"].asString();
                    string max_angle = root[i]["max_angle"].asString();
                    string min_angle = root[i]["min_angle"].asString();
                    string resolution = root[i]["resolution"].asString();
                    string samples = root[i]["samples"].asString();
                    string update_rate = root[i]["update_rate"].asString();
                    // check data
                    if (!check_input_data(x, 0) ||
                        !check_input_data(y, 0) ||
                        !check_input_data(z, 0) ||
                        !check_input_data(R, 0) ||
                        !check_input_data(P, 0) ||
                        !check_input_data(Y, 0) ||
                        !check_input_data(distance, 0) ||
                        !check_input_data(width, 1) ||
                        !check_input_data(height, 1) ||
                        !check_input_data(max_angle, 0) ||
                        !check_input_data(min_angle, 0) ||
                        !check_input_data(resolution, 0) ||
                        !check_input_data(samples, 1) ||
                        !check_input_data(update_rate, 1))
                    {
                        msg_box->setText("The Sensors Configuration Json File is Bad, please check the file.");
                        msg_box->exec();
                        return;
                    }
                    load_single_data.name = name;
                    load_single_data.position = {x, y, z};
                    load_single_data.pose = {R, P, Y};
                    load_single_data.samples = samples;
                    load_single_data.resolution = resolution;
                    load_single_data.update_rate = update_rate;
                    load_single_data.width = width;
                    load_single_data.height = height;
                    load_single_data.min_angle = min_angle;
                    load_single_data.max_angle = max_angle;
                    load_single_data.distance = distance;
                    load_data.push_back(load_single_data);
                }
            }
            // save to class var
            for (int i = 0; i < load_data.size(); i++)
            {
                sensors_data[i] = &load_data[i];
            }
            sensors_save_data = sensors_data;
            // show info
            msg_box->setIcon(QMessageBox::Icon::Information);
            msg_box->setText("Info");
            msg_box->setWindowTitle("Info");
            msg_box->setText("The Sensors Configuration Json File is Successfully Loaded.");
            msg_box->exec();
        }

        // utility function
        bool check_input_data(string data, int data_type = 0, string error_msg = "")
        {
            string err = "";
            msg_box = new QMessageBox(win);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            if (!data.compare(""))
            {
                err = "Parameter [" + error_msg + "] Can not Be Empty!";
                msg_box->setText(err.c_str());
                msg_box->exec();
                return false;
            }
            // int
            if (data_type == 0)
            {
                try
                {
                    stof(data);
                }
                catch (const std::exception &e)
                {
                    err = "Parameter [" + error_msg + "] Only Support Int Type!";
                    msg_box->setText(err.c_str());
                    msg_box->exec();
                    return false;
                }
            }
            // double
            if (data_type == 1)
            {
                try
                {
                    stoi(data);
                }
                catch (const std::exception &e)
                {
                    err = "Parameter [" + error_msg + "] Only Support Float/Double Type!";
                    msg_box->setText(err.c_str());
                    msg_box->exec();
                    return false;
                }
            }
            return true;
        }

        // recover data
        void recover_data(sensor_data* recover_data)
        {
            pos_txt_1->setText(recover_data->position[0].c_str());
            pos_txt_2->setText(recover_data->position[1].c_str());
            pos_txt_3->setText(recover_data->position[2].c_str());
            pose_txt_1->setText(recover_data->pose[0].c_str());
            pose_txt_2->setText(recover_data->pose[1].c_str());
            pose_txt_3->setText(recover_data->pose[2].c_str());
            update_rate->setText(recover_data->update_rate.c_str());
            // lidar
            if (recover_data->name == sensors_types[1])
            {
                lidar_samples->setText(recover_data->samples.c_str());
                lidar_resolution->setText(recover_data->resolution.c_str());
                lidar_min_angle->setText(recover_data->min_angle.c_str());
                lidar_max_angle->setText(recover_data->max_angle.c_str());
            }
            // camera
            else
            {
                cam_width->setText(recover_data->width.c_str());
                cam_height->setText(recover_data->height.c_str());
                if (recover_data->name == "Stereo Camera")
                {
                    stereo_cam_distance->setText(recover_data->distance.c_str());
                }
            }
        }
};
#endif