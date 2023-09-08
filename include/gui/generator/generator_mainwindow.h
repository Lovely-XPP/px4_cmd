#ifndef GENERATORMAINWINDOW_H
#define GENERATORMAINWINDOW_H
#include <QMainWindow>
#include <QApplication>
#include <QDialog>
#include <QMessageBox>
#include <QLabel>
#include <QFrame>
#include <QIcon>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QTableView>
#include <QPushButton>
#include <QHeaderView>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QFileDialog>
#include <QGraphicsOpacityEffect>
#include <string>
#include <vector>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>

#include <gui/generator/generator_infowindow.h>
#include <gui/generator/generator_sensorswindow.h>
#include <print_utility/printf_utility.h>
#include <tinyxml2.h>

#define PI 3.14159265358979323846

using namespace std;
using namespace tinyxml2;

class GeneratorMainWindow : public QWidget
{

    public:
        QDialog *win = new QDialog();
        QWidget *parent;
        GeneratorInfoWindow *info_win = new GeneratorInfoWindow(win);
        GeneratorSensorsWindow *sensors_win = new GeneratorSensorsWindow(win);
        GeneratorMainWindow(QWidget *parent_widget)
        {
            setup();
        }

    private:
        // settings
        string version = "V1.1.6";
        int local_port = 34580;
        int remote_port = 14540;
        int sitl_port = 24560;
        int tcp_port = 4560;
        vector<string> topic_types = {"{Vehicle Type}_{ID}", "uav_{ID}"};
        vector<string> vehicle_types = {"iris", "typhoon_h480", "plane"};
        vector<string> sensor_types = {"None", "Lidar", "Depth Camera", "RGB Camera", "Stereo Camera", "Realsense Camera"};
        QStringList table_headers = {"Vehicle", "Sensor", "x", "y", "z", "R", "P", "Y"};

        // init vector
        vector<string> vehicles = {};
        vector<string> sensors = {};
        vector<vector<string>> init_pos;
        vector<string> world_files = {};
        vector<sensor_data *> sensor_datas = {};
        // output file string
        string output_file = "";
        string sim_dir = "";
        bool update_signal = false;

        //Widgets
        QMessageBox *msg_box;
        QComboBox *vehicles_list;
        QComboBox *sensors_list;
        QComboBox *worlds_list;
        QComboBox *topics_list;
        QLineEdit *txt_x;
        QLineEdit *txt_y;
        QLineEdit *txt_z;
        QLineEdit *txt_R;
        QLineEdit *txt_P;
        QLineEdit *txt_Y;
        QLineEdit *txt_dir;
        QLineEdit *txt_sim_dir;
        QTableView *table;
        QStandardItemModel *model;
        QPushButton *add_button;
        QPushButton *del_button;
        QPushButton *clc_button;
        QPushButton *generate_button;
        QPushButton *sensors_set_button;
        QPushButton *load_button;
        QPushButton *info_button;
        QPushButton *browse_button;
        QPushButton *sim_browse_button;
        QCheckBox *gazebo_gui_checkbox;
        QCheckBox *defalut_sim_dir;

        void setup()
        {
            QIcon *icon = new QIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
            win->setWindowIcon(*icon);
            win->setFixedSize(1080, 750);
            win->setWindowTitle(("PX4 Cmd Launch File Generator [Version: " + version + "]").c_str());
            win->setStyleSheet("background-color: rgb(255,250,250)");
            string err = detect_env();
            if (err.length() != 0)
            {
                msg_box = new QMessageBox(win);
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setWindowTitle("Error");
                msg_box->setText(err.c_str());
                exit(msg_box->exec());
            }
            sensor_datas = sensors_win->sensors_save_data;
            get_world_files();
            add_list();
            add_table();
            add_init_pos_input();
            add_buttons();
            add_browse_dir();
            add_checkbox();
            QObject::connect(browse_button, &QPushButton::clicked, this, &GeneratorMainWindow::browse_dir_slot);
            QObject::connect(sim_browse_button, &QPushButton::clicked, this, &GeneratorMainWindow::sim_browse_dir_slot);
            QObject::connect(add_button, &QPushButton::clicked, this, &GeneratorMainWindow::add_vehicle_slot);
            QObject::connect(model, &QStandardItemModel::itemChanged, this, &GeneratorMainWindow::update_edit_table_slot);
            QObject::connect(del_button, &QPushButton::clicked, this, &GeneratorMainWindow::del_vehicle_slot);
            QObject::connect(clc_button, &QPushButton::clicked, this, &GeneratorMainWindow::clc_vehicles_slot);
            QObject::connect(info_button, &QPushButton::clicked, this, &GeneratorMainWindow::info_window_slot);
            QObject::connect(sensors_set_button, &QPushButton::clicked, this, &GeneratorMainWindow::sensor_window_slot);
            QObject::connect(generate_button, &QPushButton::clicked, this, &GeneratorMainWindow::generate_launch_slot);
            QObject::connect(load_button, &QPushButton::clicked, this, &GeneratorMainWindow::load_launch_slot);
            QObject::connect(defalut_sim_dir, &QCheckBox::stateChanged, this, &GeneratorMainWindow::defalut_sim_dir_slot);
        }

        string detect_env()
        {
            string error_msg = "";
            string error_header = "This Programme requires ROS enviroment and PX4 installation and px4_cmd ROS Package.\n";
            string res = "";
            get_cmd_output("which rospack", res);
            strip(res, "\n");
            if (res.length() == 0)
            {
                error_msg = error_header + "Please Check ROS Installation & rospack command.";
                return error_msg;
            }
            res.clear();
            get_cmd_output("rospack list-names", res);
            strip(res, "\n");
            if (res.find("px4") == res.npos)
            {
                error_msg = error_header + "Please Check PX4 ROS Package Installation.";
                return error_msg;
            }
            if (res.find("mavlink_sitl_gazebo") == res.npos)
            {
                error_msg = error_header + "Please Check mavlink_sitl_gazebo ROS Package Installation.";
                return error_msg;
            }
            if (res.find("px4_cmd") == res.npos)
            {
                error_msg = error_header + "Please Check px4_cmd ROS Package Installation.\nGithub: https://github.com/Lovely-XPP/PX4_cmd/";
                return error_msg;
            }
            return error_msg;
        }

        void add_list()
        {
            int xshift = -20;
            // vehicles type
            QLabel *label1 = new QLabel("Vehicle Type", win);
            label1->move(50 + xshift, 20);
            label1->resize(150, 35);
            QComboBox *list1 = new QComboBox(win);
            for (auto item = vehicle_types.begin(); item != vehicle_types.end(); item++)
            {
                list1->addItem(&(*item->c_str()));
            }
            list1->move(150 + xshift, 20);
            list1->resize(380, 35);
            list1->setStyleSheet("background-color: rgb(135,206,235)");
            vehicles_list = list1;

            // sensors type
            QLabel *label2 = new QLabel("Sensor Type", win);
            label2->move(580 + xshift, 20);
            label2->resize(150, 35);
            QComboBox *list2 = new QComboBox(win);
            for (auto item = sensor_types.begin(); item != sensor_types.end(); item++)
            {
                list2->addItem(&(*item->c_str()));
            }
            list2->move(680 + xshift, 20);
            list2->resize(390, 35);
            list2->setStyleSheet("background-color: rgb(155,205,155)");
            sensors_list = list2;

            // worlds file
            QLabel *label3 = new QLabel("World Files", win);
            label3->move(50 + xshift, 70);
            label3->resize(150, 35);
            QComboBox *list3 = new QComboBox(win);
            for (auto item = world_files.begin(); item != world_files.end(); item++)
            {
                list3->addItem(&(*item->c_str()));
            }
            list3->setCurrentText("empty.world");
            list3->move(150 + xshift, 70);
            list3->resize(380, 35);
            list3->setStyleSheet("background-color: rgb(135,206,235)");
            worlds_list = list3;

            // topic_type
            QLabel *label4 = new QLabel("Topic Type", win);
            label4->move(580 + xshift, 70);
            label4->resize(150, 35);
            QComboBox *list4 = new QComboBox(win);
            for (auto item = topic_types.begin(); item != topic_types.end(); item++)
            {
                list4->addItem(&(*item->c_str()));
            }
            list4->move(680 + xshift, 70);
            list4->resize(390, 35);
            list4->setStyleSheet("background-color: rgb(155,205,155)");
            list4->setCurrentText("uav_{ID}");
            topics_list = list4;
        }

        void add_table()
        {
            float xshift = -20;
            QTableView *ftable = new QTableView(win);
            ftable->move(50 + xshift, 360);
            ftable->resize(1020, 360);
            ftable->setStyleSheet("background-color: white");
            QStandardItemModel *fmodel = new QStandardItemModel(0, table_headers.size(), win);
            fmodel->setHorizontalHeaderLabels(table_headers);
            ftable->setModel(fmodel);
            ftable->horizontalHeader()->setStretchLastSection(true);
            ftable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
            table = ftable;
            model = fmodel;
        }

        void add_init_pos_input()
        {
            int yshift = 10 + 50;
            int x0 = 50;
            int dx1 = 20;
            int dx2 = 15;
            int x1 = 95;
            int x = x0;

            QFrame *label00 = new QFrame(win);
            QGraphicsOpacityEffect *op = new QGraphicsOpacityEffect;
            op->setOpacity(0.3);
            label00->setFrameShape(QFrame::Box);
            label00->setGraphicsEffect(op);
            label00->move(30, 63 + yshift);
            label00->resize(800, 75);
            QLabel *label0 = new QLabel("Initial Position", win);
            label0->move(380, 55 + yshift);
            label0->resize(100, 35);
            label0->setAlignment(Qt::AlignHCenter);

            // x
            // label
            QLabel *label1 = new QLabel("x", win);
            label1->move(x, 85 + yshift);
            label1->resize(30, 35);
            // text
            QLineEdit *text1 = new QLineEdit("0.0", win);
            text1->setReadOnly(false);
            x = x + dx2;
            text1->move(x, 85 + yshift);
            text1->resize(x1, 35);
            text1->setStyleSheet("background-color: white");
            txt_x = text1;

            // y
            // label
            QLabel *label2 = new QLabel("y", win);
            x = x + x1 + dx1;
            label2->move(x, 85 + yshift);
            label2->resize(30, 35);
            // text
            QLineEdit *text2 = new QLineEdit("0.0", win);
            text2->setReadOnly(false);
            x = x + dx2;
            text2->move(x, 85 + yshift);
            text2->resize(x1, 35);
            text2->setStyleSheet("background-color: white");
            txt_y = text2;

            // z
            // label
            QLabel *label3 = new QLabel("z", win);
            x = x + x1 + dx1;
            label3->move(x, 85 + yshift);
            label3->resize(30, 35);
            // text
            QLineEdit *text3 = new QLineEdit("0.0", win);
            text3->setReadOnly(false);
            x = x + dx2;
            text3->move(x, 85 + yshift);
            text3->resize(x1, 35);
            text3->setStyleSheet("background-color: white");
            txt_z = text3;

            // R
            // label
            QLabel *label4 = new QLabel("R", win);
            x = x + x1 + dx1;
            label4->move(x, 85 + yshift);
            label4->resize(30, 35);
            // text
            QLineEdit *text4 = new QLineEdit("0.0", win);
            text4->setReadOnly(false);
            x = x + dx2;
            text4->move(x, 85 + yshift);
            text4->resize(x1, 35);
            text4->setStyleSheet("background-color: white");
            txt_R = text4;

            // P
            // label
            QLabel *label5 = new QLabel("P", win);
            x = x + x1 + dx1;
            label5->move(x, 85 + yshift);
            label5->resize(30, 35);
            // text
            QLineEdit *text5 = new QLineEdit("0.0", win);
            text5->setReadOnly(false);
            x = x + dx2;
            text5->move(x, 85 + yshift);
            text5->resize(x1, 35);
            text5->setStyleSheet("background-color: white");
            txt_P = text5;

            // Y
            // label
            QLabel *label6 = new QLabel("P", win);
            x = x + x1 + dx1;
            label6->move(x, 85 + yshift);
            label6->resize(30, 35);
            // text
            QLineEdit *text6 = new QLineEdit("0.0", win);
            text6->setReadOnly(false);
            x = x + dx2;
            text6->move(x, 85 + yshift);
            text6->resize(x1, 35);
            text6->setStyleSheet("background-color: white");
            txt_Y = text6;
        }

        void add_buttons()
        {
            float xshift = 75;
            // add button 
            QPushButton *button1 = new QPushButton("Add Vehicle", win);
            button1->move(30, 305);
            button1->resize(190, 40);
            button1->setStyleSheet("background-color: rgb(50,191,255); font-size: 13pt");
            add_button = button1;

            // del vehicle button
            QPushButton *button2 = new QPushButton("Del Vehicle", win);
            button2->move(235, 305);
            button2->resize(190, 40);
            button2->setStyleSheet("background-color: rgb(255,106,106); font-size: 13pt");
            del_button = button2;

            // clear vehicle button
            QPushButton *button3 = new QPushButton("Clear Vehicles", win);
            button3->move(440, 305);
            button3->resize(190, 40);
            button3->setStyleSheet("background-color: rgb(224,102,255); font-size: 13pt");
            clc_button = button3;

            // generate button
            QPushButton *button4 = new QPushButton("Generate Launch", win);
            button4->move(780 + xshift, 214);
            button4->resize(195, 75);
            button4->setStyleSheet("background-color: rgb(84,255,159); font-weight: bold; font-size: 16pt");
            generate_button = button4;

            // sensors setting
            QPushButton *button5 = new QPushButton("Sensors Setting", win);
            button5->move(645, 305);
            button5->resize(195, 40);
            button5->setStyleSheet("background-color: rgb(255,190,155); font-size: 13pt");
            sensors_set_button = button5;

            // load button
            QPushButton *button6 = new QPushButton("Load Launch", win);
            button6->move(780 + xshift, 123);
            button6->resize(195, 75);
            button6->setStyleSheet("background-color: rgb(135,206,235); font-weight: bold; font-size: 16pt");
            load_button = button6;

            // information button
            QPushButton *button7 = new QPushButton("About", win);
            button7->move(855, 305);
            button7->resize(195, 40);
            button7->setStyleSheet("background-color: rgb(255,227,132); font-size: 13pt");
            info_button = button7;
        }

        void add_browse_dir()
        {
            float xshift = -20;
            float yshift = 30 + 42;
            // label
            QLabel *label_1 = new QLabel("Output Dir", win);
            label_1->move(170 + xshift, 142 + yshift);
            label_1->resize(100, 33);
            QLabel *label_2 = new QLabel("Simulation Dir", win);
            label_2->move(148 + xshift, 185 + yshift);
            label_2->resize(100, 33);

            // show dir
            QLineEdit *text_1 = new QLineEdit(win);
            text_1->setReadOnly(true);
            text_1->move(255 + xshift, 142 + yshift);
            text_1->resize(480, 33);
            text_1->setStyleSheet("background-color: white");
            txt_dir = text_1;
            QLineEdit *text_2 = new QLineEdit("~/.ros/[topic type]", win);
            text_2->setReadOnly(true);
            text_2->move(255 + xshift, 185 + yshift);
            text_2->resize(480, 33);
            text_2->setStyleSheet("background-color: white");
            txt_sim_dir = text_2;

            // button
            QPushButton *button_1 = new QPushButton("Browse", win);
            button_1->move(750 + xshift, 142 + yshift);
            button_1->resize(100, 33);
            button_1->setStyleSheet("background-color: rgb(255,235,230)");
            browse_button = button_1;
            QPushButton *button_2 = new QPushButton("Browse", win);
            button_2->move(750 + xshift, 185 + yshift);
            button_2->resize(100, 33);
            button_2->setStyleSheet("background-color: rgb(255,235,230)");
            button_2->setEnabled(false);
            sim_browse_button = button_2;
        }

        void add_checkbox()
        {
            float xshift = -20;
            float yshift = 30 + 42;

            // check box
            gazebo_gui_checkbox = new QCheckBox("Gazebo GUI", win);
            gazebo_gui_checkbox->move(50 + xshift, 148 + yshift);
            gazebo_gui_checkbox->setChecked(true);

            defalut_sim_dir = new QCheckBox("Default", win);
            defalut_sim_dir->move(50 + xshift, 191 + yshift);
            defalut_sim_dir->setChecked(true);
        }

        void get_world_files()
        {
            string world_files_dir;
            get_cmd_output("echo $(rospack find mavlink_sitl_gazebo)/worlds/", world_files_dir);
            strip(world_files_dir, "\n");
            strip(world_files_dir);
            world_files = get_files(world_files_dir);
            sort(world_files.begin(), world_files.end());
        }

        // Slot functions
        void add_vehicle_slot()
        {
            string x = txt_x->text().toStdString();
            if (!check_input_data(x, 0))
            {
                return;
            }
            string y = txt_y->text().toStdString();
            if (!check_input_data(y, 0))
            {
                return;
            }
            string z = txt_z->text().toStdString();
            if (!check_input_data(z, 0))
            {
                return;
            }
            string R = txt_R->text().toStdString();
            if (!check_input_data(R, 0))
            {
                return;
            }
            string P = txt_P->text().toStdString();
            if (!check_input_data(P, 0))
            {
                return;
            }
            string Y = txt_Y->text().toStdString();
            if (!check_input_data(Y, 0))
            {
                return;
            }
            vector<string> pos = {x, y, z, R, P, Y};
            init_pos.push_back(pos);
            vehicles.push_back(vehicles_list->currentText().toStdString());
            sensors.push_back(sensors_list->currentText().toStdString());
            // update table
            update_signal = true;
            QStandardItem *item_1 = new QStandardItem();
            QStandardItem *item_2 = new QStandardItem();
            QStandardItem *item_3 = new QStandardItem();
            QStandardItem *item_4 = new QStandardItem();
            QStandardItem *item_5 = new QStandardItem();
            QStandardItem *item_6 = new QStandardItem();
            QStandardItem *item_7 = new QStandardItem();
            QStandardItem *item_8 = new QStandardItem();
            item_1->setText(vehicles[(vehicles.size() - 1)].c_str());
            item_2->setText(sensors[(sensors.size() - 1)].c_str());
            item_3->setText(init_pos[(init_pos.size() - 1)][0].c_str());
            item_4->setText(init_pos[(init_pos.size() - 1)][1].c_str());
            item_5->setText(init_pos[(init_pos.size() - 1)][2].c_str());
            item_6->setText(init_pos[(init_pos.size() - 1)][3].c_str());
            item_7->setText(init_pos[(init_pos.size() - 1)][4].c_str());
            item_8->setText(init_pos[(init_pos.size() - 1)][5].c_str());
            item_1->setEditable(false);
            item_2->setEditable(false);
            model->appendRow({item_1, item_2, item_3, item_4, item_5, item_6, item_7, item_8});
            update_signal = false;
        }

        void browse_dir_slot()
        {
            QString qfilename = QFileDialog::getSaveFileName(win, "Select Saved Launch File", "", "Launch Files (*.launch)");
            auto qfilename_split = qfilename.split(".");
            string filename = qfilename_split[0].toStdString();
            if (!filename.compare(""))
            {
                txt_dir->setText("");
                output_file = "";
                return;
            }
            filename = filename + ".launch";
            txt_dir->setText(filename.c_str());
            output_file = filename;
        }

        void sim_browse_dir_slot()
        {
            QString qfilename = QFileDialog::getExistingDirectory(win, "Select Simulation Data Directory", "");
            auto qfilename_split = qfilename.split(".");
            string filename = qfilename_split[0].toStdString();
            if (!filename.compare(""))
            {
                txt_sim_dir->setText("");
                sim_dir = "";
                return;
            }
            txt_sim_dir->setText((filename + "/[topic type]").c_str());
            sim_dir = filename;
        }

        void update_edit_table_slot()
        {
            if (update_signal)
            {
                return;
            }
            int row = table->currentIndex().row();
            int column = table->currentIndex().column();
            QVariant data = table->currentIndex().data();
            string str = data.toString().toStdString();
            if (check_input_data(str, 0))
            {
                init_pos[row][column - 2] = str;
            }
            else
            {
                update_signal = true;
                QStandardItem *item = new QStandardItem();
                item->setText(init_pos[row][column - 2].c_str());
                item->setTextAlignment(Qt::AlignCenter);
                model->setItem(row, column, item);
                update_signal = false;
            }
        }

        void del_vehicle_slot()
        {
            // get selected index
            QModelIndexList indexes = table->selectionModel()->selectedIndexes();
            if (indexes.length() == 0)
            {
                msg_box = new QMessageBox(win);
                msg_box->setIcon(QMessageBox::Icon::Information);
                msg_box->setText("Info");
                msg_box->setWindowTitle("Info");
                msg_box->setText("Please Select Vehicle");
                msg_box->exec();
                return;
            }
            // delete selected index
            int num = 0;
            int row = 0;
            update_signal = true;
            for (auto item = indexes.begin(); item != indexes.end(); item++)
            {
                QModelIndex i = *item;
                row = i.row();
                row = row - num;
                num++;
                // remove data
                vehicles.erase(vehicles.begin() + row);
                sensors.erase(sensors.begin() + row);
                init_pos.erase(init_pos.begin() + row);
                // remove table row
                model->removeRow(row);
            }
            update_signal = false;
        }

        void clc_vehicles_slot()
        {
            // clear data
            vehicles.clear();
            sensors.clear();
            init_pos.clear();
            // clear table
            update_signal = true;
            model->clear();
            model->setHorizontalHeaderLabels(table_headers);
            update_signal = false;
        }

        void info_window_slot()
        {
            info_win->win->exec();
        }

        void sensor_window_slot()
        {
            if (sensor_datas.size() > 0)
            {
                sensors_win->sensors_save_data = sensor_datas;
                sensors_win->sensors_data = sensor_datas;
            }

            sensors_win->win->exec();
            sensor_datas = sensors_win->sensors_save_data;
        }

        void generate_launch_slot()
        {
            // init messagebox
            msg_box = new QMessageBox();
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            // no vehicles return
            if (vehicles.size() == 0)
            {
                msg_box->setText("Please Add Vehicle First.");
                msg_box->exec();
                return;
            }
            // no output dir return
            if (output_file == "")
            {
                msg_box->setText("Please Browse Output Directory.");
                msg_box->exec();
                return;
            }
            if (!defalut_sim_dir->isChecked() && sim_dir == "")
            {
                msg_box->setText("Please Browse Simulation Data Directory.");
                msg_box->exec();
                return;
            }

            // init new xml
            XMLDocument doc;
            doc.Parse("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<!-- This launch file is generated by PX4 Cmd generator -->");
            XMLElement *root = doc.NewElement("launch");
            doc.InsertEndChild(root);
            // arg config
            // get gui state
            string gazebo_gui = "false";
            if (gazebo_gui_checkbox->isChecked())
            {
                gazebo_gui = "true";
            }
            string world_file = worlds_list->currentText().toStdString();
            XMLElement *arg_1 = doc.NewElement("arg");
            arg_1->SetAttribute("name", "est");
            arg_1->SetAttribute("default", "ekf2");
            root->InsertEndChild(arg_1);
            XMLElement *arg_2 = doc.NewElement("arg");
            arg_2->SetAttribute("name", "world");
            arg_2->SetAttribute("default", ("$(find mavlink_sitl_gazebo)/worlds/" + world_file).c_str());
            root->InsertEndChild(arg_2);
            XMLElement *arg_3 = doc.NewElement("arg");
            arg_3->SetAttribute("name", "gui");
            arg_3->SetAttribute("default", gazebo_gui.c_str());
            root->InsertEndChild(arg_3);
            XMLElement *arg_4 = doc.NewElement("arg");
            arg_4->SetAttribute("name", "debug");
            arg_4->SetAttribute("default", "false");
            root->InsertEndChild(arg_4);
            XMLElement *arg_5 = doc.NewElement("arg");
            arg_5->SetAttribute("name", "verbose");
            arg_5->SetAttribute("default", "false");
            root->InsertEndChild(arg_5);
            XMLElement *arg_6 = doc.NewElement("arg");
            arg_6->SetAttribute("name", "paused");
            arg_6->SetAttribute("default", "false");
            root->InsertEndChild(arg_6);
            XMLElement *arg_7 = doc.NewElement("arg");
            arg_7->SetAttribute("name", "topic_type");
            arg_7->SetAttribute("default", topics_list->currentText().toStdString().c_str());
            root->InsertEndChild(arg_7);
            XMLElement *arg_8 = doc.NewElement("arg");
            arg_8->SetAttribute("name", "interactive");
            arg_8->SetAttribute("default", "true");
            root->InsertEndChild(arg_8);
            XMLElement *arg_9 = doc.NewElement("arg");
            arg_9->SetAttribute("name", "sim_dir");
            if (defalut_sim_dir->isChecked())
            {
                arg_9->SetAttribute("default", "");
            }
            else
            {
                arg_9->SetAttribute("default", sim_dir.c_str());
            }
            root->InsertEndChild(arg_9);

            // gazebo simulation config
            XMLComment *gazebo_comment = doc.NewComment(" gazebo simulation ");
            root->InsertEndChild(gazebo_comment);
            XMLElement *gazebo_sim_config = doc.NewElement("include");
            gazebo_sim_config->SetAttribute("file", "$(find gazebo_ros)/launch/empty_world.launch");
            root->InsertEndChild(gazebo_sim_config);
            XMLElement *gazebo_arg_1 = doc.NewElement("arg");
            gazebo_arg_1->SetAttribute("name", "gui");
            gazebo_arg_1->SetAttribute("value", "$(arg gui)");
            gazebo_sim_config->InsertEndChild(gazebo_arg_1);
            XMLElement *gazebo_arg_2 = doc.NewElement("arg");
            gazebo_arg_2->SetAttribute("name", "world_name");
            gazebo_arg_2->SetAttribute("value", "$(arg world)");
            gazebo_sim_config->InsertEndChild(gazebo_arg_2);
            XMLElement *gazebo_arg_3 = doc.NewElement("arg");
            gazebo_arg_3->SetAttribute("name", "debug");
            gazebo_arg_3->SetAttribute("value", "$(arg debug)");
            gazebo_sim_config->InsertEndChild(gazebo_arg_3);
            XMLElement *gazebo_arg_4 = doc.NewElement("arg");
            gazebo_arg_4->SetAttribute("name", "verbose");
            gazebo_arg_4->SetAttribute("value", "$(arg verbose)");
            gazebo_sim_config->InsertEndChild(gazebo_arg_4);
            XMLElement *gazebo_arg_5 = doc.NewElement("arg");
            gazebo_arg_5->SetAttribute("name", "paused");
            gazebo_arg_5->SetAttribute("value", "$(arg paused)");
            gazebo_sim_config->InsertEndChild(gazebo_arg_5);

            // config for vehicles
            string vehicle;
            string sensor;
            string sensor_folder_name;
            string model;
            string topic_type;
            string topic_name;
            XMLComment *agent_comment;
            XMLElement *agent;
            XMLElement *agent_arg_1;
            XMLElement *agent_arg_2;
            XMLElement *agent_arg_3;
            XMLElement *agent_arg_4;
            XMLElement *agent_arg_5;
            XMLElement *agent_arg_x;
            XMLElement *agent_arg_y;
            XMLElement *agent_arg_z;
            XMLElement *agent_arg_R;
            XMLElement *agent_arg_P;
            XMLElement *agent_arg_Y;
            XMLElement *agent_arg_mavid;
            XMLElement *agent_arg_udp_port;
            XMLElement *agent_arg_tcp_port;
            XMLElement *agent_arg_sdk_port;
            XMLElement *agent_arg_cam_udp_port;
            XMLElement *agent_arg_gst_udp_port;
            XMLElement *agent_arg_video_uri;
            XMLComment *agent_cmd_comment;
            XMLElement *agent_arg_cmd;
            XMLElement *agent_arg_cmd_param;
            XMLElement *agent_arg_vehicle_param;
            XMLElement *agent_arg_sensor_param;
            XMLElement *agent_arg_x_param;
            XMLElement *agent_arg_y_param;
            XMLElement *agent_arg_z_param;
            XMLElement *agent_arg_R_param;
            XMLElement *agent_arg_P_param;
            XMLElement *agent_arg_Y_param;
            XMLComment *px4_comment;
            XMLElement *px4_env_1;
            XMLElement *px4_env_2;
            XMLElement *px4_arg_1;
            XMLElement *px4_arg_2;
            XMLElement *px4_node;
            XMLComment *spawn_model_comment;
            XMLElement *spawn_model;
            XMLElement *mavros;
            XMLComment *mavros_comment;
            XMLElement *mavros_arg_1;
            XMLElement *mavros_arg_2;
            XMLElement *mavros_arg_3;
            XMLElement *mavros_arg_4;
            for (unsigned int i = 0; i < vehicles.size(); i++)
            {
                // get vehicle & sensor data
                topic_type = topics_list->currentText().toStdString();
                vehicle = vehicles[i];
                sensor = sensors[i];
                if (sensor == sensor_types[0])
                {
                    model = vehicle;
                }
                else
                {
                    sensor_folder_name = sensor;
                    for (auto item = sensor_folder_name.begin(); item < sensor_folder_name.end(); item++)
                    {
                        if ((*item) == ' ')
                        {
                            *item = '_';
                        }
                    }
                    transform(sensor_folder_name.begin(), sensor_folder_name.end(), sensor_folder_name.begin(), ::tolower);
                    model = vehicle + "_" + sensor_folder_name;
                    if(!generate_sdf(vehicle, sensor))
                    {
                        return;
                    }
                }
                if (topic_type == topic_types[0])
                {
                    topic_name = vehicle + "_" + to_string(i);
                }
                if (topic_type == topic_types[1])
                {
                    topic_name = "uav_" + to_string(i);
                }
                // init arg
                agent_comment = doc.NewComment(( "uav_" + to_string(i) + ": " + vehicle + " - " + sensor + " ").c_str());
                root->InsertEndChild(agent_comment);
                agent = doc.NewElement("group");
                agent->SetAttribute("ns", topic_name.c_str());
                root->InsertEndChild(agent);
                agent_arg_1 = doc.NewElement("arg");
                agent_arg_1->SetAttribute("name", "vehicle");
                agent_arg_1->SetAttribute("value", vehicle.c_str());
                agent->InsertEndChild(agent_arg_1);
                agent_arg_2 = doc.NewElement("arg");
                agent_arg_2->SetAttribute("name", "sensor");
                agent_arg_2->SetAttribute("value", sensor.c_str());
                agent->InsertEndChild(agent_arg_2);
                agent_arg_3 = doc.NewElement("arg");
                agent_arg_3->SetAttribute("name", "ID");
                agent_arg_3->SetAttribute("value", (to_string(i)).c_str());
                agent->InsertEndChild(agent_arg_3);
                agent_arg_4 = doc.NewElement("arg");
                agent_arg_4->SetAttribute("name", "fcu_url");
                agent_arg_4->SetAttribute("value", ("udp://:" + to_string(remote_port) + "@localhost:" + to_string(local_port)).c_str());
                agent->InsertEndChild(agent_arg_4);
                agent_arg_5 = doc.NewElement("arg");
                agent_arg_5->SetAttribute("name", "model");
                agent_arg_5->SetAttribute("value", model.c_str());
                agent->InsertEndChild(agent_arg_5);
                agent_arg_x = doc.NewElement("arg");
                agent_arg_x->SetAttribute("name", "x");
                agent_arg_x->SetAttribute("value", init_pos[i][0].c_str());
                agent->InsertEndChild(agent_arg_x);
                agent_arg_y = doc.NewElement("arg");
                agent_arg_y->SetAttribute("name", "y");
                agent_arg_y->SetAttribute("value", init_pos[i][1].c_str());
                agent->InsertEndChild(agent_arg_y);
                agent_arg_z = doc.NewElement("arg");
                agent_arg_z->SetAttribute("name", "z");
                agent_arg_z->SetAttribute("value", init_pos[i][2].c_str());
                agent->InsertEndChild(agent_arg_z);
                agent_arg_R = doc.NewElement("arg");
                agent_arg_R->SetAttribute("name", "R");
                agent_arg_R->SetAttribute("value", to_string(atof(init_pos[i][3].c_str()) / 180 * PI).c_str());
                agent->InsertEndChild(agent_arg_R);
                agent_arg_P = doc.NewElement("arg");
                agent_arg_P->SetAttribute("name", "P");
                agent_arg_P->SetAttribute("value", to_string(atof(init_pos[i][4].c_str()) / 180 * PI).c_str());
                agent->InsertEndChild(agent_arg_P);
                agent_arg_Y = doc.NewElement("arg");
                agent_arg_Y->SetAttribute("name", "Y");
                agent_arg_Y->SetAttribute("value", to_string(atof(init_pos[i][5].c_str()) / 180 * PI).c_str());
                agent->InsertEndChild(agent_arg_Y);
                agent_arg_mavid = doc.NewElement("arg");
                agent_arg_mavid->SetAttribute("name", "mavlink_id");
                agent_arg_mavid->SetAttribute("value", "$(eval 1 + arg('ID'))");
                agent->InsertEndChild(agent_arg_mavid);
                agent_arg_udp_port = doc.NewElement("arg");
                agent_arg_udp_port->SetAttribute("name", "mavlink_udp_port");
                agent_arg_udp_port->SetAttribute("value", to_string(sitl_port).c_str());
                agent->InsertEndChild(agent_arg_udp_port);
                agent_arg_tcp_port = doc.NewElement("arg");
                agent_arg_tcp_port->SetAttribute("name", "mavlink_tcp_port");
                agent_arg_tcp_port->SetAttribute("value", to_string(tcp_port).c_str());
                agent->InsertEndChild(agent_arg_tcp_port);
                agent_arg_sdk_port = doc.NewElement("arg");
                agent_arg_sdk_port->SetAttribute("name", "sdk_udp_port");
                agent_arg_sdk_port->SetAttribute("value", to_string(remote_port).c_str());
                agent->InsertEndChild(agent_arg_sdk_port);
                agent_arg_cam_udp_port = doc.NewElement("arg");
                agent_arg_cam_udp_port->SetAttribute("name", "mavlink_cam_udp_port");
                agent_arg_cam_udp_port->SetAttribute("value", "14530");
                agent->InsertEndChild(agent_arg_cam_udp_port);
                agent_arg_gst_udp_port = doc.NewElement("arg");
                agent_arg_gst_udp_port->SetAttribute("name", "gst_udp_port");
                agent_arg_gst_udp_port->SetAttribute("value", "$(eval 5600 + arg('ID'))");
                agent->InsertEndChild(agent_arg_gst_udp_port);
                agent_arg_video_uri = doc.NewElement("arg");
                agent_arg_video_uri->SetAttribute("name", "video_uri");
                agent_arg_video_uri->SetAttribute("value", "$(eval 5600 + arg('ID'))");
                agent->InsertEndChild(agent_arg_video_uri);
                agent_cmd_comment = doc.NewComment(" generate sdf vehicle model ");
                agent->InsertEndChild(agent_cmd_comment);
                agent_arg_cmd = doc.NewElement("arg");
                agent_arg_cmd->SetAttribute("name", "cmd");
                agent_arg_cmd->SetAttribute("value", "$(find px4_cmd)/scripts/model_gen.py --stdout --mavlink_id=$(arg mavlink_id) --mavlink_udp_port=$(arg mavlink_udp_port) --sdk_udp_port=$(arg sdk_udp_port) --mavlink_tcp_port=$(arg mavlink_tcp_port) --gst_udp_port=$(arg gst_udp_port) --video_uri=$(arg video_uri) --mavlink_cam_udp_port=$(arg mavlink_cam_udp_port) $(find px4_cmd)/models/$(arg model)/$(arg model).sdf.jinja $(find px4_cmd)");
                agent->InsertEndChild(agent_arg_cmd);
                agent_arg_cmd_param = doc.NewElement("param");
                agent_arg_cmd_param->SetAttribute("command", "$(arg cmd)");
                agent_arg_cmd_param->SetAttribute("name", "sdf_$(arg vehicle)$(arg ID)");
                agent->InsertEndChild(agent_arg_cmd_param);
                // vehicle and sensor
                agent_arg_vehicle_param = doc.NewElement("param");
                agent_arg_vehicle_param->SetAttribute("name", "vehicle");
                agent_arg_vehicle_param->SetAttribute("value", vehicle.c_str());
                agent->InsertEndChild(agent_arg_vehicle_param);
                agent_arg_sensor_param = doc.NewElement("param");
                agent_arg_sensor_param->SetAttribute("name", "sensor");
                agent_arg_sensor_param->SetAttribute("value", sensor.c_str());
                agent->InsertEndChild(agent_arg_sensor_param);
                // parameter for init
                agent_arg_x_param = doc.NewElement("param");
                agent_arg_x_param->SetAttribute("name", "init_x");
                agent_arg_x_param->SetAttribute("value", init_pos[i][0].c_str());
                agent->InsertEndChild(agent_arg_x_param);
                agent_arg_y_param = doc.NewElement("param");
                agent_arg_y_param->SetAttribute("name", "init_y");
                agent_arg_y_param->SetAttribute("value", init_pos[i][1].c_str());
                agent->InsertEndChild(agent_arg_y_param);
                agent_arg_z_param = doc.NewElement("param");
                agent_arg_z_param->SetAttribute("name", "init_z");
                agent_arg_z_param->SetAttribute("value", init_pos[i][2].c_str());
                agent->InsertEndChild(agent_arg_z_param);
                agent_arg_R_param = doc.NewElement("param");
                agent_arg_R_param->SetAttribute("name", "init_R");
                agent_arg_R_param->SetAttribute("value", init_pos[i][3].c_str());
                agent->InsertEndChild(agent_arg_R_param);
                agent_arg_P_param = doc.NewElement("param");
                agent_arg_P_param->SetAttribute("name", "init_P");
                agent_arg_P_param->SetAttribute("value", init_pos[i][4].c_str());
                agent->InsertEndChild(agent_arg_P_param);
                agent_arg_Y_param = doc.NewElement("param");
                agent_arg_Y_param->SetAttribute("name", "init_Y");
                agent_arg_Y_param->SetAttribute("value", init_pos[i][5].c_str());
                agent->InsertEndChild(agent_arg_Y_param);
                // PX4 config
                px4_comment = doc.NewComment(" px4 ");
                agent->InsertEndChild(px4_comment);
                px4_env_1 = doc.NewElement("env");
                px4_env_1->SetAttribute("name", "PX4_SIM_MODEL");
                px4_env_1->SetAttribute("value", "$(arg vehicle)");
                agent->InsertEndChild(px4_env_1);
                px4_env_2 = doc.NewElement("env");
                px4_env_2->SetAttribute("name", "PX4_ESTIMATOR");
                px4_env_2->SetAttribute("value", "$(arg est)");
                agent->InsertEndChild(px4_env_2);
                px4_arg_1 = doc.NewElement("arg");
                px4_arg_1->SetAttribute("unless", "$(arg interactive)");
                px4_arg_1->SetAttribute("name", "px4_command_arg1");
                px4_arg_1->SetAttribute("value", "");
                agent->InsertEndChild(px4_arg_1);
                px4_arg_2 = doc.NewElement("arg");
                px4_arg_2->SetAttribute("if", "$(arg interactive)");
                px4_arg_2->SetAttribute("name", "px4_command_arg1");
                px4_arg_2->SetAttribute("value", "-d");
                agent->InsertEndChild(px4_arg_2);
                px4_node = doc.NewElement("node");
                px4_node->SetAttribute("name", "sitl_$(arg ID)");
                px4_node->SetAttribute("pkg", "px4");
                px4_node->SetAttribute("type", "px4");
                px4_node->SetAttribute("output", "screen");
                if (defalut_sim_dir->isChecked())
                {
                    px4_node->SetAttribute("args", ("$(find px4)/build/px4_sitl_default/etc -s etc/init.d-posix/rcS -i $(arg ID) -w " + topic_name + " $(arg px4_command_arg1)").c_str());
                }
                else
                {
                    px4_node->SetAttribute("args", ("$(find px4)/build/px4_sitl_default/etc -s etc/init.d-posix/rcS -i $(arg ID) -w " + sim_dir + "/" + topic_name + " $(arg px4_command_arg1)").c_str());
                }
                
                agent->InsertEndChild(px4_node);
                // spawn model
                spawn_model_comment = doc.NewComment(" spawn model ");
                agent->InsertEndChild(spawn_model_comment);
                spawn_model = doc.NewElement("node");
                spawn_model->SetAttribute("name", "$(anon vehicle_spawn)");
                spawn_model->SetAttribute("pkg", "gazebo_ros");
                spawn_model->SetAttribute("type", "spawn_model");
                spawn_model->SetAttribute("output", "screen");
                spawn_model->SetAttribute("args", ("-sdf -param sdf_$(arg vehicle)$(arg ID) -model " + topic_name + " -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)").c_str());
                agent->InsertEndChild(spawn_model);
                // mavros
                mavros_comment = doc.NewComment(" mavros ");
                agent->InsertEndChild(mavros_comment);
                mavros = doc.NewElement("include");
                mavros->SetAttribute("file", "$(find mavros)/launch/px4.launch");
                agent->InsertEndChild(mavros);
                mavros_arg_1 = doc.NewElement("arg");
                mavros_arg_1->SetAttribute("name", "fcu_url");
                mavros_arg_1->SetAttribute("value", "$(arg fcu_url)");
                mavros->InsertEndChild(mavros_arg_1);
                mavros_arg_2 = doc.NewElement("arg");
                mavros_arg_2->SetAttribute("name", "gcs_url");
                mavros_arg_2->SetAttribute("value", "");
                mavros->InsertEndChild(mavros_arg_2);
                mavros_arg_3 = doc.NewElement("arg");
                mavros_arg_3->SetAttribute("name", "tgt_system");
                mavros_arg_3->SetAttribute("value", "$(eval 1 + arg(\'ID\'))");
                mavros->InsertEndChild(mavros_arg_3);
                mavros_arg_4 = doc.NewElement("arg");
                mavros_arg_4->SetAttribute("name", "tgt_component");
                mavros_arg_4->SetAttribute("value", "1");
                mavros->InsertEndChild(mavros_arg_4);
                local_port = local_port + 1;
                remote_port = remote_port + 1;
                sitl_port = sitl_port + 1;
                tcp_port = tcp_port + 1;
            }
            // generate
            if (doc.SaveFile(output_file.c_str()) != 0)
            {
                msg_box->setText("Fail to save launch file, please retry.");
                msg_box->exec();
                return;
            }
            msg_box->setIcon(QMessageBox::Icon::Information);
            msg_box->setText("Info");
            msg_box->setWindowTitle("Info");
            msg_box->setText(("Generate Launch File to " + output_file + " Successfully!").c_str());
            msg_box->exec();
        }

        void defalut_sim_dir_slot()
        {
            if (defalut_sim_dir->isChecked())
            {
                txt_sim_dir->setText("~/.ros/[topic type]");
                sim_browse_button->setEnabled(false);
            }
            else
            {
                txt_sim_dir->setText("");
                sim_browse_button->setEnabled(true);
            }
        }

        void load_launch_slot()
        {
            // sim dir state
            bool sim_dir_state = false;
            // show tip for load launch file
            msg_box = new QMessageBox();
            msg_box->setIcon(QMessageBox::Icon::Information);
            msg_box->setText("Info");
            msg_box->setWindowTitle("Info");
            msg_box->setText("Only Support Loading Launch File Generated by the Programme and will replace The Edited One.");
            msg_box->exec();
            // get filename by file browser
            QString qfilename = QFileDialog::getOpenFileName(win, "Select  Launch File", "", "Launch Files (*.launch)");
            string filename = qfilename.toStdString();
            // err msg
            msg_box = new QMessageBox();
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            // judge if filename is empty
            if (filename == "")
            {
                msg_box->setText("Cannot find the Launch File.");
                msg_box->exec();
                return;
            }
            // init doc
            XMLDocument doc;
            if (doc.LoadFile(filename.c_str()))
            {
                msg_box->setText("Cannot load the Launch File.");
                msg_box->exec();
                return;
            }
            msg_box->setText("The Launch File is bad, please check the file.");
            // get root node
            XMLElement *root = doc.RootElement();
            // get topic type & world file & gui state
            string sim_dir_load = "";
            string gazebo_gui = "";
            string topic_type = "";
            string world_file = "";
            string name = "";
            string world_dir = "$(find mavlink_sitl_gazebo)/worlds/";
            for (XMLElement *element = root->FirstChildElement("arg"); element; element = element->NextSiblingElement("arg"))
            {
                if (!element->FindAttribute("name") || !element->FindAttribute("default"))
                {
                    msg_box->exec();
                    return;
                }
                name = element->Attribute("name");
                // world file
                if (name == "world")
                {
                    world_file = element->Attribute("default");
                    if (world_file.size() <= world_dir.size())
                    {
                        msg_box->exec();
                        return;
                    }
                    world_file.erase(0, world_dir.size());
                }
                // topic type
                if (name == "topic_type")
                {
                    topic_type = element->Attribute("default");
                }
                // gui state
                if (name == "gui")
                {
                    gazebo_gui = element->Attribute("default");
                    if (gazebo_gui != "true" && gazebo_gui != "false")
                    {
                        msg_box->exec();
                        return;
                    }
                }
                if (name == "sim_dir")
                {
                    sim_dir_load = element->Attribute("default");
                }
            }
            // check data
            if (std::find(world_files.begin(), world_files.end(), world_file) == world_files.end())
            {
                msg_box->exec();
                return;
            }
            if (std::find(topic_types.begin(), topic_types.end(), topic_type) == topic_types.end())
            {
                msg_box->exec();
                return;
            }
            // get vehicles data
            int count = 0;
            int collect = 0;
            string value = "";
            string vehicle_load = "";
            string sensor_load = "";
            string x = "";
            string y = "";
            string z = "";
            string R = "";
            string P = "";
            string Y = "";
            vector<string> vehicles_load = {};
            vector<string> sensors_load = {};
            vector<vector<string>> init_pos_load = {};
            vector<string> init_pos_load_single = {"", "", "", "", "", ""};
            for (XMLElement *element = root->FirstChildElement("group"); element; element = element->NextSiblingElement("group"))
            {
                count++;
                for (XMLElement *element_child = element->FirstChildElement("arg"); element_child; element_child = element_child->NextSiblingElement("arg"))
                {
                    if (!element_child->FindAttribute("name") || !element_child->FindAttribute("value"))
                    {
                        continue;
                    }
                    name = element_child->Attribute("name");
                    value = element_child->Attribute("value");
                    // vehicle
                    if (name == "vehicle")
                    {
                        vehicle_load = value;
                        continue;
                    }
                    // sensor
                    if (name == "sensor")
                    {
                        sensor_load = value;
                        continue;
                    }
                    // init pos
                    if (name == "x")
                    {
                        x = value;
                        continue;
                    }
                    if (name == "y")
                    {
                        y = value;
                        continue;
                    }
                    if (name == "z")
                    {
                        z = value;
                        continue;
                    }
                    if (name == "R")
                    {
                        R = value;
                        continue;
                    }
                    if (name == "P")
                    {
                        P = value;
                        continue;
                    }
                    if (name == "Y")
                    {
                        Y = value;
                        continue;
                    }
                }
                // check data
                if (std::find(vehicle_types.begin(), vehicle_types.end(), vehicle_load) == vehicle_types.end())
                {
                    continue;
                }
                if (std::find(sensor_types.begin(), sensor_types.end(), sensor_load) == sensor_types.end())
                {
                    continue;
                }
                if (!check_input_data(x, 0) ||
                    !check_input_data(y, 0) ||
                    !check_input_data(z, 0) ||
                    !check_input_data(R, 0) ||
                    !check_input_data(P, 0) ||
                    !check_input_data(Y, 0) )
                {
                    continue;
                }
                vehicles_load.push_back(vehicle_load);
                sensors_load.push_back(sensor_load);
                init_pos_load_single[0] = x;
                init_pos_load_single[1] = y;
                init_pos_load_single[2] = z;
                init_pos_load_single[3] = R;
                init_pos_load_single[4] = P;
                init_pos_load_single[5] = Y;
                init_pos_load.push_back(init_pos_load_single);
                collect++;
            }
            if (collect == 0)
            {
                msg_box->exec();
                return;
            }
            vehicles = vehicles_load;
            sensors = sensors_load;
            init_pos = init_pos_load;
            // update table
            model->clear();
            update_signal = true;
            int numbers = vehicles.size();
            QStandardItem *item_1;
            QStandardItem *item_2;
            QStandardItem *item_3;
            QStandardItem *item_4;
            QStandardItem *item_5;
            QStandardItem *item_6;
            QStandardItem *item_7;
            QStandardItem *item_8;
            for (int i = 0; i < numbers; i++)
            {
                item_1 = new QStandardItem();
                item_2 = new QStandardItem();
                item_3 = new QStandardItem();
                item_4 = new QStandardItem();
                item_5 = new QStandardItem();
                item_6 = new QStandardItem();
                item_7 = new QStandardItem();
                item_8 = new QStandardItem();
                item_1->setText(vehicles[i].c_str());
                item_2->setText(sensors[i].c_str());
                item_3->setText(init_pos[i][0].c_str());
                item_4->setText(init_pos[i][1].c_str());
                item_5->setText(init_pos[i][2].c_str());
                item_6->setText(init_pos[i][3].c_str());
                item_7->setText(init_pos[i][4].c_str());
                item_8->setText(init_pos[i][5].c_str());
                item_1->setEditable(false);
                item_2->setEditable(false);
                model->setItem(i, 0, item_1);
                model->setItem(i, 1, item_2);
                model->setItem(i, 2, item_3);
                model->setItem(i, 3, item_4);
                model->setItem(i, 4, item_5);
                model->setItem(i, 5, item_6);
                model->setItem(i, 6, item_7);
                model->setItem(i, 7, item_8);
            }
            update_signal = false;
            // update settings
            topics_list->setCurrentText(topic_type.c_str());
            worlds_list->setCurrentText(world_file.c_str());
            if (gazebo_gui == "true")
            {
                gazebo_gui_checkbox->setChecked(true);
            }
            else
            {
                gazebo_gui_checkbox->setChecked(false);
            }
            sim_dir = sim_dir_load;
            if (sim_dir != "" && access(sim_dir.c_str(), 0) == -1)
            {
                sim_dir_state = false;
                sim_dir = "";
            }
            else
            {
                sim_dir_state = true;
            }
            if (sim_dir == "")
            {
                defalut_sim_dir->setChecked(true);
                txt_sim_dir->setText("~/.ros/[topic type]");
            }
            else
            {
                defalut_sim_dir->setChecked(false);
                txt_sim_dir->setText(sim_dir.c_str());
            }
            output_file = filename;
            txt_dir->setText(output_file.c_str());
            if (!sim_dir_state)
            {
                msg_box->setIcon(QMessageBox::Icon::Warning);
                msg_box->setText("Warning");
                msg_box->setWindowTitle("Warning");
                msg_box->setText(("Can not Find Simualtion Data Directory in Launch File: " + sim_dir_load + "\nAutomatically Change to Default.").c_str());
                msg_box->exec();
            }
            msg_box->setIcon(QMessageBox::Icon::Information);
            msg_box->setText("Info");
            msg_box->setWindowTitle("Info");
            msg_box->setText(("The Launch File is Loaded Successfully.\nDetected Vehicles Count: " + to_string(count) + "\nFailed Loaded Vehicles Count: " + to_string(count - collect) + "\nSuccessfully Loaded Vehicles Count: " + to_string(collect)).c_str());
            msg_box->exec();
        }

        // utility function
        bool check_input_data(string data, int data_type = 0)
        {
            msg_box = new QMessageBox(win);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            if (!data.compare(""))
            {
                msg_box->setText("Initial Postion Can not Be Empty!");
                msg_box->exec();
                return false;
            }
            try
            {
                if (data_type == 0)
                {
                    stof(data);
                }
                if (data_type == 1)
                {
                    stoi(data);
                }
            }
            catch(const std::exception& e)
            {
                msg_box->setText("Input Initial Postion Only Support Float Type!");
                msg_box->exec();
                return false;
            }
            return true;
        }

        bool generate_sdf(string vehicle_name, string sensor_name)
        {
            // get data 
            sensor_data *sensor_load_data;
            for (auto item = sensor_datas.begin(); item < sensor_datas.end(); item++)
            {
                if ((*item)->name == sensor_name)
                {
                    sensor_load_data = *item;
                    break;
                }
            }
            transform(vehicle_name.begin(), vehicle_name.end(), vehicle_name.begin(), ::tolower);
            // get model dir
            string models_dir;
            string sensor_folder_name;
            string sensor_origin_sdf_dir;
            string sensor_sdf_dir;
            double distance;
            sensor_folder_name = sensor_name;
            for (auto item = sensor_folder_name.begin(); item < sensor_folder_name.end(); item++)
            {
                if ((*item) == ' ')
                {
                    *item = '_';
                }
            }
            transform(sensor_folder_name.begin(), sensor_folder_name.end(), sensor_folder_name.begin(), ::tolower);
            get_cmd_output("rospack find px4_cmd", models_dir);
            strip(models_dir, "\n");
            strip(models_dir);
            models_dir = models_dir + "/models/";
            sensor_origin_sdf_dir = models_dir + sensor_folder_name + "/" + sensor_folder_name + "_origin.sdf";
            sensor_sdf_dir = models_dir + sensor_folder_name + "/" + sensor_folder_name + ".sdf";
            // err msg
            msg_box = new QMessageBox();
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            msg_box->setText(("Fail to load sensor model sdf file :" + sensor_origin_sdf_dir + "\nPlease check the integrity of px4_cmd package.").c_str());
            // read and edit sensor sdf file
            // init doc
            XMLDocument doc;
            if (doc.LoadFile(sensor_origin_sdf_dir.c_str()))
            {
                msg_box->exec();
                return false;
            }
            // get root node
            XMLElement *root = doc.RootElement();
            XMLElement *model_node = root->FirstChildElement("model");
            if (!model_node)
            {
                msg_box->exec();
                return false;
            }
            // get pose node
            XMLElement *pose_node = model_node->FirstChildElement("pose");
            // get link node
            XMLElement *link_node = model_node->FirstChildElement("link");
            if (!pose_node || !link_node)
            {
                msg_box->exec();
                return false;
            }
            // set pose
            pose_node->SetText((sensor_load_data->position[0] + " " + sensor_load_data->position[1] + " " + sensor_load_data->position[2] + " " + sensor_load_data->pose[0] + " " + sensor_load_data->pose[1] + " " + sensor_load_data->pose[2] + " ").c_str());
            // get sensors node
            XMLElement *node_0;
            XMLElement *node_1;
            XMLElement *node_2;
            XMLElement *node_3;
            XMLElement *para_1;
            XMLElement *para_2;
            XMLElement *para_3;
            XMLElement *para_4;
            QString cam_pose;
            QStringList cam_poses;
            string cam_edited_pose;
            for (auto item = link_node->FirstChildElement("sensor"); item; item = item->NextSiblingElement("sensor"))
            {
                node_0 = item->FirstChildElement("update_rate");
                if (!node_0)
                {
                    msg_box->exec();
                    return false;
                }
                if (sensor_name != "Stereo Camera")
                {
                    node_0->SetText(sensor_load_data->update_rate.c_str());
                }
                if (sensor_name == "Lidar")
                {
                    node_1 = item->FirstChildElement("ray");
                    if (!node_1)
                    {
                        msg_box->exec();
                        return false;
                    }
                    node_2 = node_1->FirstChildElement("scan");
                    if (!node_2)
                    {
                        msg_box->exec();
                        return false;
                    }
                    node_3 = node_2->FirstChildElement("horizontal");
                    if (!node_3)
                    {
                        msg_box->exec();
                        return false;
                    }
                    // get para and check
                    para_1 = node_3->FirstChildElement("samples");
                    para_2 = node_3->FirstChildElement("resolution");
                    para_3 = node_3->FirstChildElement("min_angle");
                    para_4 = node_3->FirstChildElement("max_angle");
                    if (!para_1 || !para_2 || !para_3 || !para_4)
                    {
                        msg_box->exec();
                        return false;
                    }
                    para_1->SetText(sensor_load_data->samples.c_str());
                    para_2->SetText(sensor_load_data->resolution.c_str());
                    para_3->SetText(sensor_load_data->min_angle.c_str());
                    para_4->SetText(sensor_load_data->max_angle.c_str());
                }
                // camera
                else
                {
                    for (auto cam_item = item->FirstChildElement("camera"); cam_item; cam_item = cam_item->NextSiblingElement("camera"))
                    {
                        node_1 = cam_item->FirstChildElement("image");
                        if (!node_1)
                        {
                            msg_box->exec();
                            return false;
                        }
                        para_1 = node_1->FirstChildElement("width");
                        para_2 = node_1->FirstChildElement("height");
                        if (!para_1 || !para_2)
                        {
                            msg_box->exec();
                            return false;
                        }
                        para_1->SetText(sensor_load_data->width.c_str());
                        para_2->SetText(sensor_load_data->height.c_str());
                        // distance parameter for stereo camera
                        if (sensor_name == "Stereo Camera")
                        {
                            node_2 = item->FirstChildElement("update_rate");
                            if (!node_2)
                            {
                                msg_box->exec();
                                return false;
                            }
                            node_2->SetText(sensor_load_data->update_rate.c_str());
                            distance = stof(sensor_load_data->distance);
                            if (!cam_item->FindAttribute("name"))
                            {
                                msg_box->exec();
                                return false;
                            }
                            para_3 = cam_item->FirstChildElement("pose");
                            if (string(cam_item->Attribute("name")) == "left")
                            {
                                cam_pose = para_3->GetText();
                                cam_poses = cam_pose.split(" ");
                                cam_poses[1] = QString(to_string(distance / 2).c_str());
                                cam_pose = "";
                                for (auto str = cam_poses.begin(); str < cam_poses.end(); str++)
                                {
                                    cam_pose = cam_pose + " " + *str;
                                }
                                cam_edited_pose = cam_pose.toStdString();
                                cam_edited_pose.erase(0);
                                para_3->SetText(cam_edited_pose.c_str());
                            }
                            else
                            {
                                if (string(cam_item->Attribute("name")) == "right")
                                {
                                    cam_pose = para_3->GetText();
                                    cam_poses = cam_pose.split(" ");
                                    cam_poses[1] = QString(to_string(-distance / 2).c_str());
                                    cam_pose = "";
                                    for (auto str = cam_poses.begin(); str < cam_poses.end(); str++)
                                    {
                                        cam_pose = cam_pose + " " + *str;
                                    }
                                    cam_edited_pose = cam_pose.toStdString();
                                    cam_edited_pose.erase(0);
                                    para_3->SetText(cam_edited_pose.c_str());
                                }
                                else
                                {
                                    msg_box->exec();
                                    return false;
                                }
                            }
                        }
                    }
                }
            }
            if (doc.SaveFile(sensor_sdf_dir.c_str()) != 0)
            {
                msg_box->setText("Fail to save sensor sdf file, please retry.");
                msg_box->exec();
                return false;
            }
            
            // generate model sdf
            string vehicle_sdf_dir = models_dir + vehicle_name + "/" + vehicle_name + ".sdf.jinja";
            string model_name = vehicle_name + "_" + sensor_folder_name;
            string model_sdf_folder = models_dir + model_name;
            string model_sdf_dir = model_sdf_folder + "/" + model_name + ".sdf.jinja";
            string model_config_dir = model_sdf_folder + "/" + "model.config";
            mkdir(model_sdf_folder.c_str(), 0777);
            msg_box->setWindowTitle("Error");
            msg_box->setText(("Fail to load vehicle model sdf file :" + vehicle_sdf_dir + "\nPlease check the integrity of px4_cmd package.").c_str());
            if (doc.LoadFile(vehicle_sdf_dir.c_str()))
            {
                msg_box->exec();
                return false;
            }
            root = doc.RootElement();
            root->SetAttribute("version", "1.7");
            XMLElement *model = root->FirstChildElement();
            model->SetAttribute("name", model_name.c_str());
            // include sensor model sdf
            XMLComment *sensor_info = doc.NewComment(" sensor model ");
            model->InsertEndChild(sensor_info);
            XMLElement *sensor_sdf = doc.NewElement("include");
            model->InsertEndChild(sensor_sdf);
            XMLElement *sensor_uri = doc.NewElement("uri");
            sensor_uri->SetText(("file://" + models_dir + sensor_folder_name).c_str());
            sensor_sdf->InsertEndChild(sensor_uri);
            // joint
            XMLComment *joint_info = doc.NewComment(" joint ");
            model->InsertEndChild(joint_info);
            XMLElement *joint = doc.NewElement("joint");
            joint->SetAttribute("name", (model_name + "_joint").c_str());
            joint->SetAttribute("type", "fixed");
            model->InsertEndChild(joint);
            // child
            XMLElement *joint_child = doc.NewElement("child");
            joint_child->SetText((sensor_folder_name + "::link").c_str());
            joint->InsertEndChild(joint_child);
            // parent
            XMLElement *joint_parent = doc.NewElement("parent");
            joint_parent->SetText((vehicle_name + "::base_link").c_str());
            joint->InsertEndChild(joint_parent);
            // axis
            XMLElement *joint_axis = doc.NewElement("axis");
            joint->InsertEndChild(joint_axis);
            XMLElement *xyz = doc.NewElement("xyz");
            xyz->SetText("0 0 0");
            joint_axis->InsertEndChild(xyz);
            XMLElement *lim = doc.NewElement("limit");
            joint_axis->InsertEndChild(lim);
            XMLElement *upper = doc.NewElement("upper");
            XMLElement *lower = doc.NewElement("lower");
            upper->SetText("0");
            lower->SetText("0");
            lim->InsertEndChild(upper);
            lim->InsertEndChild(lower);
            XMLElement *ele_1;
            XMLElement *ele_2;
            XMLElement *ele_3;
            XMLElement *ele_4;
            XMLElement *ele_5;
            XMLElement *ele_6;
            XMLElement *ele_7;
            // for plane addition plugin is needed
            if (vehicle_name == "plane")
            {
                XMLElement *plugin = doc.NewElement("plugin");
                plugin->SetAttribute("name", "catapult_plugin");
                plugin->SetAttribute("filename", "libgazebo_catapult_plugin.so");
                ele_1 = doc.NewElement("robotNamespace");
                plugin->InsertEndChild(ele_1);
                ele_2 = doc.NewElement("link_name");
                ele_2->SetText("plane::base::link");
                plugin->InsertEndChild(ele_2);
                ele_3 = doc.NewElement("commandSubTopic");
                ele_3->SetText("/gazebo/command/motor_speed");
                plugin->InsertEndChild(ele_3);
                ele_4 = doc.NewElement("motorNumber");
                ele_4->SetText("4");
                plugin->InsertEndChild(ele_4);
                ele_5 = doc.NewElement("force");
                ele_5->SetText("50");
                plugin->InsertEndChild(ele_5);
                ele_6 = doc.NewElement("duration");
                ele_6->SetText("0.5");
                plugin->InsertEndChild(ele_6);
            }
            if (doc.SaveFile(model_sdf_dir.c_str()) != 0)
            {
                msg_box->setText("Fail to save model sdf file, please retry.");
                msg_box->exec();
                return false;
            }
            
            // generate model config file
            doc.Parse("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<!-- This launch file is generated by PX4 Cmd generator -->");
            root = doc.NewElement("model");
            doc.InsertEndChild(root);
            ele_1 = doc.NewElement("name");
            ele_1->SetText(model_name.c_str());
            root->InsertEndChild(ele_1);
            ele_2 = doc.NewElement("version");
            ele_2->SetText("1.7");
            root->InsertEndChild(ele_2);
            ele_3 = doc.NewElement("sdf");
            ele_3->SetAttribute("version", "1.7");
            ele_3->SetText((model_name + ".sdf").c_str());
            root->InsertEndChild(ele_3);
            ele_4 = doc.NewElement("author");
            root->InsertEndChild(ele_4);
            ele_5 = doc.NewElement("name");
            ele_5->SetText("Peng Yi");
            ele_4->InsertEndChild(ele_5);
            ele_6 = doc.NewElement("email");
            ele_6->SetText("yipeng3@mail2.sysu.edu.cn");
            ele_4->InsertEndChild(ele_6);
            ele_7 = doc.NewElement("description");
            ele_7->SetText(("A " + vehicle_name + "with " + sensor_folder_name + ".").c_str());
            root->InsertEndChild(ele_7);
            if (doc.SaveFile(model_config_dir.c_str()) != 0)
            {
                msg_box->setText("Fail to save model sdf file, please retry.");
                msg_box->exec();
                return false;
            }
            return true;
        }
};

#endif