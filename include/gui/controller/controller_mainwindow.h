#ifndef CONTROLLERMAINWINDOW_H
#define CONTROLLERMAINWINDOW_H
#include <QMainWindow>
#include <QApplication>
#include <QDialog>
#include <QMessageBox>
#include <QLabel>
#include <QVector>
#include <QBrush>
#include <QColor>
#include <QTableView>
#include <QPushButton>
#include <QHeaderView>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QAbstractItemModel>
#include <QPersistentModelIndex>
#include <QString>
#include <QStringList>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <mavros_msgs/State.h>

#include <gui/controller/controller_infowindow.h>
#include <gui/controller/controller_modewindow.h>
#include <gui/controller/controller_takeoffwindow.h>
#include <gui/controller/controller_manualwindow.h>
#include <gui/controller/controller_trajectorywindow.h>
#include <gui/controller/controller_generatewindow.h>

#define PI 3.14159265358979323846

struct cell_info
{
    int node_id;
    int cell_id;
    QString str;
};
struct button_info
{
    QPushButton * button_name;
    bool state;
};
Q_DECLARE_METATYPE(cell_info);
Q_DECLARE_METATYPE(button_info);

using namespace std;

class ControllerMainWindow : public QDialog
{
    Q_OBJECT
    public:
        ControllerInfoWindow *info_win = new ControllerInfoWindow(this);
        ControllerModeWindow *mode_win = new ControllerModeWindow(this);
        ControllerTakeoffWindow *takeoff_win = new ControllerTakeoffWindow(this);
        ControllerManualWindow *manual_win = new ControllerManualWindow(this);
        ControllerTrajectoryWindow *trajectory_win = new ControllerTrajectoryWindow(this);
        ControllerGenerateWindow *generate_win = new ControllerGenerateWindow(this);
        QWidget *parent;
        ControllerMainWindow(QWidget *parent_widget, QStringList nodes_input)
        {
            this->setAttribute(Qt::WA_DeleteOnClose);
            qRegisterMetaType<QList<QPersistentModelIndex>>("QList<QPersistentModelIndex>");
            qRegisterMetaType<QList<QAbstractItemModel::LayoutChangeHint>>("QList<QAbstractItemModel::LayoutChangeHint>");
            qRegisterMetaType<QList<Qt::Orientation>>("QList<Qt::Orientation>");
            qRegisterMetaType<cell_info>("cell_info");
            qRegisterMetaType<cell_info>("cell_info&");
            qRegisterMetaType<button_info>("button_info");
            qRegisterMetaType<button_info>("button_info&");
            nodes = nodes_input;
            setup();
        }
        ~ControllerMainWindow()
        {
            thread_stop = true;
            usleep(200000);
        }

    signals:
        void update_cell_info_signal(QVariant info);
        void update_info_signal();
        void update_button_state_signal(QVariant state);

    private:
        // settings
        string version = "V1.0.3";
        QString current_cmd = "None";
        double update_time = 0.3;

        // init vectors
        QVector<vehicle_command *> data;
        QVector<px4_cmd::Command> cmds;
        QVector<ros::Publisher> pubs;
        vector<vector<double>> cmd_values;
        vector<vector<QStandardItem *>> table_items;
        bool thread_stop = false;
        bool land_return_operate = false;
        bool ext_cmd_state = false;
        QStringList nodes;
        QString operating_info;
        bool fix_wing_include = false;
        mutex cmd_mutex;

        //Widgets
        QMessageBox *msg_box;
        QPushButton *mode_button;
        QPushButton *signal_button_1;
        QPushButton *signal_button_2;
        QPushButton *arm_button;
        QPushButton *takeoff_button;
        QPushButton *disarm_button;
        QPushButton *about_button;
        QPushButton *exit_button;
        QPushButton *manual_button;
        QPushButton *trajectory_button;
        QPushButton *external_button;
        QPushButton *hover_button;
        QPushButton *land_button;
        QPushButton *return_button;
        QPushButton *generate_button;
        QTableView *info_table = new QTableView(this);
        QStandardItemModel *info_model;
        QLabel *info_label_1;
        QLabel *info_label_2;
        QLabel *info_label_3;
        QLabel *state_label_1;
        QLabel *state_label_2;
        QLabel *state_label_3;

        void setup()
        {
            // set window
            QIcon *icon = new QIcon(QApplication::style()->standardIcon(QStyle::SP_FileIcon));
            this->setWindowIcon(*icon);
            this->setFixedSize(1280, 750);
            this->setWindowTitle(("PX4 Cmd Simulation Controller [Version: " + version + "]").c_str());
            this->setStyleSheet("background-color: rgb(255,250,250)");

            // get data
            for (auto item = nodes.begin(); item != nodes.end(); item++)
            {
                vehicle_command *vec = new vehicle_command();
                vec->start((*item).toStdString());
                data.push_back(vec);
            }

            // ros setting
            int argc = 0;
            char **argv;
            ros::init(argc, argv, "px4_cmd/px4_controller");
            ros::NodeHandle nh;
            for (size_t i = 0; i < nodes.size(); i++)
            {
                px4_cmd::Command cmd;
                if (data[i]->vehicle_name == "plane")
                {
                    cmd.Vehicle = px4_cmd::Command::FixWing;
                    fix_wing_include = true;
                }
                else
                {
                    cmd.Vehicle = px4_cmd::Command::Multicopter;
                }
                cmd.desire_cmd[0] = data[i]->init_x;
                cmd.desire_cmd[1] = data[i]->init_y;
                cmd.desire_cmd[2] = 0;
                ros::Publisher pub = nh.advertise<px4_cmd::Command>((nodes[i] + "/px4_cmd/control_command").toStdString().c_str(), 50);
                pubs.push_back(pub);
                cmds.push_back(cmd);
                cmd_values.push_back({0, 0, 0, 0});
            }

            // start ros thread
            for (size_t i = 0; i < nodes.size(); i++)
            {
                std::thread ros_thread(&ControllerMainWindow::ros_thread_func, this, i);
                ros_thread.detach();
            }

            // layout
            QVBoxLayout *vbox = new QVBoxLayout();
            QHBoxLayout *hbox_top = new QHBoxLayout();
            QVBoxLayout *vbox_top_1 = new QVBoxLayout();
            QVBoxLayout *vbox_top_2 = new QVBoxLayout();
            QVBoxLayout *vbox_top_3 = new QVBoxLayout();
            QVBoxLayout *vbox_top_4 = new QVBoxLayout();
            QVBoxLayout *vbox_top_5 = new QVBoxLayout();
            QHBoxLayout *hbox_buttom = new QHBoxLayout();
            QVBoxLayout *vbox_buttom_1 = new QVBoxLayout();
            QVBoxLayout *vbox_buttom_2 = new QVBoxLayout();
            QHBoxLayout *hbox_buttom_1 = new QHBoxLayout();

            // add labels
            info_label_1 = new QLabel("[ROS State]  Running", this);
            info_label_1->setStyleSheet("color: green; font-size: 14pt");
            info_label_2 = new QLabel(("[Vehicle Count]  " + to_string(nodes.size())).c_str(), this);
            info_label_2->setStyleSheet("color: green; font-size: 14pt");
            info_label_3 = new QLabel(("[Current Mode]  " + data[0]->state_mode).c_str(), this);
            info_label_3->setStyleSheet("color: green; font-size: 14pt");
            state_label_1 = new QLabel("[Arm State]  DisArm", this);
            state_label_1->setStyleSheet("color: orange; font-size: 14pt");
            state_label_2 = new QLabel("[Current CMD]  None", this);
            state_label_2->setStyleSheet("color: green; font-size: 14pt");
            state_label_3 = new QLabel("", this);
            state_label_3->setStyleSheet("color: orange; font-size: 14pt");
            QLabel *label_1 = new QLabel("[Move CMD Mode]", this);
            label_1->setStyleSheet("color: rgb(0,169,190); font-size: 16pt; font-weight: bold");
            label_1->setAlignment(Qt::AlignmentFlag::AlignCenter);
            QLabel *label_2 = new QLabel("[State CMD Mode]", this);
            label_2->setStyleSheet("color: rgb(209,167,0); font-size: 16pt; font-weight: bold");
            label_2->setAlignment(Qt::AlignmentFlag::AlignCenter);
            info_label_3->setMinimumWidth(300);
            state_label_2->setMinimumWidth(300);

            // add top buttons
            QPushButton *split_line = new QPushButton("", this);
            split_line->setMaximumHeight(1);
            split_line->setFocusPolicy(Qt::NoFocus);
            split_line->setEnabled(false);
            mode_button = new QPushButton("Mode", this);
            arm_button = new QPushButton("Arm", this);
            takeoff_button = new QPushButton("Take Off", this);
            disarm_button = new QPushButton("DisArm", this);
            about_button = new QPushButton("About", this);
            exit_button = new QPushButton("Exit", this);
            disarm_button->setEnabled(false);
            mode_button->setMinimumHeight(45);
            arm_button->setMinimumHeight(45);
            takeoff_button->setMinimumHeight(45);
            disarm_button->setMinimumHeight(45);
            about_button->setMinimumHeight(45);
            exit_button->setMinimumHeight(45);
            mode_button->setStyleSheet("background-color: rgb(50,191,255); font-size: 16pt");
            arm_button->setStyleSheet("background-color: rgb(84,255,159); font-size: 16pt");
            takeoff_button->setStyleSheet("background-color: rgb(170,123,255); font-size: 16pt");
            disarm_button->setStyleSheet("background-color: rgb(255,190,155); font-size: 16pt");
            about_button->setStyleSheet("background-color: rgb(255,227,132); font-size: 16pt");
            exit_button->setStyleSheet("background-color: rgb(255,106,106); font-size: 16pt");

            // add buttom button
            manual_button = new QPushButton("Manual CMD", this);
            trajectory_button = new QPushButton("Trajectory CMD", this);
            external_button = new QPushButton("External CMD", this);
            hover_button = new QPushButton("Hover / Loiter", this);
            land_button = new QPushButton("Land", this);
            return_button = new QPushButton("Return", this);
            generate_button = new QPushButton("Generate Template External CMD Code");
            generate_button->setVisible(false);
            signal_button_1 = new QPushButton("", this);
            signal_button_2 = new QPushButton("", this);
            signal_button_1->setVisible(false);
            signal_button_2->setVisible(false);
            manual_button->setMinimumHeight(60);
            trajectory_button->setMinimumHeight(60);
            external_button->setMinimumHeight(60);
            hover_button->setMinimumHeight(60);
            land_button->setMinimumHeight(60);
            return_button->setMinimumHeight(60);
            generate_button->setMinimumHeight(45);
            manual_button->setEnabled(false);
            trajectory_button->setEnabled(false);
            external_button->setEnabled(false);
            hover_button->setEnabled(false);
            land_button->setEnabled(false);
            return_button->setEnabled(false);
            manual_button->setStyleSheet("background-color: rgb(182,228,222); font-size: 16pt");
            trajectory_button->setStyleSheet("background-color: rgb(182,228,222); font-size: 16pt");
            external_button->setStyleSheet("background-color: rgb(182,228,222); font-size: 16pt");
            hover_button->setStyleSheet("background-color: rgb(255,245,129); font-size: 16pt");
            land_button->setStyleSheet("background-color: rgb(255,245,129); font-size: 16pt");
            return_button->setStyleSheet("background-color: rgb(255,245,129); font-size: 16pt");
            generate_button->setStyleSheet("background-color: rgb(233,181,177); font-size: 16pt");

            // add table
            QStringList table_headers_ext_cmd = {
                "Node",
                "CMD Mode",
                "Frame",
                "CMD 1",
                "CMD 2",
                "CMD 3",
                "CMD 4  [Yaw]",
                "External State",
                "External CMD Topic"
            };
            info_model = new QStandardItemModel(nodes.size(), table_headers_ext_cmd.size(), this);
            info_model->setHorizontalHeaderLabels(table_headers_ext_cmd);
            vector<QStandardItem *> table_item;
            for (int i = 0; i < nodes.size(); i++)
            {
                table_item.clear();
                QStandardItem *item_1 = new QStandardItem();
                QStandardItem *item_2 = new QStandardItem();
                QStandardItem *item_3 = new QStandardItem();
                QStandardItem *item_4 = new QStandardItem();
                QStandardItem *item_5 = new QStandardItem();
                QStandardItem *item_6 = new QStandardItem();
                QStandardItem *item_7 = new QStandardItem();
                QStandardItem *item_8 = new QStandardItem();
                QStandardItem *item_9 = new QStandardItem();
                item_1->setText("  " + nodes[i] + "  ");
                item_9->setText((nodes[i] + "/px4_cmd/external_command").toStdString().c_str());
                item_1->setEditable(false);
                item_2->setEditable(false);
                item_3->setEditable(false);
                item_4->setEditable(false);
                item_5->setEditable(false);
                item_6->setEditable(false);
                item_7->setEditable(false);
                item_8->setEditable(false);
                item_9->setEditable(false);
                item_1->setTextAlignment(Qt::AlignCenter);
                item_2->setTextAlignment(Qt::AlignCenter);
                item_3->setTextAlignment(Qt::AlignCenter);
                item_4->setTextAlignment(Qt::AlignCenter);
                item_5->setTextAlignment(Qt::AlignCenter);
                item_6->setTextAlignment(Qt::AlignCenter);
                item_7->setTextAlignment(Qt::AlignCenter);
                item_8->setTextAlignment(Qt::AlignCenter);
                item_9->setTextAlignment(Qt::AlignCenter);
                info_model->setItem(i, 0, item_1);
                info_model->setItem(i, 1, item_2);
                info_model->setItem(i, 2, item_3);
                info_model->setItem(i, 3, item_4);
                info_model->setItem(i, 4, item_5);
                info_model->setItem(i, 5, item_6);
                info_model->setItem(i, 6, item_7);
                info_model->setItem(i, 7, item_8);
                info_model->setItem(i, 8, item_9);
                table_item.push_back(item_1);
                table_item.push_back(item_2);
                table_item.push_back(item_3);
                table_item.push_back(item_4);
                table_item.push_back(item_5);
                table_item.push_back(item_6);
                table_item.push_back(item_7);
                table_item.push_back(item_8);
                table_item.push_back(item_9);
                table_items.push_back(table_item);
            }
            info_table->setModel(info_model);
            info_table->horizontalHeader()->setStretchLastSection(true);
            info_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
            info_table->setStyleSheet("background-color: rgb(236,245,225)");

            // set layout
            vbox->setContentsMargins(30, 20, 30, 20);
            vbox_top_1->setSpacing(15);
            vbox_top_2->setSpacing(15);
            vbox_top_3->setSpacing(15);
            vbox_top_4->setSpacing(15);
            vbox_top_5->setSpacing(15);
            vbox_buttom_1->setAlignment(Qt::AlignmentFlag::AlignTop);
            vbox_buttom_1->setSpacing(20);
            vbox_buttom_2->setSpacing(10);
            hbox_buttom->setSpacing(40);
            vbox_top_1->addWidget(info_label_1);
            vbox_top_1->addWidget(info_label_2);
            vbox_top_1->addWidget(info_label_3);
            hbox_top->addLayout(vbox_top_1);
            vbox_top_2->addWidget(state_label_1);
            vbox_top_2->addWidget(state_label_2);
            vbox_top_2->addWidget(state_label_3);
            hbox_top->addLayout(vbox_top_2);
            vbox_top_3->addWidget(mode_button);
            vbox_top_3->addWidget(arm_button);
            hbox_top->addLayout(vbox_top_3);
            vbox_top_4->addWidget(takeoff_button);
            vbox_top_4->addWidget(disarm_button);
            hbox_top->addLayout(vbox_top_4);
            vbox_top_5->addWidget(about_button);
            vbox_top_5->addWidget(exit_button);
            hbox_top->addLayout(vbox_top_5);
            hbox_top->setSpacing(30);
            vbox->addLayout(hbox_top, 10);
            vbox->addSpacing(15);
            vbox->addWidget(split_line, 1);
            vbox->addSpacing(15);
            vbox_buttom_1->addWidget(label_1);
            vbox_buttom_1->addWidget(manual_button);
            vbox_buttom_1->addWidget(trajectory_button);
            vbox_buttom_1->addWidget(external_button);
            vbox_buttom_1->addSpacing(15);
            vbox_buttom_1->addWidget(label_2);
            vbox_buttom_1->addWidget(hover_button);
            vbox_buttom_1->addWidget(land_button);
            vbox_buttom_1->addWidget(return_button);
            hbox_buttom->addLayout(vbox_buttom_1, 1);
            vbox_buttom_2->addWidget(info_table);
            //vbox_buttom_2->addWidget(generate_button);
            hbox_buttom->addLayout(vbox_buttom_2, 5);
            vbox->addLayout(hbox_buttom);
            this->setLayout(vbox);

            // connect
            QObject::connect(about_button, &QPushButton::clicked, this, &ControllerMainWindow::info_window_slot);
            QObject::connect(mode_button, &QPushButton::clicked, this, &ControllerMainWindow::mode_window_slot);
            QObject::connect(takeoff_button, &QPushButton::clicked, this, &ControllerMainWindow::take_off_window_slot);
            QObject::connect(arm_button, &QPushButton::clicked, this, &ControllerMainWindow::arm_slot);
            QObject::connect(disarm_button, &QPushButton::clicked, this, &ControllerMainWindow::disarm_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &ControllerMainWindow::exit_slot);
            QObject::connect(hover_button, &QPushButton::clicked, this, &ControllerMainWindow::hover_slot);
            QObject::connect(land_button, &QPushButton::clicked, this, &ControllerMainWindow::land_slot);
            QObject::connect(return_button, &QPushButton::clicked, this, &ControllerMainWindow::return_slot);
            QObject::connect(manual_button, &QPushButton::clicked, this, &ControllerMainWindow::manual_cmd_slot);
            QObject::connect(trajectory_button, &QPushButton::clicked, this, &ControllerMainWindow::trajectory_cmd_slot);
            QObject::connect(external_button, &QPushButton::clicked, this, &ControllerMainWindow::ext_cmd_slot);
            QObject::connect(signal_button_1, &QPushButton::clicked, this, &ControllerMainWindow::ext_cmd_err_msg_slot);
            QObject::connect(signal_button_2, &QPushButton::clicked, this, &ControllerMainWindow::arm_fail_slot);
            QObject::connect(mode_win, &ControllerModeWindow::change_mode_signal, this, &ControllerMainWindow::change_mode_slot);
            QObject::connect(takeoff_win, &ControllerTakeoffWindow::take_off_info_signal, this, &ControllerMainWindow::take_off_info_slot);
            QObject::connect(generate_button, &QPushButton::clicked, this, &ControllerMainWindow::ext_cmd_generate_slot);
            QObject::connect(this, &ControllerMainWindow::update_cell_info_signal, this, &ControllerMainWindow::update_cell_info_slot);
            QObject::connect(this, &ControllerMainWindow::update_info_signal, this, &ControllerMainWindow::update_info_slot);

            // thread
            for (size_t i = 0; i < nodes.size(); i++)
            {
                std::thread update_table_thread(&ControllerMainWindow::update_table_info, this, i);
                update_table_thread.detach();
            }
            std::thread update_thread(&ControllerMainWindow::update_info, this);
            update_thread.detach();
        }

        // ros thread
        void ros_thread_func(int node_id)
        {
            while (ros::ok() && !thread_stop)
            {
                pubs[node_id].publish(cmds[node_id]);
                ros::spinOnce();
                usleep(20000);
            }
        }

        // slot functions
        void info_window_slot()
        {
            info_win->exec();
        }

        void mode_window_slot()
        {
            mode_win->set_data(data);
            mode_win->exec();
        }

        void arm_slot()
        {
            operating_info = "Arming...";
            std::thread arm_thread(&ControllerMainWindow::arm_thread_func, this);
            arm_thread.detach();
        }

        void arm_thread_func()
        {
            QVariant button_data;
            button_info button;
            string err = "";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                err = data[i]->set_mode("Arm");
                if (err != "")
                {
                    msg_box = new QMessageBox(this);
                    msg_box->setIcon(QMessageBox::Icon::Critical);
                    msg_box->setWindowTitle("Error");
                    msg_box->setText("Armed Failure, Please Retry.");
                    msg_box->exec();
                    for (size_t j = 0; j < nodes.size(); j++)
                    {
                        data[i]->set_mode(mavros_msgs::State::MODE_PX4_RTL);
                    }
                    operating_info = "";
                    return;
                }
            }
            operating_info = "";
            current_cmd = "Arm";
            button.button_name = arm_button;
            button.state = false;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = disarm_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
        }

        void disarm_slot()
        {
            operating_info = "DisArming...";
            std::thread disarm_thread(&ControllerMainWindow::disarm_thread_func, this);
            disarm_thread.detach();
        }

        void disarm_thread_func()
        {
            QVariant button_data;
            button_info button;
            string err = "";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                err = data[i]->set_mode("DisArm");
                if (err != "")
                {
                    msg_box = new QMessageBox(this);
                    msg_box->setIcon(QMessageBox::Icon::Critical);
                    msg_box->setWindowTitle("Error");
                    msg_box->setText(err.c_str());
                    msg_box->exec();
                    operating_info = "";
                    return;
                }
            }
            operating_info = "";
            current_cmd = "DisArm";
            button.button_name = arm_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = disarm_button;
            button.state = false;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
        }

        void take_off_window_slot()
        {
            if (!takeoff_win->set_data(data))
            {
                return;
            }
            takeoff_win->exec();
            if (takeoff_win->set_height)
            {
                takeoff_button->setEnabled(false);
                arm_button->setEnabled(false);
                mode_button->setEnabled(false);
                std::thread take_off_thread(&ControllerMainWindow::take_off_thread_func, this);
                take_off_thread.detach();
            }
        }

        void take_off_thread_func()
        {
            QVariant button_data;
            button_info button;
            string err = "";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                cmds[i].Mode = px4_cmd::Command::Takeoff;
                cmds[i].Move_frame = px4_cmd::Command::ENU;
                cmd_values[i][0] = data[i]->home_position[0] + data[i]->init_x;
                cmd_values[i][1] = data[i]->home_position[1] + data[i]->init_y;
                cmd_values[i][2] = takeoff_win->takeoff_height;
                cmd_values[i][3] = 0;
                cmds[i].desire_cmd[0] = data[i]->home_position[0];
                cmds[i].desire_cmd[1] = data[i]->home_position[1];
                cmds[i].desire_cmd[2] = cmd_values[i][2];
                cmds[i].yaw_cmd = cmd_values[i][3];
                err = data[i]->set_mode("Arm");
                if (err != "")
                {
                    msg_box = new QMessageBox(this);
                    msg_box->setIcon(QMessageBox::Icon::Critical);
                    msg_box->setWindowTitle("Error");
                    msg_box->setText(("Armed Failure:" + err + "\nPlease Retry...").c_str());
                    msg_box->exec();
                    for (size_t j = 0; j < nodes.size(); j++)
                    {
                        data[i]->set_mode(mavros_msgs::State::MODE_PX4_RTL);
                    }
                    operating_info = "";
                    signal_button_2->click();
                    return;
                }
            }
            operating_info = "";
            current_cmd = "Take Off";
            button.button_name = mode_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = takeoff_button;
            button.state = false;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = arm_button;
            button.state = false;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = disarm_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = manual_button;
            button.state = true;
            manual_button->setEnabled(true);
            button.button_name = trajectory_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = external_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = hover_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = land_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
            button.button_name = return_button;
            button.state = true;
            button_data.setValue(button);
            emit update_button_state_signal(button_data);
        }

        void trajectory_cmd_slot()
        {
            trajectory_win->set_nodes(nodes);
            trajectory_win->exec();
            if (trajectory_win->exec_state)
            {
                current_cmd = "Trajectory CMD";
                std::thread trajectory_cmd_thread(&ControllerMainWindow::trajectory_cmd_thread_func, this);
                trajectory_cmd_thread.detach();
                usleep(500000);
            }
        }

        void trajectory_cmd_thread_func()
        {
            double sleep_time = trajectory_win->set_time;
            for (size_t i = 0; i < trajectory_win->cmd_values.size(); i++)
            {
                cmd_mutex.lock();
                cmd_values = trajectory_win->cmd_values[i];
                cmd_mutex.unlock();
                // set cmd
                for (size_t j = 0; j < cmds.size(); j++)
                {
                    cmds[j].Mode = px4_cmd::Command::Move;
                    cmds[j].Move_mode = trajectory_win->set_mode;
                    cmds[j].Move_frame = trajectory_win->set_frame;
                    if (cmds[j].Move_mode == px4_cmd::Command::XYZ_REL_POS)
                    {
                        cmds[j].desire_cmd[0] = cmds[j].desire_cmd[0] + cmd_values[j][0] + data[j]->home_position[0];
                        cmds[j].desire_cmd[1] = cmds[j].desire_cmd[1] + cmd_values[j][1] + data[j]->home_position[1];
                        cmds[j].desire_cmd[2] = cmds[j].desire_cmd[2] + cmd_values[j][2];
                    }
                    else
                    {
                        cmds[j].desire_cmd[0] = cmd_values[j][0];
                        cmds[j].desire_cmd[1] = cmd_values[j][1];
                        cmds[j].desire_cmd[2] = cmd_values[j][2];
                        if (cmds[j].Move_mode == px4_cmd::Command::XYZ_POS)
                        {
                            cmds[j].desire_cmd[0] = cmds[j].desire_cmd[0] - data[j]->init_x;
                            cmds[j].desire_cmd[1] = cmds[j].desire_cmd[1] - data[j]->init_y;
                            cmds[j].desire_cmd[2] = cmds[j].desire_cmd[2] - data[j]->init_z;
                        }
                        else
                        {
                            if (cmds[j].Move_mode == px4_cmd::Command::XY_VEL_Z_POS)
                            {
                                cmds[j].desire_cmd[2] = cmds[j].desire_cmd[2] - data[j]->init_z;
                            }
                        }
                    }
                    cmds[j].yaw_cmd = cmd_values[j][3];
                }
                operating_info = ("Flying to Trajectory Point [" + to_string(i) + "] ...").c_str();
                // judge if achieve desire cmd
                sleep(1);
                while (!judge_all_achieve_state(true))
                {
                    usleep(200000);
                }
                usleep(floor(sleep_time * 1000000));
            }
            operating_info = "Trajectory CMD Done. Change to Hover Mode.";
            sleep(3);
            hover_button->click();
        }

        void ext_cmd_slot()
        {
            if (!ext_cmd_state)
            {
                msg_box = new QMessageBox(this);
                msg_box->setIcon(QMessageBox::Icon::Critical);
                msg_box->setWindowTitle("Error");
                msg_box->setText("Not Detect External Command Topic, Please Check.");
                msg_box->exec();
                return;
            }
            current_cmd = "External Command";
            for (size_t i = 0; i < nodes.size(); i++)
            {
                std::thread ext_cmd_thread(&ControllerMainWindow::ext_cmd_thread_func, this, i);
                ext_cmd_thread.detach();
            }
            usleep(500000);
            manual_button->setEnabled(false);
            trajectory_button->setEnabled(false);
            external_button->setEnabled(false);
        }

        void ext_cmd_thread_func(int node_id)
        {
            while (ext_cmd_state && (current_cmd == "External Command"))
            {
                cmd_mutex.lock();
                cmd_values[node_id][0] = data[node_id]->ext_cmd.desire_cmd[0];
                cmd_values[node_id][1] = data[node_id]->ext_cmd.desire_cmd[1];
                cmd_values[node_id][2] = data[node_id]->ext_cmd.desire_cmd[2];
                cmd_values[node_id][3] = data[node_id]->ext_cmd.yaw_cmd;
                cmd_mutex.unlock();
                cmds[node_id].Mode = data[node_id]->ext_cmd.Mode;
                cmds[node_id].Move_mode = data[node_id]->ext_cmd.Move_mode;
                cmds[node_id].Move_frame = data[node_id]->ext_cmd.Move_frame;
                cmds[node_id].Vehicle = data[node_id]->ext_cmd.Vehicle;
                if (cmds[node_id].Move_mode == px4_cmd::Command::XYZ_REL_POS)
                {
                    cmds[node_id].desire_cmd[0] = cmds[node_id].desire_cmd[0] + cmd_values[node_id][0] + data[node_id]->home_position[0];
                    cmds[node_id].desire_cmd[1] = cmds[node_id].desire_cmd[1] + cmd_values[node_id][1] + data[node_id]->home_position[1];
                    cmds[node_id].desire_cmd[2] = cmds[node_id].desire_cmd[2] + cmd_values[node_id][2];
                }
                else
                {
                    cmds[node_id].desire_cmd[0] = cmd_values[node_id][0];
                    cmds[node_id].desire_cmd[1] = cmd_values[node_id][1];
                    cmds[node_id].desire_cmd[2] = cmd_values[node_id][2];
                    if (cmds[node_id].Move_mode == px4_cmd::Command::XYZ_POS)
                    {
                        cmds[node_id].desire_cmd[0] = cmds[node_id].desire_cmd[0] - data[node_id]->init_x;
                        cmds[node_id].desire_cmd[1] = cmds[node_id].desire_cmd[1] - data[node_id]->init_y;
                        cmds[node_id].desire_cmd[2] = cmds[node_id].desire_cmd[2] - data[node_id]->init_z;
                    }
                    else
                    {
                        if (cmds[node_id].Move_mode == px4_cmd::Command::XY_VEL_Z_POS)
                        {
                            cmds[node_id].desire_cmd[2] = cmds[node_id].desire_cmd[2] - data[node_id]->init_z;
                        }
                    }
                }
                cmds[node_id].yaw_cmd = cmd_values[node_id][3];
                // pubs[node_id].publish(cmds[node_id]);
                usleep(20000);
            }
            if (!ext_cmd_state)
            {
                if (data[node_id]->vehicle_name == "plane")
                {
                    cmds[node_id].Mode = px4_cmd::Command::Loiter;
                }
                else
                {
                    cmds[node_id].Mode = px4_cmd::Command::Hover;
                }
            }
            manual_button->setEnabled(true);
            external_button->setEnabled(true);
            trajectory_button->setEnabled(true);
        }

        void hover_slot()
        {
            operating_info = "";
            current_cmd = "Hover";
            std::thread hover_thread(&ControllerMainWindow::hover_thread_func, this);
            hover_thread.detach();
        }

        void hover_thread_func()
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->vehicle_name == "plane")
                {
                    cmds[i].Mode = px4_cmd::Command::Loiter;
                    continue;
                }
                cmds[i].Mode = px4_cmd::Command::Hover;
            }
        }

        void land_slot()
        {
            current_cmd = "Land";
            mode_button->setEnabled(false);
            takeoff_button->setEnabled(false);
            arm_button->setEnabled(false);
            manual_button->setEnabled(false);
            trajectory_button->setEnabled(false);
            external_button->setEnabled(false);
            hover_button->setEnabled(false);
            land_button->setEnabled(false);
            return_button->setEnabled(false);
            std::thread land_thread(&ControllerMainWindow::land_thread_func, this);
            land_thread.detach();
        }

        void land_thread_func()
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                data[i]->set_mode(mavros_msgs::State::MODE_PX4_LAND);
            }
        }

        void return_slot()
        {
            current_cmd = "Return";
            mode_button->setEnabled(false);
            takeoff_button->setEnabled(false);
            arm_button->setEnabled(false);
            manual_button->setEnabled(false);
            trajectory_button->setEnabled(false);
            external_button->setEnabled(false);
            hover_button->setEnabled(false);
            land_button->setEnabled(false);
            // fix wing need to land manually
            if (fix_wing_include)
            {
                land_button->setEnabled(true);
            }
            return_button->setEnabled(false);
            std::thread return_thread(&ControllerMainWindow::return_thread_func, this);
            return_thread.detach();
        }

        void return_thread_func()
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                data[i]->set_mode(mavros_msgs::State::MODE_PX4_RTL);
            }
        }

        void change_mode_slot()
        {
            current_cmd = "Change Mode";
        }

        void exit_slot()
        {
            bool stop = false;
            int tmp = 0;
            thread_stop = true;
            for (auto item = data.begin(); item != data.end(); item++)
            {
                (*item)->thread_stop = true;
            }
            usleep(20000);
            this->close();
        }

        void take_off_info_slot()
        {
            operating_info = "Take off and Arming...";
        }

        void manual_cmd_slot()
        {
            manual_win->set_nodes(nodes);
            manual_win->exec();
            if (manual_win->exec_state)
            {
                current_cmd = "Manual CMD";
                operating_info = "Flying to Set Point...";
                std::thread manual_cmd_thread(&ControllerMainWindow::manual_cmd_thread_func, this);
                manual_cmd_thread.detach();
            }
        }

        void manual_cmd_thread_func()
        {
            bool achieve = false;
            cmd_mutex.lock();
            cmd_values = manual_win->cmd_values[0];
            cmd_mutex.unlock();
            for (size_t i = 0; i < cmds.size(); i++)
            {
                cmds[i].Mode = px4_cmd::Command::Move;
                cmds[i].Move_mode = manual_win->set_mode;
                cmds[i].Move_frame = manual_win->set_frame;
                if (cmds[i].Move_mode == px4_cmd::Command::XYZ_REL_POS)
                {
                    cmds[i].desire_cmd[0] = cmds[i].desire_cmd[0] + cmd_values[i][0];
                    cmds[i].desire_cmd[1] = cmds[i].desire_cmd[1] + cmd_values[i][1];
                    cmds[i].desire_cmd[2] = cmds[i].desire_cmd[2] + cmd_values[i][2];
                }
                else
                {
                    cmds[i].desire_cmd[0] = cmd_values[i][0];
                    cmds[i].desire_cmd[1] = cmd_values[i][1];
                    cmds[i].desire_cmd[2] = cmd_values[i][2];
                    if (cmds[i].Move_mode == px4_cmd::Command::XYZ_POS)
                    {
                        cmds[i].desire_cmd[0] = cmds[i].desire_cmd[0] - data[i]->init_x;
                        cmds[i].desire_cmd[1] = cmds[i].desire_cmd[1] - data[i]->init_y;
                        cmds[i].desire_cmd[2] = cmds[i].desire_cmd[2] - data[i]->init_z;
                    }
                    else
                    {
                        if (cmds[i].Move_mode == px4_cmd::Command::XY_VEL_Z_POS)
                        {
                            cmds[i].desire_cmd[2] = cmds[i].desire_cmd[2] - data[i]->init_z;
                        }
                    }
                }
                cmds[i].yaw_cmd = cmd_values[i][3];
            }
            // judge if achieve desire cmd
            sleep(1);
            while (!judge_all_achieve_state(true))
            {
                usleep(200000);
            }
            operating_info = "Manual CMD Done. Change to Hover Mode";
            sleep(3);
            hover_button->click();
        }

        void ext_cmd_err_msg_slot()
        {
            msg_box = new QMessageBox(this);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            msg_box->setText("Disconnection to External Command Topic, Please Check. Change to Hover Mode.");
            msg_box->exec();
        }

        void ext_cmd_generate_slot()
        {
            generate_win->exec();
        }

        void arm_fail_slot()
        {
            mode_button->setEnabled(true);
            takeoff_button->setEnabled(true);
            arm_button->setEnabled(true);
        }

        void update_info()
        {
            while (!thread_stop)
            {
                emit update_info_signal();
                usleep(200000);
            }
        }

        void update_info_slot()
        {
            bool pre_arm_state = false;
            bool arm_state = false;
            bool ext_cmd_state_single = false;
            QString pre_cmd = "None";
            string current_mode = "";
            string pre_mode = "";
            QStringList table_headers_ext_cmd = {
                "Node",
                "CMD Mode",
                "Frame",
                "CMD 1",
                "CMD 2",
                "CMD 3",
                "CMD 4  [Yaw]",
                "External State",
                "External CMD Topic"};
        
            // ros state
            if (ros::master::check())
            {
                info_label_1->setText("[ROS State]  Running");
                info_label_1->setStyleSheet("color: green; font-size: 14pt");
                info_label_2->setText(("[Vehicle Count]  " + to_string(nodes.size())).c_str());
                info_label_2->setStyleSheet("color: green; font-size: 14pt");
            }
            else
            {
                info_label_1->setText("[ROS State]  Not Running");
                info_label_1->setStyleSheet("color: red; font-size: 14pt");
                info_label_2->setText("[Vehicle Count]  0");
                info_label_2->setStyleSheet("color: green; font-size: 14pt");
            }

            // arm state
            current_mode = data[0]->state_mode;
            if (current_mode == mavros_msgs::State::MODE_PX4_RTL || current_mode == mavros_msgs::State::MODE_PX4_LAND)
            {
                arm_state = judge_all_arm_state(false);
            }
            else
            {
                arm_state = judge_all_arm_state(true);
            }
            if (arm_state != pre_arm_state || current_mode != pre_mode)
            {
                if (arm_state)
                {
                    state_label_1->setText("[Arm State]  Arm");
                    state_label_1->setStyleSheet("color: green; font-size: 14pt");
                    arm_button->setEnabled(false);
                    disarm_button->setEnabled(true);
                    if (judge_all_land_state(false))
                    {
                        mode_button->setEnabled(true);
                        manual_button->setEnabled(true);
                        trajectory_button->setEnabled(true);
                        external_button->setEnabled(true);
                        hover_button->setEnabled(true);
                        land_button->setEnabled(true);
                        return_button->setEnabled(true);
                    }
                    if ((current_mode == mavros_msgs::State::MODE_PX4_LAND) && !judge_all_land_state(false))
                    {
                        operating_info = "Landing...";
                        mode_button->setEnabled(false);
                        takeoff_button->setEnabled(false);
                        arm_button->setEnabled(false);
                        disarm_button->setEnabled(false);
                        manual_button->setEnabled(false);
                        trajectory_button->setEnabled(false);
                        external_button->setEnabled(false);
                        hover_button->setEnabled(false);
                        land_button->setEnabled(false);
                        return_button->setEnabled(false);
                    }
                    if ((current_mode == mavros_msgs::State::MODE_PX4_RTL) && !judge_all_land_state(false))
                    {
                        operating_info = "Returning...";
                        mode_button->setEnabled(false);
                        takeoff_button->setEnabled(false);
                        arm_button->setEnabled(false);
                        disarm_button->setEnabled(false);
                        manual_button->setEnabled(false);
                        trajectory_button->setEnabled(false);
                        external_button->setEnabled(false);
                        hover_button->setEnabled(false);
                        land_button->setEnabled(false);
                        // fix wing need to land manually
                        if (fix_wing_include)
                        {
                            land_button->setEnabled(true);
                        }
                        return_button->setEnabled(false);
                    }
                }
                else
                {
                    state_label_1->setText("[Arm State]  DisArm");
                    state_label_1->setStyleSheet("color: orange; font-size: 14pt");
                    arm_button->setEnabled(true);
                    disarm_button->setEnabled(false);
                    takeoff_button->setEnabled(true);
                    manual_button->setEnabled(false);
                    trajectory_button->setEnabled(false);
                    external_button->setEnabled(false);
                    hover_button->setEnabled(false);
                    land_button->setEnabled(false);
                    return_button->setEnabled(false);
                    if (current_mode == mavros_msgs::State::MODE_PX4_LAND || current_mode == mavros_msgs::State::MODE_PX4_RTL)
                    {
                        mode_button->setEnabled(true);
                        operating_info = "";
                    }
                }
                pre_arm_state = arm_state;
            }

            // current cmd
            if (pre_cmd != current_cmd)
            {
                state_label_2->setText("[Current CMD]  " + current_cmd);
                pre_cmd = current_cmd;
            }
            if (current_cmd == "External Command" && !ext_cmd_state)
            {
                current_cmd = "Hover";
                signal_button_1->click();
            }

            // update headers
            // update cmd mode info
            if (current_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                if (current_cmd == "Take Off")
                {
                    table_headers_ext_cmd[3] = "CMD 1  [x]";
                    table_headers_ext_cmd[4] = "CMD 2  [y]";
                    table_headers_ext_cmd[5] = "CMD 3  [z]";
                }
                else
                {
                    switch (cmds[0].Move_mode)
                    {
                        case px4_cmd::Command::XYZ_POS:
                            table_headers_ext_cmd[3] = "CMD 1  [x]";
                            table_headers_ext_cmd[4] = "CMD 2  [y]";
                            table_headers_ext_cmd[5] = "CMD 3  [z]";
                            break;

                        case px4_cmd::Command::XYZ_REL_POS:
                            table_headers_ext_cmd[3] = "CMD 1  [Rel x]";
                            table_headers_ext_cmd[4] = "CMD 2  [Rel y]";
                            table_headers_ext_cmd[5] = "CMD 3  [Rel z]";
                            break;

                        case px4_cmd::Command::XYZ_VEL:
                            table_headers_ext_cmd[3] = "CMD 1  [vx]";
                            table_headers_ext_cmd[4] = "CMD 2  [vy]";
                            table_headers_ext_cmd[5] = "CMD 3  [vz]";
                            break;

                        case px4_cmd::Command::XY_VEL_Z_POS:
                            table_headers_ext_cmd[3] = "CMD 1  [vx]";
                            table_headers_ext_cmd[4] = "CMD 2  [vy]";
                            table_headers_ext_cmd[5] = "CMD 3  [z]";
                            break;
                    }
                }
            }
            else
            {
                table_headers_ext_cmd[3] = "CMD 1";
                table_headers_ext_cmd[4] = "CMD 2";
                table_headers_ext_cmd[5] = "CMD 3";
            }
            info_model->setHorizontalHeaderLabels(table_headers_ext_cmd);

            // current mode
            if (pre_mode != current_mode)
            {
                info_label_3->setText(("[Current Mode]  " + current_mode).c_str());
                pre_mode = current_mode;
                if (current_mode == mavros_msgs::State::MODE_PX4_LAND)
                {
                    current_cmd = "Land";
                    state_label_2->setText("[Current CMD]  " + current_cmd);
                    pre_cmd = current_cmd;
                }
                if (current_mode == mavros_msgs::State::MODE_PX4_RTL)
                {
                    current_cmd = "Return";
                    state_label_2->setText("[Current CMD]  " + current_cmd);
                    pre_cmd = current_cmd;
                }
            }

            // operating info
            state_label_3->setText(operating_info);

            // ext_cmd_state
            ext_cmd_state_single = false;
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->ext_cmd_state)
                {
                    ext_cmd_state_single = true;
                    continue;
                }
                ext_cmd_state_single = false;
                break;
            }
            ext_cmd_state = ext_cmd_state_single;
        }

        void update_table_info(int node_id)
        {
            QString cmd_mode = "Loiter";
            QString pre_cmd = "";
            QString frame = "";
            string current_mode = "";
            string pre_mode = "";
            QString null_str = "  -----------  ";
            QString cmd1 = "";
            QString cmd2 = "";
            QString cmd3 = "";
            QString cmd4 = "";
            QVariant info;
            cell_info info1, info2, info3, info4, info5, info6, info7;
            info1.node_id = node_id;
            info2.node_id = node_id;
            info3.node_id = node_id;
            info4.node_id = node_id;
            info5.node_id = node_id;
            info6.node_id = node_id;
            info7.node_id = node_id;
            info1.cell_id = 1;
            info2.cell_id = 2;
            info3.cell_id = 3;
            info4.cell_id = 4;
            info5.cell_id = 5;
            info6.cell_id = 6;
            info7.cell_id = 7;
            while (ros::master::check() && !thread_stop)
            {
                // current mode
                current_mode = data[node_id]->state_mode;
                if (pre_mode != current_mode || pre_cmd != current_cmd)
                {
                    pre_mode = current_mode;
                    pre_cmd = current_cmd;
                    if (current_mode != mavros_msgs::State::MODE_PX4_OFFBOARD)
                    {
                        info2.str = null_str;
                        info3.str = null_str;
                        info4.str = null_str;
                        info5.str = null_str;
                        info6.str = null_str;
                        info.setValue(info2);
                        emit update_cell_info_signal(info);
                        info.setValue(info3);
                        emit update_cell_info_signal(info);
                        info.setValue(info4);
                        emit update_cell_info_signal(info);
                        info.setValue(info5);
                        emit update_cell_info_signal(info);
                        info.setValue(info6);
                        emit update_cell_info_signal(info);
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_LOITER)
                    {
                        cmd_mode = "Loiter";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_MANUAL)
                    {
                        cmd_mode = "Manual";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_LAND)
                    {
                        cmd_mode = "Land";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_RTL)
                    {
                        cmd_mode = "Return";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_POSITION)
                    {
                        cmd_mode = "Postition Control";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_ALTITUDE)
                    {
                        cmd_mode = "Altitude Control";
                    }
                    if (current_mode == mavros_msgs::State::MODE_PX4_STABILIZED)
                    {
                        cmd_mode = "Stabilized";
                    }
                }

                // ext_cmd_publish_info
                if (ext_cmd_state)
                {
                    info7.str = "  Active  ";
                    info.setValue(info7);
                    emit update_cell_info_signal(info);
                }
                else
                {
                    info7.str = "  Deactive  ";
                    info.setValue(info7);
                    emit update_cell_info_signal(info);
                }

                // table info
                if (current_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    // update cmd mode
                    if (current_cmd == "Take Off")
                    {
                        cmd_mode = "Take Off";
                    }
                    else
                    {
                        switch (cmds[0].Move_mode)
                        {
                            case px4_cmd::Command::XYZ_POS:
                                cmd_mode = "Postion";
                                break;

                            case px4_cmd::Command::XYZ_REL_POS:
                                cmd_mode = "Rel Position";
                                break;

                            case px4_cmd::Command::XYZ_VEL:
                                cmd_mode = "Velocity";
                                break;

                            case px4_cmd::Command::XY_VEL_Z_POS:
                                cmd_mode = "Velocity with Height";
                                break;
                        }
                    }

                    // update frame info
                    switch (cmds[0].Move_frame)
                    {
                        case px4_cmd::Command::ENU:
                            frame = "ENU";
                            break;

                        case px4_cmd::Command::BODY:
                            frame = "Body";
                            break;
                    }
                    // set frame info
                    info2.str = "  " + frame + "  ";
                    info.setValue(info2);
                    emit update_cell_info_signal(info);
                }
                
                // set cmd mode info
                info1.str = "  " + cmd_mode + "  ";
                info.setValue(info1);
                emit update_cell_info_signal(info);

                if (current_mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    cmd_mutex.lock();
                    info3.str = ("  " + to_string(cmd_values[node_id][0]) + "  ").c_str();
                    info4.str = ("  " + to_string(cmd_values[node_id][1]) + "  ").c_str();
                    info5.str = ("  " + to_string(cmd_values[node_id][2]) + "  ").c_str();
                    info6.str = ("  " + to_string(cmd_values[node_id][3]) + "  ").c_str();
                    cmd_mutex.unlock();
                    info.setValue(info3);
                    emit update_cell_info_signal(info);
                    info.setValue(info4);
                    emit update_cell_info_signal(info);
                    info.setValue(info5);
                    emit update_cell_info_signal(info);
                    info.setValue(info6);
                    emit update_cell_info_signal(info);
                }
                // sleep
                usleep(200000);
            }
        }

        void update_cell_info_slot(QVariant info)
        {
            cell_info data = info.value<cell_info>();
            table_items[data.node_id][data.cell_id]->setText(data.str);
            if (ext_cmd_state)
            {
                info_model->setData(info_model->index(data.node_id, 7), QBrush(QColor(0, 150, 0)), Qt::TextColorRole);
            }
            else
            {
                info_model->setData(info_model->index(data.node_id, 7), QBrush(Qt::red), Qt::TextColorRole);
            }
        };

        void update_button_state_slot(QVariant state)
        {
            button_info data = state.value<button_info>();
            data.button_name->setEnabled(data.state);
        };

        // utility functions
        bool judge_all_arm_state(bool desire)
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->arm_state != desire)
                {
                    return !desire;
                }
            }
            return desire;
        }

        bool judge_all_land_state(bool desire)
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->land_state != desire)
                {
                    return !desire;
                }
            }
            return desire;
        }

        bool judge_all_achieve_state(bool desire)
        {
            for (size_t i = 0; i < nodes.size(); i++)
            {
                if (data[i]->achieve_desire != desire)
                {
                    return !desire;
                }
            }
            return desire;
        }
};
#endif
