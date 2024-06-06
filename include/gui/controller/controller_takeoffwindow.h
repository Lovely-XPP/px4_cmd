// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef CONTROLLER_TAKEOFFWINDOW_H
#define CONTROLLER_TAKEOFFWINDOW_H
#include <QMainWindow>
#include <QApplication>
#include <QDialog>
#include <QMessageBox>
#include <QLabel>
#include <QVector>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>
#include <string>
#include <vector>
#include <thread>

#include <vehicle_command.h>
#include <mavros_msgs/State.h>

class ControllerTakeoffWindow : public QDialog
{   
    Q_OBJECT
    public:
        double takeoff_height;
        bool set_height = false;
        ControllerTakeoffWindow(QWidget *parent_widget = 0)
        {
            setup();
        }
        bool set_data(QVector<vehicle_command *> input)
        {
            set_height = false;
            data = input;
            QString err = "";
            msg_box = new QMessageBox(this);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            for (auto item = data.begin(); item != data.end(); item++)
            {
                if ((*item)->state_mode != mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    msg_box->setText("Take Off by Programme Need OFFBOARD Mode.");
                    msg_box->exec();
                    return false;
                }
                if((*item)->arm_state && !(*item)->land_state)
                {
                    msg_box->setText("Vehicle has Already Taken Off!");
                    msg_box->exec();
                    return false;
                }
            }
            return true;
        }

    signals:
        void take_off_info_signal();

    private:
        // init data
        QVector<vehicle_command *> data;

        // init widgets
        QMessageBox *msg_box;
        QPushButton *exec_button;
        QPushButton *exit_button;
        QLineEdit *takeoff_height_txt;

        // add layouts
        QVBoxLayout *vbox = new QVBoxLayout();
        QHBoxLayout *hbox_1 = new QHBoxLayout();
        QHBoxLayout *hbox_2 = new QHBoxLayout();

        void setup()
        {
            // set this
            this->setFixedSize(600, 160);
            this->setWindowTitle("Take Off Setting");
            this->setStyleSheet("background-color: rgb(255,250,250)");

            // set label
            QLabel *txt = new QLabel("Take off height (meters):", this);
            txt->setStyleSheet("font-size: 14pt");

            // set buttons
            exit_button = new QPushButton("Exit", this);
            exec_button = new QPushButton("Exec and Arm", this);
            exit_button->setMinimumHeight(50);
            exec_button->setMinimumHeight(50);
            exit_button->setStyleSheet("background-color: rgb(255,106,106); font-size: 16pt");
            exec_button->setStyleSheet("background-color: rgb(84,255,159); font-size: 16pt");

            // set line edit
            takeoff_height_txt = new QLineEdit("2", this);
            takeoff_height_txt->setStyleSheet("font-size: 16pt");

            // set layout
            hbox_1->setAlignment(Qt::AlignHCenter);
            hbox_1->setSpacing(30);
            hbox_2->setSpacing(50);
            vbox->addSpacing(15);
            hbox_1->addWidget(txt);
            hbox_1->addWidget(takeoff_height_txt);
            vbox->addLayout(hbox_1);
            vbox->addSpacing(15);
            hbox_2->addWidget(exit_button);
            hbox_2->addWidget(exec_button);
            vbox->addLayout(hbox_2);
            this->setLayout(vbox);

            // Signal connect slot
            QObject::connect(exec_button, &QPushButton::clicked, this, &ControllerTakeoffWindow::exec_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &ControllerTakeoffWindow::exit_slot);
        }

        // slot functions
        void exec_slot()
        {
            QString height = takeoff_height_txt->text();
            if(!check_input_data(height.toStdString()))
            {
                return;
            }
            emit take_off_info_signal();
            takeoff_height = height.toDouble();
            set_height = true;
            this->close();
        }

        void exit_slot()
        {
            this->close();
        }

        // utility function
        bool check_input_data(string data)
        {
            double data_double;
            msg_box = new QMessageBox(this);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            if (!data.compare(""))
            {
                msg_box->setText("Takeoff Height Can not Be Empty!");
                msg_box->exec();
                return false;
            }
            try
            {
                data_double = stof(data);
            }
            catch (const std::exception &e)
            {
                msg_box->setText("Takeoff Height Only Support Positive Float Type!");
                msg_box->exec();
                return false;
            }
            if (data_double <= 0)
            {
                msg_box->setText("Takeoff Height Only Support Positive Float Type!");
                msg_box->exec();
                return false;
            }
            return true;
        }
};
#endif