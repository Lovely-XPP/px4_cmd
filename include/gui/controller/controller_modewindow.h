#ifndef CONTROLLERMODEWINDOW_H
#define CONTROLLERMODEWINDOW_H
#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextBrowser>
#include <QStringList>
#include <QMessageBox>
#include <QVector>
#include <string>
#include <vector>
#include <iostream>
#include <thread>

#include <print_utility/printf_utility.h>
#include <mavros_msgs/State.h>
#include <vehicle_command.h>

class ControllerModeWindow : public QWidget
{
    public:
        QWidget *parent;
        QDialog *win = new QDialog();
        ControllerModeWindow(QWidget *parent_widget)
        {
            setup();
        }
        void set_data(QVector<vehicle_command *> input)
        {
            data = input;
            update_mode();
        }

    private:
        // init widgets
        QMessageBox *msg_box;
        QPushButton *signal_button;
        QPushButton *manual_mode;
        QPushButton *stable_mode;
        QPushButton *offboard_mode;
        QPushButton *pos_mode;
        QPushButton *alt_mode;
        QPushButton *land_mode;
        QPushButton *return_mode;
        QPushButton *exit_mode;
        QLabel *state_txt;
        QVector<vehicle_command *> data;
        QString error;
        string pre_mode;

        void setup()
        {
            // set win
            win->setFixedSize(1000, 250);
            win->setWindowTitle("Mode Setting");
            win->setStyleSheet("background-color: rgb(255,250,250)");

            // add Layout
            QVBoxLayout *vbox = new QVBoxLayout();
            QHBoxLayout *hbox_1 = new QHBoxLayout();
            QHBoxLayout *hbox_2 = new QHBoxLayout();
            QHBoxLayout *hbox_3 = new QHBoxLayout();

            // add text
            state_txt = new QLabel("", win);
            state_txt->setAlignment(Qt::AlignmentFlag::AlignCenter);
            QLabel *info_txt = new QLabel("Note: [POSCTL] = Position Control, [ALTCTL] = Altitude Control, [OFFBOARD] = programme working mode", win);
            info_txt->setAlignment(Qt::AlignmentFlag::AlignCenter);
            info_txt->setStyleSheet("font-size: 12pt");

            // add buttons
            manual_mode = new QPushButton("Manual", win);
            stable_mode = new QPushButton("Stablized", win);
            offboard_mode = new QPushButton("Offboard", win);
            pos_mode = new QPushButton("Position Control", win);
            alt_mode = new QPushButton("Altitude Control", win);
            land_mode = new QPushButton("Auto Land", win);
            return_mode = new QPushButton("Auto Return", win);
            exit_mode = new QPushButton("Exit", win);
            manual_mode->setStyleSheet("font-size: 16pt");
            stable_mode->setStyleSheet("font-size: 16pt");
            offboard_mode->setStyleSheet("font-size: 16pt");
            pos_mode->setStyleSheet("font-size: 16pt");
            alt_mode->setStyleSheet("font-size: 16pt");
            land_mode->setStyleSheet("font-size: 16pt");
            return_mode->setStyleSheet("font-size: 16pt");
            exit_mode->setStyleSheet("font-size: 16pt");
            manual_mode->setMinimumHeight(50);
            stable_mode->setMinimumHeight(50);
            offboard_mode->setMinimumHeight(50);
            pos_mode->setMinimumHeight(50);
            alt_mode->setMinimumHeight(50);
            exit_mode->setMinimumHeight(50);
            land_mode->setMinimumHeight(50);
            return_mode->setMinimumHeight(50);
            signal_button = new QPushButton(win);
            signal_button->setVisible(false);
            
            // set layout
            hbox_1->setSpacing(20);
            hbox_2->setSpacing(20);
            vbox->setSpacing(20);
            vbox->addWidget(state_txt, 1);
            vbox->addWidget(info_txt, 1);
            hbox_1->addWidget(manual_mode);
            hbox_1->addWidget(stable_mode);
            hbox_1->addWidget(offboard_mode);
            hbox_1->addWidget(exit_mode);
            vbox->addLayout(hbox_1, 6);
            hbox_2->addWidget(pos_mode);
            hbox_2->addWidget(alt_mode);
            hbox_2->addWidget(land_mode);
            hbox_2->addWidget(return_mode);
            vbox->addLayout(hbox_2, 6);

            // set layout
            win->setLayout(vbox);

            // Signal connect slot
            QObject::connect(manual_mode, &QPushButton::clicked, this, &ControllerModeWindow::manual_mode_slot);
            QObject::connect(offboard_mode, &QPushButton::clicked, this, &ControllerModeWindow::offboard_mode_slot);
            QObject::connect(stable_mode, &QPushButton::clicked, this, &ControllerModeWindow::stable_mode_slot);
            QObject::connect(pos_mode, &QPushButton::clicked, this, &ControllerModeWindow::pos_mode_slot);
            QObject::connect(alt_mode, &QPushButton::clicked, this, &ControllerModeWindow::alt_mode_slot);
            QObject::connect(land_mode, &QPushButton::clicked, this, &ControllerModeWindow::land_mode_slot);
            QObject::connect(return_mode, &QPushButton::clicked, this, &ControllerModeWindow::return_mode_slot);
            QObject::connect(exit_mode, &QPushButton::clicked, this, &ControllerModeWindow::exit_mode_slot);
            QObject::connect(signal_button, &QPushButton::clicked, this, &ControllerModeWindow::show_error);
        }

        // slot functions
        void manual_mode_slot()
        {
            change_mode_txt(mavros_msgs::State::MODE_PX4_MANUAL);
            std::thread ros_thread(&ControllerModeWindow::change_mode, this, mavros_msgs::State::MODE_PX4_MANUAL);
            ros_thread.detach();
        }

        void offboard_mode_slot()
        {
            change_mode_txt(mavros_msgs::State::MODE_PX4_OFFBOARD);
            std::thread ros_thread(&ControllerModeWindow::change_mode, this, mavros_msgs::State::MODE_PX4_OFFBOARD);
            ros_thread.detach();
        }

        void stable_mode_slot()
        {
            change_mode_txt(mavros_msgs::State::MODE_PX4_STABILIZED);
            std::thread ros_thread(&ControllerModeWindow::change_mode, this, mavros_msgs::State::MODE_PX4_STABILIZED);
            ros_thread.detach();
        }

        void pos_mode_slot()
        {
            change_mode_txt(mavros_msgs::State::MODE_PX4_POSITION);
            std::thread ros_thread(&ControllerModeWindow::change_mode, this, mavros_msgs::State::MODE_PX4_POSITION);
            ros_thread.detach();
        }

        void alt_mode_slot()
        {
            change_mode_txt(mavros_msgs::State::MODE_PX4_ALTITUDE);
            std::thread ros_thread(&ControllerModeWindow::change_mode, this, mavros_msgs::State::MODE_PX4_ALTITUDE);
            ros_thread.detach();
        }

        void land_mode_slot()
        {
            change_mode_txt(mavros_msgs::State::MODE_PX4_LAND);
            std::thread ros_thread(&ControllerModeWindow::change_mode, this, mavros_msgs::State::MODE_PX4_LAND);
            ros_thread.detach();
        }

        void return_mode_slot()
        {
            change_mode_txt(mavros_msgs::State::MODE_PX4_RTL);
            std::thread ros_thread(&ControllerModeWindow::change_mode, this, mavros_msgs::State::MODE_PX4_RTL);
            ros_thread.detach();
        }

        void exit_mode_slot()
        {
            win->close();
        }

        // utility functions
        void update_mode()
        {
            pre_mode = data[0]->state_mode;
            state_txt->setText(("[Current Mode]  " + pre_mode).c_str());
            state_txt->setStyleSheet("color: green; font-size: 16pt");
        }

        void change_mode_txt(string desire_mode)
        {
            state_txt->setText(("Changing Mode: [" + pre_mode + "] -> [" + desire_mode + "]").c_str());
            state_txt->setStyleSheet("color: orange; font-size: 16pt");
            manual_mode->setEnabled(false);
            stable_mode->setEnabled(false);
            offboard_mode->setEnabled(false);
            pos_mode->setEnabled(false);
            alt_mode->setEnabled(false);
            land_mode->setEnabled(false);
            return_mode->setEnabled(false);
            exit_mode->setEnabled(false);
        }

        void change_mode(string desire_mode)
        {
            string err = "";
            for (auto item = data.begin(); item != data.end(); item++)
            {
                err = (*item)->set_mode(desire_mode).c_str();
                if (err != "")
                {
                    error = err.c_str();
                    signal_button->click();
                    for (auto item = data.begin(); item != data.end(); item++)
                    {
                        (*item)->set_mode(pre_mode).c_str();
                    }
                    break;
                }
            }
            update_mode();
            manual_mode->setEnabled(true);
            stable_mode->setEnabled(true);
            offboard_mode->setEnabled(true);
            pos_mode->setEnabled(true);
            alt_mode->setEnabled(true);
            land_mode->setEnabled(true);
            return_mode->setEnabled(true);
            exit_mode->setEnabled(true);
        }

        void show_error()
        {
            msg_box = new QMessageBox(win);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setText("Error");
            msg_box->setWindowTitle("Error");
            msg_box->setText(error);
            msg_box->exec();
            error = "";
        }
};
#endif