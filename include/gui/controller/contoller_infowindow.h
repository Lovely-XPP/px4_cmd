#ifndef CONTROLLERINFOWINDOW_H
#define CONTROLLERINFOWINDOW_H
#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextBrowser>
#include <string>
#include <vector>
#include <iostream>

#include <print_utility/printf_utility.h>

class ControllerInfoWindow : public QWidget
{
    public:
        QWidget *parent;
        QDialog *win = new QDialog();
        ControllerInfoWindow(QWidget *parent_widget)
        {
            setup();
        }

    private:
        // init widgets
        QVBoxLayout *vbox = new QVBoxLayout();
        QHBoxLayout *hbox = new QHBoxLayout();
        QPushButton *programme_info_button = new QPushButton(win);
        QPushButton *topic_name_info_button = new QPushButton(win);
        QPushButton *sensors_info_button = new QPushButton(win);
        QTextBrowser *txt = new QTextBrowser(win);

        void setup()
        {
            // set win
            win->setFixedSize(600, 600);
            win->setWindowTitle("PX4 Cmd Launch File Generator");
            win->setStyleSheet("background-color: rgb(255,250,250)");
            // set buttons
            programme_info_button->setMinimumSize(200, 40);
            programme_info_button->setText("About the Programme");
            topic_name_info_button->setMinimumSize(200, 40);
            topic_name_info_button->setText("About Topics");
            sensors_info_button->setMinimumSize(200, 40);
            sensors_info_button->setText("About Sensors");

            // set layout
            vbox->addWidget(programme_info_button);
            vbox->addWidget(topic_name_info_button);
            vbox->addWidget(sensors_info_button);
            hbox->addLayout(vbox);

            // show text
            txt->setStyleSheet("background-color: white");
            txt->setText("Welcome.");
            hbox->addWidget(txt);

            // set layout
            win->setLayout(hbox);

            // Signal connect slot
            QObject::connect(programme_info_button, &QPushButton::clicked, this, &ControllerInfoWindow::programme_info_slot);
            QObject::connect(topic_name_info_button, &QPushButton::clicked, this, &ControllerInfoWindow::topic_name_info_slot);
            QObject::connect(sensors_info_button, &QPushButton::clicked, this, &ControllerInfoWindow::sensors_info_slot);
        }

        // slot functions
        void programme_info_slot()
        {
            txt->setText("This is programme information.");
        }

        void topic_name_info_slot()
        {
            txt->setText("This is topic information.");
        }

        void sensors_info_slot()
        {
            txt->setText("This is sensors information.");
        }
};
#endif