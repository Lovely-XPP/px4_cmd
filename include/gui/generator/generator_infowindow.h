// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef GENERATORINFOWINDOW_H
#define GENERATORINFOWINDOW_H
#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextBrowser>
#include <string>
#include <vector>
#include <iostream>

class GeneratorInfoWindow : public QDialog
{
    public:
        GeneratorInfoWindow(QWidget *parent_widget)
        {
            setup();
        }

    private:
        // init widgets
        QVBoxLayout *vbox = new QVBoxLayout();
        QHBoxLayout *hbox = new QHBoxLayout();
        QPushButton *programme_info_button = new QPushButton(this);
        QPushButton *topic_name_info_button = new QPushButton(this);
        QPushButton *sensors_info_button = new QPushButton(this);
        QTextBrowser *txt = new QTextBrowser(this);

        void setup()
        {
            // set this
            this->setFixedSize(600, 600);
            this->setWindowTitle("About");
            this->setStyleSheet("background-color: rgb(255,250,250)");
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
            this->setLayout(hbox);

            // Signal connect slot
            QObject::connect(programme_info_button, &QPushButton::clicked, this, &GeneratorInfoWindow::programme_info_slot);
            QObject::connect(topic_name_info_button, &QPushButton::clicked, this, &GeneratorInfoWindow::topic_name_info_slot);
            QObject::connect(sensors_info_button, &QPushButton::clicked, this, &GeneratorInfoWindow::sensors_info_slot);
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