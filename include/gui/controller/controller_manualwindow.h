#ifndef CONTROLLERMANUALWINDOW_H
#define CONTROLLERMANUALWINDOW_H
#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QTableView>
#include <QHeaderView>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QLabel>
#include <string>
#include <vector>

#include <px4_cmd/Command.h>

using namespace std;

class ControllerManualWindow : public QWidget
{
    public:
        QWidget *parent;
        QDialog *win = new QDialog();
        int set_mode;
        int set_frame;
        bool exec_state = false;
        vector<vector<vector<double>>> cmd_values = {{{0, 0, 0, 0}}};
        ControllerManualWindow(QWidget *parent_widget)
        {

        }
        void set_nodes(QStringList input)
        {
            nodes = input;
            setup();
        }

    private:
        // init vector
        QStringList frames = {
            "ENU (Global Frame)",
            "Body (Relative Frame)"
        };
        QStringList modes = {
            "[Postion] x - y - z",
            "[Relative Position] x - y - z",
            "[Velocity] vx - vy - vz",
            "[Velocity with Altitude] vx - vy - z"
        };
        QStringList table_headers = {
            "Node",
            "CMD 1  [x]",
            "CMD 2  [y]",
            "CMD 3  [z]",
            "CMD Yaw (deg)"
        };
        QStringList nodes;
        vector<int> frames_msg = {
            px4_cmd::Command::ENU,
            px4_cmd::Command::BODY
        };
        vector<int> modes_msg = {
            px4_cmd::Command::XYZ_POS,
            px4_cmd::Command::XYZ_REL_POS,
            px4_cmd::Command::XYZ_VEL,
            px4_cmd::Command::XY_VEL_Z_POS
        };

        // init vars
        bool first = true;
        bool update_signal = false;

        // init widgets
        QMessageBox *msg_box;
        QLabel *title_txt;
        QLabel *frame_txt;
        QLabel *mode_txt;
        QComboBox *frame_select;
        QComboBox *mode_select;
        QPushButton *exit_button;
        QPushButton *exec_button;
        QTableView *cmd_table;
        QStandardItemModel *cmd_model;

        void setup()
        {
            // init
            exec_state = false;
            if (!first)
            {
                return;
            }

            for (auto i = 0; i < (nodes.size() - 1); i++)
            {
                vector<double> tmp = {0, 0, 0, 0};
                cmd_values[0].push_back(tmp);
            }

            // set win
            win->setFixedSize(1000, 450);
            win->setWindowTitle("Manual Cmd Setting");
            win->setStyleSheet("background-color: rgb(255,250,250)");

            // add layouts
            QVBoxLayout *vbox = new QVBoxLayout();
            QHBoxLayout *hbox_1 = new QHBoxLayout();
            QHBoxLayout *hbox_2 = new QHBoxLayout();

            // add labels
            title_txt = new QLabel("Manual Command Setting", win);
            title_txt->setStyleSheet("font-size: 18pt");
            title_txt->setAlignment(Qt::AlignCenter);
            frame_txt = new QLabel("Frame ", win);
            frame_txt->setStyleSheet("font-size: 14pt");
            frame_txt->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
            mode_txt = new QLabel("Mode ", win);
            mode_txt->setStyleSheet("font-size: 14pt");
            mode_txt->setAlignment(Qt::AlignVCenter | Qt::AlignRight);

            // add buttons
            exec_button = new QPushButton("Exec", win);
            exit_button = new QPushButton("Exit", win);
            exit_button->setMinimumHeight(50);
            exec_button->setMinimumHeight(50);
            exit_button->setStyleSheet("background-color: rgb(255,106,106); font-size: 16pt");
            exec_button->setStyleSheet("background-color: rgb(84,255,159); font-size: 16pt");

            // add select box & lineedit
            frame_select = new QComboBox(win);
            mode_select = new QComboBox(win);
            frame_select->addItems(frames);
            mode_select->addItems(modes);
            frame_select->setCurrentIndex(0);
            mode_select->setCurrentIndex(0);
            frame_select->setStyleSheet("background-color: rgb(155,205,155); font-size: 14pt");
            mode_select->setStyleSheet("background-color: rgb(135,206,235); font-size: 14pt");
            frame_select->setMinimumWidth(360);
            mode_select->setMinimumWidth(360);

            // add table
            cmd_table = new QTableView(win);
            cmd_table->setStyleSheet("background-color: rgb(236,245,225); font-size: 12pt");
            cmd_model = new QStandardItemModel(nodes.size(), table_headers.size(), win);
            cmd_model->setHorizontalHeaderLabels(table_headers);
            for (size_t i = 0; i < nodes.size(); i++)
            {
                QStandardItem *item_1 = new QStandardItem();
                QStandardItem *item_2 = new QStandardItem();
                QStandardItem *item_3 = new QStandardItem();
                QStandardItem *item_4 = new QStandardItem();
                QStandardItem *item_5 = new QStandardItem();
                item_1->setEditable(false);
                item_1->setTextAlignment(Qt::AlignCenter);
                item_2->setTextAlignment(Qt::AlignCenter);
                item_3->setTextAlignment(Qt::AlignCenter);
                item_4->setTextAlignment(Qt::AlignCenter);
                item_5->setTextAlignment(Qt::AlignCenter);
                item_1->setText(nodes[i]);
                item_2->setText(to_string(cmd_values[0][i][0]).c_str());
                item_3->setText(to_string(cmd_values[0][i][1]).c_str());
                item_4->setText(to_string(cmd_values[0][i][2]).c_str());
                item_5->setText(to_string(cmd_values[0][i][3]).c_str());
                cmd_model->setItem(i, 0, item_1);
                cmd_model->setItem(i, 1, item_2);
                cmd_model->setItem(i, 2, item_3);
                cmd_model->setItem(i, 3, item_4);
                cmd_model->setItem(i, 4, item_5);
            }
            cmd_table->setModel(cmd_model);
            cmd_table->horizontalHeader()->setStretchLastSection(true);
            cmd_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

            // set layouts
            vbox->setContentsMargins(30, 20, 30, 20);
            hbox_1->setAlignment(Qt::AlignCenter);
            vbox->addWidget(title_txt);
            vbox->addSpacing(20);
            hbox_1->addWidget(frame_txt);
            hbox_1->addWidget(frame_select);
            hbox_1->addSpacing(50);
            hbox_1->addWidget(mode_txt);
            hbox_1->addWidget(mode_select);
            vbox->addLayout(hbox_1);
            vbox->addSpacing(10);
            vbox->addWidget(cmd_table);
            hbox_2->addWidget(exit_button, 1);
            hbox_2->addStretch(3);
            hbox_2->addWidget(exec_button, 1);
            vbox->addLayout(hbox_2);
            win->setLayout(vbox);

            // connect signals and slots
            QObject::connect(cmd_model, &QStandardItemModel::itemChanged, this, &ControllerManualWindow::update_table_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &ControllerManualWindow::exit_slot);
            QObject::connect(exec_button, &QPushButton::clicked, this, &ControllerManualWindow::exec_slot);
            QObject::connect(mode_select, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ControllerManualWindow::mode_change_slot);
            QObject::connect(frame_select, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ControllerManualWindow::frame_change_slot);

            // first false and skip setup
            first = false;
            
            // init set_mode and set_frame
            set_mode = modes_msg[mode_select->currentIndex()];
            set_frame = frames_msg[frame_select->currentIndex()];
        }

        // slot functions
        void update_table_slot()
        {
            if (update_signal)
            {
                return;
            }
            int row = cmd_table->currentIndex().row();
            int column = cmd_table->currentIndex().column();
            QVariant data = cmd_table->currentIndex().data();
            string str = data.toString().toStdString();
            if (check_input_data(str))
            {
                cmd_values[0][row][column - 1] = stof(str);
            }
            else
            {
                update_signal = true;
                QStandardItem *item = new QStandardItem();
                item->setText(to_string(cmd_values[0][row][column - 1]).c_str());
                item->setTextAlignment(Qt::AlignCenter);
                cmd_model->setItem(row, column, item);
                update_signal = false;
            }
        }

        void frame_change_slot(int index)
        {
            set_frame = frames_msg[index];
        }

        void mode_change_slot(int index)
        {
            set_mode = modes_msg[index];
            switch (set_mode)
            {
                case px4_cmd::Command::XYZ_POS:
                    table_headers[1] = "CMD 1  [x]";
                    table_headers[2] = "CMD 2  [y]";
                    table_headers[3] = "CMD 3  [z]";
                    break;

                case px4_cmd::Command::XYZ_REL_POS:
                    table_headers[1] = "CMD 1  [Relative x]";
                    table_headers[2] = "CMD 2  [Relative y]";
                    table_headers[3] = "CMD 3  [Relative z]";
                    break;

                case px4_cmd::Command::XYZ_VEL:
                    table_headers[1] = "CMD 1  [vx]";
                    table_headers[2] = "CMD 2  [vy]";
                    table_headers[3] = "CMD 3  [vz]";
                    break;

                case px4_cmd::Command::XY_VEL_Z_POS:
                    table_headers[1] = "CMD 1  [vx]";
                    table_headers[2] = "CMD 2  [vy]";
                    table_headers[3] = "CMD 3  [z]";
                    break;
            }
            cmd_model->setHorizontalHeaderLabels(table_headers);
        }

        void exec_slot()
        {
            msg_box = new QMessageBox(QMessageBox::Question, "Confirmation", "Comfirm to Execute Commands?", QMessageBox::Yes | QMessageBox::No, win);
            int result = msg_box->exec();
            switch (result)
            {
                case QMessageBox::Yes:
                    win->close();
                    exec_state = true;
                    break;
                case QMessageBox::No:
                    exec_state = false;
                    return;
                    break;
            }
        }

        void exit_slot()
        {
            win->close();
        }

        // utility function
        bool check_input_data(string data)
        {
            double data_double;
            msg_box = new QMessageBox(win);
            msg_box->setIcon(QMessageBox::Icon::Critical);
            msg_box->setWindowTitle("Error");
            if (!data.compare(""))
            {
                msg_box->setText("Command Values Can not Be Empty!");
                msg_box->exec();
                return false;
            }
            try
            {
                data_double = stof(data);
            }
            catch (const std::exception &e)
            {
                msg_box->setText("Command Values Only Support Positive Float Type!");
                msg_box->exec();
                return false;
            }
            return true;
        }
};
#endif