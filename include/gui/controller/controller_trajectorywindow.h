#ifndef CONTROLLERTRAJECTORYWINDOW_H
#define CONTROLLERTRAJECTORYWINDOW_H
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

class ControllerTrajectoryWindow : public QWidget
{
    public:
        QWidget *parent;
        QDialog *win = new QDialog();
        int set_mode;
        int set_frame;
        int set_time;
        bool exec_state = false;
        vector<vector<vector<double>>> cmd_values = {{{0, 0, 0, 0}}};
        ControllerTrajectoryWindow(QWidget *parent_widget)
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
            px4_cmd::Command::XYZ_REL_POS
        };
        int uav_index = 0;

        // init vars
        bool update_signal = false;
        bool first = true;

        // init widgets
        QMessageBox *msg_box;
        QLabel *title_txt;
        QLabel *sub_title_1_txt;
        QLabel *sub_title_2_txt;
        QLabel *frame_txt;
        QLabel *mode_txt;
        QLabel *uav_txt;
        QLabel *time_txt;
        QComboBox *uav_select;
        QComboBox *frame_select;
        QComboBox *mode_select;
        QPushButton *exit_button;
        QPushButton *exec_button;
        QPushButton *add_button;
        QPushButton *del_button;
        QPushButton *clc_button;
        QLineEdit *time_input;
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
            win->setFixedSize(1200, 550);
            win->setWindowTitle("Trajectory Cmd Setting");
            win->setStyleSheet("background-color: rgb(255,250,250)");

            // add layouts
            QVBoxLayout *vbox = new QVBoxLayout();
            QVBoxLayout *vbox_button = new QVBoxLayout();
            QHBoxLayout *hbox_1 = new QHBoxLayout();
            QHBoxLayout *hbox_2 = new QHBoxLayout();
            QHBoxLayout *hbox_3 = new QHBoxLayout();
            QHBoxLayout *hbox_4 = new QHBoxLayout();

            // add labels
            title_txt = new QLabel("Trajectory Command Setting", win);
            title_txt->setStyleSheet("font-size: 18pt");
            title_txt->setAlignment(Qt::AlignCenter);
            sub_title_1_txt = new QLabel("[Global Setting]", win);
            sub_title_1_txt->setStyleSheet("font-size: 16pt; font-weight: bold; color: rgb(0,130,186)");
            sub_title_1_txt->setAlignment(Qt::AlignCenter);
            sub_title_2_txt = new QLabel("[Single Node Setting]", win);
            sub_title_2_txt->setStyleSheet("font-size: 16pt; font-weight: bold; color: rgb(0,131,0)");
            sub_title_2_txt->setAlignment(Qt::AlignCenter);
            uav_txt = new QLabel("Node ", win);
            uav_txt->setStyleSheet("font-size: 14pt");
            uav_txt->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
            uav_txt->setMaximumWidth(60);
            frame_txt = new QLabel("Frame ", win);
            frame_txt->setStyleSheet("font-size: 14pt");
            frame_txt->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
            mode_txt = new QLabel("Mode ", win);
            mode_txt->setStyleSheet("font-size: 14pt");
            mode_txt->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
            time_txt = new QLabel("Gap Time (second) ", win);
            time_txt->setStyleSheet("font-size: 14pt");
            time_txt->setAlignment(Qt::AlignVCenter | Qt::AlignRight);

            // add buttons
            add_button = new QPushButton("Add Point", win);
            del_button = new QPushButton("Delete Point", win);
            clc_button = new QPushButton("Clear Points", win);
            add_button->setStyleSheet("background-color: rgb(50,191,255); font-size: 14pt");
            del_button->setStyleSheet("background-color: rgb(255,106,106); font-size: 14pt");
            clc_button->setStyleSheet("background-color: rgb(224,102,255); font-size: 14pt");
            add_button->setMinimumHeight(55);
            del_button->setMinimumHeight(55);
            clc_button->setMinimumHeight(55);
            exec_button = new QPushButton("Exec", win);
            exit_button = new QPushButton("Exit", win);
            exit_button->setMinimumHeight(50);
            exec_button->setMinimumHeight(50);
            exit_button->setStyleSheet("background-color: rgb(255,106,106); font-size: 16pt");
            exec_button->setStyleSheet("background-color: rgb(84,255,159); font-size: 16pt");

            // add select box & lineedit
            uav_select = new QComboBox(win);
            frame_select = new QComboBox(win);
            mode_select = new QComboBox(win);
            uav_select->addItems(nodes);
            frame_select->addItems(frames);
            mode_select->addItems(modes);
            frame_select->setCurrentIndex(0);
            mode_select->setCurrentIndex(0);
            uav_select->setStyleSheet("background-color: rgb(155,205,155); font-size: 14pt");
            frame_select->setStyleSheet("background-color: rgb(135,206,235); font-size: 14pt");
            mode_select->setStyleSheet("background-color: rgb(135,206,235); font-size: 14pt");
            time_input = new QLineEdit("1", win);
            time_input->setStyleSheet("background-color: white; font-size: 14pt");

            // add table
            cmd_table = new QTableView(win);
            cmd_table->setStyleSheet("background-color: rgb(236,245,225); font-size: 12pt");
            cmd_model = new QStandardItemModel(cmd_values.size(), table_headers.size(), win);
            cmd_model->setHorizontalHeaderLabels(table_headers);
            QStandardItem *item_1;
            QStandardItem *item_2;
            QStandardItem *item_3;
            QStandardItem *item_4;
            QStandardItem *item_5;
            for (int i = 0; i < cmd_values.size(); i++)
            {
                item_1 = new QStandardItem();
                item_2 = new QStandardItem();
                item_3 = new QStandardItem();
                item_4 = new QStandardItem();
                item_5 = new QStandardItem();
                item_1->setEditable(false);
                item_1->setText(nodes[uav_index]);
                item_2->setText(to_string(cmd_values[i][uav_index][0]).c_str());
                item_3->setText(to_string(cmd_values[i][uav_index][1]).c_str());
                item_4->setText(to_string(cmd_values[i][uav_index][2]).c_str());
                item_5->setText(to_string(cmd_values[i][uav_index][3]).c_str());
                item_1->setTextAlignment(Qt::AlignCenter);
                item_2->setTextAlignment(Qt::AlignCenter);
                item_3->setTextAlignment(Qt::AlignCenter);
                item_4->setTextAlignment(Qt::AlignCenter);
                item_5->setTextAlignment(Qt::AlignCenter);
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
            vbox->addWidget(sub_title_1_txt);
            vbox->addSpacing(10);
            hbox_1->addWidget(frame_txt);
            hbox_1->addWidget(frame_select);
            hbox_1->addSpacing(40);
            hbox_1->addWidget(mode_txt);
            hbox_1->addWidget(mode_select);
            hbox_1->addSpacing(40);
            hbox_1->addWidget(time_txt);
            hbox_1->addWidget(time_input);
            vbox->addLayout(hbox_1);
            vbox->addSpacing(20);
            vbox->addWidget(sub_title_2_txt);
            vbox->addSpacing(10);
            hbox_2->addWidget(uav_txt);
            hbox_2->addWidget(uav_select);
            vbox->addLayout(hbox_2);
            vbox->addSpacing(10);
            hbox_3->addWidget(cmd_table, 4);
            hbox_3->addSpacing(15);
            vbox_button->addWidget(add_button);
            vbox_button->addWidget(del_button);
            vbox_button->addWidget(clc_button);
            hbox_3->addLayout(vbox_button, 1);
            vbox->addLayout(hbox_3);
            hbox_4->addWidget(exit_button, 1);
            hbox_4->addStretch(3);
            hbox_4->addWidget(exec_button, 1);
            vbox->addSpacing(10);
            vbox->addLayout(hbox_4);
            win->setLayout(vbox);

            // connect signals and slots
            QObject::connect(cmd_model, &QStandardItemModel::itemChanged, this, &ControllerTrajectoryWindow::update_table_slot);
            QObject::connect(add_button, &QPushButton::clicked, this, &ControllerTrajectoryWindow::add_button_slot);
            QObject::connect(del_button, &QPushButton::clicked, this, &ControllerTrajectoryWindow::del_button_slot);
            QObject::connect(clc_button, &QPushButton::clicked, this, &ControllerTrajectoryWindow::clc_button_slot);
            QObject::connect(exit_button, &QPushButton::clicked, this, &ControllerTrajectoryWindow::exit_slot);
            QObject::connect(exec_button, &QPushButton::clicked, this, &ControllerTrajectoryWindow::exec_slot);
            QObject::connect(mode_select, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ControllerTrajectoryWindow::mode_change_slot);
            QObject::connect(frame_select, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ControllerTrajectoryWindow::frame_change_slot);
            QObject::connect(uav_select, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ControllerTrajectoryWindow::uav_change_slot);

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
                cmd_values[row][uav_index][column - 1] = stof(str);
            }
            else
            {
                update_signal = true;
                QStandardItem *item = new QStandardItem();
                item->setText(to_string(cmd_values[row][uav_index][column - 1]).c_str());
                item->setTextAlignment(Qt::AlignCenter);
                cmd_model->setItem(row, column, item);
                update_signal = false;
            }
        }

        void uav_change_slot(int index)
        {
            // update index
            uav_index = index;

            // update table
            update_signal = true;
            QStandardItem *item_1;
            QStandardItem *item_2;
            QStandardItem *item_3;
            QStandardItem *item_4;
            QStandardItem *item_5;
            for (int i = 0; i < cmd_values.size(); i++)
            {
                item_1 = new QStandardItem();
                item_2 = new QStandardItem();
                item_3 = new QStandardItem();
                item_4 = new QStandardItem();
                item_5 = new QStandardItem();
                item_1->setEditable(false);
                item_1->setText(nodes[uav_index]);
                item_2->setText(to_string(cmd_values[i][uav_index][0]).c_str());
                item_3->setText(to_string(cmd_values[i][uav_index][1]).c_str());
                item_4->setText(to_string(cmd_values[i][uav_index][2]).c_str());
                item_5->setText(to_string(cmd_values[i][uav_index][3]).c_str());
                item_1->setTextAlignment(Qt::AlignCenter);
                item_2->setTextAlignment(Qt::AlignCenter);
                item_3->setTextAlignment(Qt::AlignCenter);
                item_4->setTextAlignment(Qt::AlignCenter);
                item_5->setTextAlignment(Qt::AlignCenter);
                cmd_model->setItem(i, 0, item_1);
                cmd_model->setItem(i, 1, item_2);
                cmd_model->setItem(i, 2, item_3);
                cmd_model->setItem(i, 3, item_4);
                cmd_model->setItem(i, 4, item_5);
            }
            update_signal = false;
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

        void add_button_slot()
        {
            // none values -> add init vector
            if (cmd_values.size() == 0)
            {
                cmd_values = {{{0, 0, 0, 0}}};
                for (auto i = 0; i < (nodes.size() - 1); i++)
                {
                    vector<double> tmp = {0, 0, 0, 0};
                    cmd_values[0].push_back(tmp);
                }
            }
            else
            {
                // else add vector as previous one
                cmd_values.push_back(*(cmd_values.end() - 1));
            }
            // update table
            update_signal = true;
            QStandardItem *item_1 = new QStandardItem();
            QStandardItem *item_2 = new QStandardItem();
            QStandardItem *item_3 = new QStandardItem();
            QStandardItem *item_4 = new QStandardItem();
            QStandardItem *item_5 = new QStandardItem();
            item_1->setEditable(false);
            item_1->setText(nodes[uav_index]);
            item_2->setText(to_string(cmd_values[cmd_values.size() - 1][uav_index][0]).c_str());
            item_3->setText(to_string(cmd_values[cmd_values.size() - 1][uav_index][1]).c_str());
            item_4->setText(to_string(cmd_values[cmd_values.size() - 1][uav_index][2]).c_str());
            item_5->setText(to_string(cmd_values[cmd_values.size() - 1][uav_index][3]).c_str());
            item_1->setTextAlignment(Qt::AlignCenter);
            item_2->setTextAlignment(Qt::AlignCenter);
            item_3->setTextAlignment(Qt::AlignCenter);
            item_4->setTextAlignment(Qt::AlignCenter);
            item_5->setTextAlignment(Qt::AlignCenter);
            cmd_model->appendRow({item_1, item_2, item_3, item_4, item_5});
            update_signal = false;
        }

        void del_button_slot()
        {
            // get selected index
            QModelIndexList indexes = cmd_table->selectionModel()->selectedIndexes();
            if (indexes.length() == 0)
            {
                msg_box = new QMessageBox(win);
                msg_box->setIcon(QMessageBox::Icon::Information);
                msg_box->setWindowTitle("Info");
                msg_box->setText("Please Select Point");
                msg_box->exec();
                return;
            }
            // delete selected index
            int num = 0;
            int row = 0;
            update_signal = true;
            for (auto item = indexes.begin(); item != indexes.end(); item++)
            {
                row = item->row();
                row = row - num;
                num++;
                // remove data
                cmd_values.erase(cmd_values.begin() + row);
                // remove table row
                cmd_model->removeRow(row);
            }
            update_signal = false;
        }

        void clc_button_slot()
        {
            // clear data
            cmd_values.clear();
            // clear table
            update_signal = true;
            cmd_model->clear();
            cmd_model->setHorizontalHeaderLabels(table_headers);
            update_signal = false;
        }

        void exec_slot()
        {
            string str = time_input->text().toStdString();
            if (check_input_data(str, true))
            {
                set_time = stof(str);
            }
            else
            {
                time_input->setText(QString(set_time));
                return;
            }
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
        bool check_input_data(string data, bool positive = false)
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
                msg_box->setText("Gap Time Values Only Support Float Type!");
                msg_box->exec();
                return false;
            }
            if (positive && data_double <= 0)
            {
                msg_box->setText("Gap Time Only Support Positive Float Type!");
                msg_box->exec();
                return false;
            }
            return true;
        }
};
#endif