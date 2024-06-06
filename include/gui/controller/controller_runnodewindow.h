// Copyright (c) 2023 易鹏 中山大学航空航天学院
// Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

#ifndef CONTROLLER_RUNNODEWINDOW_H
#define CONTROLLER_RUNNODEWINDOW_H
#include <QApplication>
#include <QDialog>
#include <QBrush>
#include <QColor>
#include <QMenu>
#include <QPoint>
#include <QComboBox>
#include <QLineEdit>
#include <QStringList>
#include <QPushButton>
#include <QAbstractItemView>
#include <QLabel>
#include <QTableView>
#include <QHeaderView>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <thread>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>

#include <ros/ros.h>
#include <ros/package.h>
#include <regex>

class ControllerRunNodeWindow : public QDialog
{
    Q_OBJECT
    public:
        ControllerRunNodeWindow(ros::NodeHandle &nh)
        {
            nh.getParam("rosdistro", ros_distro);
            ros_distro = std::regex_replace(ros_distro, std::regex("\n"), "");
            ros_distro = "/opt/ros/" + ros_distro;
            setup();
        }

        ~ControllerRunNodeWindow()
        {
            for (size_t i = 0; i < ros_node_threads.size(); i++)
            {
                if (!ros_node_threads[i]->joinable())
                {
                    std::terminate_handler(ros_node_threads[i]->native_handle());
                }
            }
        }

        void reset_window()
        {
            run_mode_select->setCurrentIndex(0);
            run_pack_select->setCurrentIndex(0);
            run_exec_select->clear();
            run_exec_select->addItem("--- Select Exec ---");
            run_exec_select->setCurrentIndex(0);
            run_exec_select->setEnabled(false);
            run_button->setEnabled(false);
            args_edit->setEnabled(false);
        }

    signals:
        void update_table_signal();

    private:
        // ros vars
        std::string ros_distro;

        // thread state
        std::vector<std::thread *> ros_node_threads;
        std::vector<QStandardItem *> state_items;
        std::vector<bool> ros_node_threads_run_state;
        std::vector<bool> ros_node_threads_terminal_flag;

        // Widgets
        QLabel *run_mode_txt;
        QComboBox *run_mode_select;
        QLabel *run_pack_txt;
        QComboBox *run_pack_select;
        QLabel *run_exec_txt;
        QComboBox *run_exec_select;
        QLabel *args_txt;
        QLineEdit *args_edit;
        QTableView *table_info;
        QStandardItemModel *model_info;
        QPushButton *run_button;
        QStringList thread_states = {
            "State",
            "Command",
            "Package",
            "Launch File / Execute Node",
            "Args"};

        void setup()
        {
            // set this
            this->setFixedSize(1000, 450);
            this->setWindowTitle("Run Ros Node");
            this->setStyleSheet("background-color: rgb(255,250,250)");

            // setup run_mode_select
            run_mode_txt = new QLabel("Mode:", this);
            run_mode_txt->setStyleSheet("font-size: 12pt");
            run_mode_txt->setFixedWidth(55);
            run_mode_select = new QComboBox(this);
            run_mode_select->setStyleSheet("background-color: rgb(91, 155, 213); font-size: 12pt");
            run_mode_select->addItem("rosrun");
            run_mode_select->addItem("roslaunch");
            
            // setup run_pack_select
            run_pack_txt = new QLabel("Pack:", this);
            run_pack_txt->setStyleSheet("font-size: 12pt");
            run_pack_txt->setFixedWidth(55);
            run_pack_select = new QComboBox(this);
            run_pack_select->setStyleSheet("background-color: rgb(155, 205, 155); font-size: 12pt");
            run_pack_select->view()->setMaximumHeight(300);
            run_pack_select->view()->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
            run_pack_select->findChild<QFrame *>()->setMaximumHeight(300);
            ros::V_string ros_all_packs;
            ros::package::getAll(ros_all_packs);
            run_pack_select->addItem("--- Select Package ---");
            for (auto ros_pack : ros_all_packs)
            {
                run_pack_select->addItem(QString::fromStdString(ros_pack));
            }

            // setup run_exec_select
            run_exec_txt = new QLabel("Exec:", this);
            run_exec_txt->setStyleSheet("font-size: 12pt");
            run_exec_txt->setFixedWidth(55);
            run_exec_select = new QComboBox(this);
            run_exec_select->setStyleSheet("background-color: rgb(155, 205, 155); font-size: 12pt");
            run_exec_select->view()->setMaximumHeight(300);
            run_exec_select->view()->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
            run_exec_select->findChild<QFrame *>()->setMaximumHeight(300);
            run_exec_select->addItem("--- Select Exec ---");
            run_exec_select->setEnabled(false);

            // setup line edit
            args_txt = new QLabel("Args:", this);
            args_txt->setStyleSheet("font-size: 12pt");
            args_txt->setFixedWidth(55);
            args_edit = new QLineEdit(this);
            args_edit->setEnabled(false);
            args_edit->setStyleSheet("font-size: 12pt");

            // setup run button
            run_button = new QPushButton("Run", this);
            run_button->setStyleSheet("background-color: rgb(129,79,255); font-size: 20pt; font-weight: bold;");
            run_button->setEnabled(false);
            run_button->setMinimumHeight(65);
            run_button->setMinimumWidth(200);

            // setup table
            table_info = new QTableView(this);
            table_info->setContextMenuPolicy(Qt::CustomContextMenu);
            model_info = new QStandardItemModel(0, thread_states.size(), this);
            model_info->setHorizontalHeaderLabels(thread_states);
            table_info->setModel(model_info);
            table_info->horizontalHeader()->setStretchLastSection(true);
            table_info->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
            table_info->setStyleSheet("background-color: rgb(236,245,225); font-size: 12pt");

            // setup Layout
            QVBoxLayout *vbox = new QVBoxLayout();
            QHBoxLayout *hbox = new QHBoxLayout();
            QVBoxLayout *vbox_select = new QVBoxLayout();
            QHBoxLayout *hbox_mode = new QHBoxLayout();
            QHBoxLayout *hbox_pack = new QHBoxLayout();
            QHBoxLayout *hbox_exec = new QHBoxLayout();
            QHBoxLayout *hbox_args = new QHBoxLayout();

            // select cmd and nodes / launch layout
            hbox_mode->addWidget(run_mode_txt, Qt::AlignLeft);
            hbox_mode->addWidget(run_mode_select, Qt::AlignLeft);
            vbox_select->addLayout(hbox_mode);
            hbox_pack->setAlignment(Qt::AlignLeft);
            hbox_pack->addWidget(run_pack_txt, Qt::AlignLeft);
            hbox_pack->addWidget(run_pack_select, Qt::AlignLeft);
            vbox_select->addLayout(hbox_pack);
            hbox_exec->setAlignment(Qt::AlignLeft);
            hbox_exec->addWidget(run_exec_txt, Qt::AlignLeft);
            hbox_exec->addWidget(run_exec_select, Qt::AlignLeft);
            vbox_select->addLayout(hbox_exec);
            hbox->addLayout(vbox_select, 4);
            hbox->addSpacing(20);
            hbox->addWidget(run_button, 1);
            vbox->addLayout(hbox);

            // args layout
            hbox_args->addWidget(args_txt);
            hbox_args->addWidget(args_edit);
            vbox->addLayout(hbox_args);

            // add tableview layout
            vbox->addWidget(table_info);

            // set window layout
            this->setLayout(vbox);

            // QOBJECT connect signal and slot funtion
            QObject::connect(run_mode_select, &QComboBox::currentTextChanged, this, &ControllerRunNodeWindow::change_mode_slot);
            QObject::connect(run_pack_select, &QComboBox::currentTextChanged, this, &ControllerRunNodeWindow::change_pack_slot);
            QObject::connect(run_exec_select, &QComboBox::currentTextChanged, this, &ControllerRunNodeWindow::change_exec_slot);
            QObject::connect(table_info, &QTableView::customContextMenuRequested, this, &ControllerRunNodeWindow::table_click_slot);
            QObject::connect(run_button, &QPushButton::clicked, this, &ControllerRunNodeWindow::run_button_slot);
            QObject::connect(this, &ControllerRunNodeWindow::update_table_signal, this, &ControllerRunNodeWindow::update_table_slot);
        }

        // slot functions
        void change_mode_slot()
        {
            run_pack_select->blockSignals(true);
            run_pack_select->setCurrentIndex(0);
            run_exec_select->clear();
            if (run_mode_select->currentIndex() == 0)
            {
                run_exec_select->addItem("--- Select Exec ---");
            }
            else
            {
                run_exec_select->addItem("--- Select Launch ---");
            }
            run_exec_select->setEnabled(false);
            run_exec_select->setCurrentIndex(0);
            run_pack_select->blockSignals(false);
        }

        void change_pack_slot()
        {
            run_exec_select->clear();
            if (run_mode_select->currentIndex() == 0)
            {
                run_exec_select->addItem("--- Select Exec ---");
            }
            else
            {
                run_exec_select->addItem("--- Select Launch ---");
            }
            run_exec_select->setCurrentIndex(0);
            if (run_pack_select->currentIndex() == 0)
            {
                run_exec_select->setEnabled(false);
                return;
            }
            if (run_mode_select->currentIndex() == 0)
            {
                run_exec_select->addItems(get_exec_list(run_pack_select->currentText().toStdString()));
            }
            else
            {
                run_exec_select->addItems(get_launch_list(run_pack_select->currentText().toStdString()));
            }
            run_exec_select->setEnabled(true);
        }

        void change_exec_slot()
        {
            if (run_exec_select->currentIndex() == 0)
            {
                args_edit->setEnabled(false);
                args_edit->clear();
                run_button->setEnabled(false);
                return;
            }
            else
            {
                run_button->setEnabled(true);
                args_edit->setEnabled(true);
            }
        }

        void run_button_slot()
        {
            // get cmd
            std::string cmd;
            QString mode = run_mode_select->currentText();
            QString pack = run_pack_select->currentText();
            QString exec_info = run_exec_select->currentText();
            QString args = args_edit->text();
            cmd = mode.toStdString() + " ";
            cmd = cmd + pack.toStdString() + " ";
            cmd = cmd + exec_info.toStdString() + " ";
            cmd = cmd + args.toStdString() + " ";
            cmd = cmd + ">> /dev/null";

            // start thread
            int thread_id = ros_node_threads.size();
            std::thread run_node_thread(&ControllerRunNodeWindow::run_main_thread_func, this, cmd, thread_id);
            ros_node_threads.emplace_back(&run_node_thread);
            run_node_thread.detach();

            // update state
            ros_node_threads_run_state.emplace_back(true);
            ros_node_threads_terminal_flag.emplace_back(false);

            // add item and update table
            int thread_count = ros_node_threads.size() - 1;
            QStandardItem *item_1 = new QStandardItem();
            QStandardItem *item_2 = new QStandardItem();
            QStandardItem *item_3 = new QStandardItem();
            QStandardItem *item_4 = new QStandardItem();
            QStandardItem *item_5 = new QStandardItem();
            item_1->setEditable(false);
            item_2->setEditable(false);
            item_3->setEditable(false);
            item_4->setEditable(false);
            item_5->setEditable(false);
            item_1->setTextAlignment(Qt::AlignCenter);
            item_2->setTextAlignment(Qt::AlignCenter);
            item_3->setTextAlignment(Qt::AlignCenter);
            item_4->setTextAlignment(Qt::AlignCenter);
            item_5->setTextAlignment(Qt::AlignCenter);
            item_1->setText("Running");
            item_2->setText(mode);
            item_3->setText(pack);
            item_4->setText(exec_info);
            item_5->setText(args);
            model_info->setItem(thread_count, 0, item_1);
            model_info->setItem(thread_count, 1, item_2);
            model_info->setItem(thread_count, 2, item_3);
            model_info->setItem(thread_count, 3, item_4);
            model_info->setItem(thread_count, 4, item_5);
            state_items.emplace_back(item_1);
            model_info->setData(model_info->index(thread_count, 0), QBrush(QColor(0, 150, 0)), Qt::TextColorRole);
        }

        void run_main_thread_func(const std::string cmd, const int thread_id)
        {
            pid_t pid;
            int status;
            // create fork
            if ((pid = fork()) < 0)
            {
                // fork error
                ros_node_threads_run_state[thread_id] = false;
                emit update_table_signal();
                return;
            }
            else if (pid == 0)
            {
                // child process
                execl("/bin/sh", "sh", "-c", cmd.c_str(), (char *)0);
                exit(127);
            }
            else
            {
                // father process
                while (waitpid(pid, &status, WNOHANG) == 0)
                {
                    if (ros_node_threads_terminal_flag[thread_id])
                    {
                        kill(pid, SIGTERM);
                        waitpid(pid, &status, 0);
                        break;
                    }
                }
                ros_node_threads_run_state[thread_id] = false;
                emit update_table_signal();
            }
        }

        void update_table_slot()
        {
            for (size_t i = 0; i < ros_node_threads.size(); i++)
            {
                // thread joinable means thread terminate
                if (!ros_node_threads_run_state[i])
                {
                    state_items[i]->setText("Terminate");
                    model_info->setData(model_info->index(i, 0), QBrush(Qt::red), Qt::TextColorRole);
                }
                else // else thread is running
                {
                    state_items[i]->setText("Running");
                    model_info->setData(model_info->index(i, 0), QBrush(QColor(0, 150, 0)), Qt::TextColorRole);
                }
            }
        }

        void table_click_slot(QPoint pos)
        {
            // get thread id
            QModelIndex index = table_info->indexAt(pos);
            int thread_id = index.row();

            // create a menu
            QMenu *popMenu = new QMenu(this);
            popMenu->setStyleSheet("QMenu::item::selected{background-color: #0078D7; color: #FFFFFF;}");

            if (index.isValid())
            {
                // add QActions
                QAction *restart_action = new QAction();
                restart_action->setText(QString(QStringLiteral("Restart")));
                QAction *terminate_action = new QAction();
                terminate_action->setText(QString(QStringLiteral("Terminate")));

                if (!ros_node_threads_run_state[thread_id])
                {
                    restart_action->setEnabled(true);
                    terminate_action->setEnabled(false);
                }
                else
                {
                    restart_action->setEnabled(false);
                    terminate_action->setEnabled(true);
                }

                // add Action to Menu
                popMenu->addAction(restart_action);
                popMenu->addAction(terminate_action);

                // QAction bind slot function
                QObject::connect(restart_action, &QAction::triggered, this, &ControllerRunNodeWindow::restart_thread_slot);
                QObject::connect(terminate_action, &QAction::triggered, this, &ControllerRunNodeWindow::terminate_thread_slot);

                // show menu cursor
                popMenu->exec(QCursor::pos());
            }

            // free memory
            QList<QAction *> list = popMenu->actions();
            foreach (QAction *pAction, list)
                delete pAction;
            delete popMenu;
        }

        void restart_thread_slot()
        {
            // get thread id
            int thread_id = table_info->currentIndex().row();

            // get cmd
            std::string cmd;
            QString mode = model_info->data(model_info->index(thread_id, 1)).toString();
            QString pack = model_info->data(model_info->index(thread_id, 2)).toString();
            QString exec_info = model_info->data(model_info->index(thread_id, 3)).toString();
            QString args = model_info->data(model_info->index(thread_id, 4)).toString();
            cmd = mode.toStdString() + " ";
            cmd = cmd + pack.toStdString() + " ";
            cmd = cmd + exec_info.toStdString() + " ";
            cmd = cmd + args.toStdString() + " ";
            cmd = cmd + ">> /dev/null";

            // delete old thread
            ros_node_threads.erase(ros_node_threads.begin() + thread_id);

            // generate new thread
            ros_node_threads_terminal_flag[thread_id] = false;
            std::thread run_node_thread(&ControllerRunNodeWindow::run_main_thread_func, this, cmd, thread_id);
            ros_node_threads.insert(ros_node_threads.begin() + thread_id, &run_node_thread);
            run_node_thread.detach();
            ros_node_threads_run_state[thread_id] = true;

            // update table
            emit update_table_signal();
        }

        void terminate_thread_slot()
        {
            // get thread id
            int thread_id = table_info->currentIndex().row();

            // terminate thread
            std::terminate_handler(ros_node_threads[thread_id]->native_handle());
            ros_node_threads_run_state[thread_id] = false;
            ros_node_threads_terminal_flag[thread_id] = true;

            // update table
            emit update_table_signal();
        }

        // utility functions
        // get rospack executable file list
        QStringList get_exec_list(const std::string &ros_pack_name)
        {
            QStringList exec_name;
            std::filesystem::path exec_path("");
            std::filesystem::path tmp_path;
            std::string ros_pack_path = ros::package::getPath(ros_pack_name);
            if (ros_pack_path.find(ros_distro) == std::string::npos)
            {
                exec_path.assign(ros_pack_path);
                tmp_path.assign("../../devel/lib/" + ros_pack_name);
                exec_path = exec_path / tmp_path;
            }
            else
            {
                exec_path.assign(ros_distro);
                tmp_path.assign("lib/" + ros_pack_name);
                exec_path = exec_path / tmp_path;
            }
            if (!std::filesystem::exists(exec_path))
            {
                return exec_name;
            }
            for (auto &file : std::filesystem::directory_iterator(exec_path))
            {
                exec_name.append(QString::fromStdString(file.path().filename()));
            }
            return exec_name;
        }

        // get rospack launch files list
        QStringList get_launch_list(const std::string &ros_pack_name)
        {
            QStringList launch_name;
            std::filesystem::path launch_path("");
            std::filesystem::path tmp_path;
            std::string ros_pack_path = ros::package::getPath(ros_pack_name);
            launch_path.assign(ros_pack_path);
            tmp_path.assign("launch");
            launch_path = launch_path / tmp_path;
            if (!std::filesystem::exists(launch_path))
            {
                return launch_name;
            }
            for (auto &file : std::filesystem::directory_iterator(launch_path))
            {
                if (file.path().extension() == ".launch")
                {
                    launch_name.append(QString::fromStdString(file.path().filename()));
                }
            }
            return launch_name;
        }
};

#endif