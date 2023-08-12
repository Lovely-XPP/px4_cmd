from PyQt5 import QtWidgets, QtCore, QtGui

class info_window():
    def __init__(self, main_win: QtWidgets.QMainWindow) -> None:
        self.main_win = main_win
        pass

    # main ui
    def main_ui(self):
        vbox = QtWidgets.QVBoxLayout()
        hbox = QtWidgets.QHBoxLayout()
        programme_info = QtWidgets.QPushButton("About the Programme", self.win)
        topic_name_info = QtWidgets.QPushButton("About Topics", self.win)
        sensors_info = QtWidgets.QPushButton("About Sensors", self.win)
        programme_info.setMinimumSize(200, 40)
        topic_name_info.setMinimumSize(200, 40)
        sensors_info.setMinimumSize(200, 40)
        programme_info.setMinimumSize(200, 40)
        topic_name_info.setMinimumSize(200, 40)
        sensors_info.setMinimumSize(200, 40)
        # set trigger must be before layout
        programme_info.clicked.connect(self.programme_info)
        topic_name_info.clicked.connect(self.topics_info)
        sensors_info.clicked.connect(self.sensors_info)

        # set layout
        vbox.addWidget(programme_info)
        vbox.addWidget(topic_name_info)
        vbox.addWidget(sensors_info)
        hbox.addLayout(vbox)

        # show text
        txt_show = QtWidgets.QTextBrowser(self.win)
        txt_show.setStyleSheet("background-color: white")
        txt_show.setText("Welcome.")
        hbox.addWidget(txt_show)
        self.txt = txt_show

        # set layout
        self.win.setLayout(hbox)


    # show programme info
    def programme_info(self):
        self.txt.setText("This is programme information.")


    # show topic info
    def topics_info(self):
        self.txt.setText("This is topic information.")


    # show sensors info
    def sensors_info(self):
        self.txt.setText("This is sensors information.")


    # main
    def setup(self) -> None:
        win = QtWidgets.QDialog(self.main_win)
        win.setWindowTitle("Information")
        win.setStyleSheet("background-color: rgb(255,250,250)")
        self.win = win
        self.main_ui()
        self.win.setFixedSize(600,600)
        self.win.show()