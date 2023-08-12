import sys, os, copy
import json
from PyQt5 import QtWidgets, QtCore, QtGui

PI = 3.1415926535897932384626433

class sensors_setting_window():
    # initial
    def __init__(self, main_win: QtWidgets.QMainWindow) -> None:
        self.main_win = main_win
        self.sensors_to_names = {
            "--- Select Sensor Type ---": "",
            "Lidar": "lidar",
            "Depth Camera": "dep_camera",
            "RGB Camera": "fpv_camera",
            "Stereo Camera": "stereo_camera",
            "Realsense Camera": "realsense_camera"
        }
        self.sensors = list(self.sensors_to_names.keys())
        # lidar initial value
        lidar = {}
        lidar['Position'] = [0.0, 0.0, 0.0]
        lidar['Pose'] = [0.0, 0.0, 0.0]
        lidar['Min Angle'] = -0.0001
        lidar['Max Angle'] = 0.0
        lidar['Resolution'] = 1.0
        lidar['Samples'] = 1
        lidar['Update Rate'] = 1
        # Camera initial value
        camera = {}
        camera['Position'] = [0.0, 0.0, 0.0]
        camera['Pose'] = [0.0, 0.0, 0.0]
        camera['Width'] = 640
        camera['Height'] = 480
        camera['Update Rate'] = 1
        # stereo camera add a distance parameter
        stereo_cam = copy.deepcopy(camera)
        stereo_cam['Distance'] = 0.0
        self.sensors_data = {
            "Lidar": lidar,
            "Depth Camera": camera,
            "RGB Camera": camera,
            "Stereo Camera": stereo_cam,
            "Realsense Camera": camera
        }
        # save data inforamtion
        self.save_sensors_data = self.sensors_data
        # set textedit widgets max height
        self.textedit_max_height = 30
        pass

    
    # main ui 
    def main_ui(self) -> None:
        # create layout
        vbox = QtWidgets.QVBoxLayout()
        hbox = QtWidgets.QHBoxLayout()
        hbox.setSpacing(10)
        hbox.setAlignment(QtCore.Qt.AlignHCenter)
        vbox.setSpacing(36)
        vbox.setAlignment(QtCore.Qt.AlignTop)
        vbox.setContentsMargins(50, 35, 50, 35)

        # select box
        lable1 = QtWidgets.QLabel("Sensors Type", self.win)
        list1 = QtWidgets.QComboBox(self.win)
        list1.setMinimumWidth(400)
        list1.addItems(self.sensors)
        list1.setStyleSheet("background-color: rgb(155,205,155)")
        hbox.addWidget(lable1)
        hbox.addWidget(list1)
        vbox.addLayout(hbox)
        # save box
        save_box = QtWidgets.QHBoxLayout()
        save_box.setAlignment(QtCore.Qt.AlignHCenter)
        export_button = QtWidgets.QPushButton("Export", self.win)
        load_button = QtWidgets.QPushButton("Load", self.win)
        load_button.setStyleSheet("background-color: rgb(159,231,167); font-weight: bold; font-size: 16pt")
        load_button.setMinimumSize(150, 40)
        export_button.setStyleSheet("background-color: rgb(176,208,238); font-weight: bold; font-size: 16pt")
        export_button.setMinimumSize(150, 40)
        save_button = QtWidgets.QPushButton("Save", self.win)
        save_button.setStyleSheet("background-color: rgb(180,180,241); font-weight: bold; font-size: 16pt")
        save_button.setMinimumSize(150, 40)
        save_box.addWidget(load_button)
        save_box.addWidget(save_button)
        save_box.addStretch(1)
        save_box.addWidget(export_button)
        vbox.addStretch(7)
        vbox.addLayout(save_box)
        # set trigger
        save_button.clicked.connect(self.save)
        export_button.clicked.connect(self.export)
        load_button.clicked.connect(self.load)

        # set layout
        list1.currentTextChanged.connect(self.update_ui)
        self.win.setLayout(vbox)
        self.list1 = list1
        self.vbox = vbox
        

    # update_ui
    def update_ui(self) -> None:
        select_text = self.list1.currentText()
        self.clear_layout(self.win.layout(), last_index=1)
        
        if select_text != self.sensors[0]:
            sensor_data = self.sensors_data[select_text]
            # Posisiton Parameters Box
            position_box = QtWidgets.QGroupBox("[Position (m)]")
            position_vbox = QtWidgets.QVBoxLayout()
            position_vbox.setSpacing(10)
            posistion_lable_1 = QtWidgets.QLabel("x", self.win)
            posistion_txt_1 = QtWidgets.QLineEdit(f"{sensor_data['Position'][0]}", self.win)
            posistion_lable_2 = QtWidgets.QLabel("y", self.win)
            posistion_txt_2 = QtWidgets.QLineEdit(f"{sensor_data['Position'][1]}", self.win)
            posistion_lable_3 = QtWidgets.QLabel("z", self.win)
            posistion_txt_3 = QtWidgets.QLineEdit(f"{sensor_data['Position'][2]}", self.win)
            posistion_txt_1.setMaximumHeight(self.textedit_max_height)
            posistion_txt_2.setMaximumHeight(self.textedit_max_height)
            posistion_txt_3.setMaximumHeight(self.textedit_max_height)
            posistion_txt_1.setStyleSheet("background-color: white")
            posistion_txt_2.setStyleSheet("background-color: white")
            posistion_txt_3.setStyleSheet("background-color: white")
            posistion_txt_1.textChanged.connect(self.update_data)
            posistion_txt_2.textChanged.connect(self.update_data)
            posistion_txt_3.textChanged.connect(self.update_data)
            self.posistion_txt = [posistion_txt_1, posistion_txt_2, posistion_txt_3]
            pos1_hbox = QtWidgets.QHBoxLayout()
            pos2_hbox = QtWidgets.QHBoxLayout()
            pos3_hbox = QtWidgets.QHBoxLayout()
            pos1_hbox.addWidget(posistion_lable_1)
            pos1_hbox.addWidget(posistion_txt_1)
            pos2_hbox.addWidget(posistion_lable_2)
            pos2_hbox.addWidget(posistion_txt_2)
            pos3_hbox.addWidget(posistion_lable_3)
            pos3_hbox.addWidget(posistion_txt_3)
            position_vbox.addLayout(pos1_hbox)
            position_vbox.addLayout(pos2_hbox)
            position_vbox.addLayout(pos3_hbox)
            position_box.setLayout(position_vbox)

            # Pose Parameters Box
            pose_box =QtWidgets.QGroupBox("[Pose (deg)]")
            pose_vbox = QtWidgets.QVBoxLayout()
            pose_vbox.setSpacing(10)
            pose_lable_1 = QtWidgets.QLabel("R", self.win)
            pose_txt_1 = QtWidgets.QLineEdit(f"{sensor_data['Pose'][0]}", self.win)
            pose_lable_2 = QtWidgets.QLabel("P", self.win)
            pose_txt_2 = QtWidgets.QLineEdit(f"{sensor_data['Pose'][1]}", self.win)
            pose_lable_3 = QtWidgets.QLabel("Y", self.win)
            pose_txt_3 = QtWidgets.QLineEdit(f"{sensor_data['Pose'][2]}", self.win)
            pose_txt_1.setMaximumHeight(self.textedit_max_height)
            pose_txt_2.setMaximumHeight(self.textedit_max_height)
            pose_txt_3.setMaximumHeight(self.textedit_max_height)
            pose_txt_1.setStyleSheet("background-color: white")
            pose_txt_2.setStyleSheet("background-color: white")
            pose_txt_3.setStyleSheet("background-color: white")
            pose_txt_1.textChanged.connect(self.update_data)
            pose_txt_2.textChanged.connect(self.update_data)
            pose_txt_3.textChanged.connect(self.update_data)
            self.pose_txt = [pose_txt_1, pose_txt_2, pose_txt_3]
            pose1_hbox = QtWidgets.QHBoxLayout()
            pose2_hbox = QtWidgets.QHBoxLayout()
            pose3_hbox = QtWidgets.QHBoxLayout()
            pose1_hbox.addWidget(pose_lable_1)
            pose1_hbox.addWidget(pose_txt_1)
            pose2_hbox.addWidget(pose_lable_2)
            pose2_hbox.addWidget(pose_txt_2)
            pose3_hbox.addWidget(pose_lable_3)
            pose3_hbox.addWidget(pose_txt_3)
            pose_vbox.addLayout(pose1_hbox)
            pose_vbox.addLayout(pose2_hbox)
            pose_vbox.addLayout(pose3_hbox)
            pose_box.setLayout(pose_vbox)

            # combine two boxes
            position_pose_box = QtWidgets.QHBoxLayout()
            position_pose_box.addWidget(position_box)
            position_pose_box.addWidget(pose_box)
            # add layout to vbox
            self.vbox.addLayout(position_pose_box)

            # lidar
            if select_text == self.sensors[1]:
                res0_box = QtWidgets.QHBoxLayout()
                res_box = QtWidgets.QHBoxLayout()
                lidar_lable_samples = QtWidgets.QLabel("[Samples]")
                lidar_txt_samples = QtWidgets.QLineEdit(f"{sensor_data['Samples']}", self.win)
                lidar_lable_resolution = QtWidgets.QLabel("[Resolution]")
                lidar_txt_resolution = QtWidgets.QLineEdit(f"{sensor_data['Resolution']}", self.win)
                lidar_lable_min_angle = QtWidgets.QLabel("[Min Angle]")
                lidar_txt_min_angle = QtWidgets.QLineEdit(f"{sensor_data['Min Angle']}", self.win)
                lidar_lable_max_angle = QtWidgets.QLabel("[Max Angle]")
                lidar_txt_max_angle = QtWidgets.QLineEdit(f"{sensor_data['Max Angle']}", self.win)
                lidar_txt_samples.setMaximumHeight(self.textedit_max_height)
                lidar_txt_resolution.setMaximumHeight(self.textedit_max_height)
                lidar_txt_min_angle.setMaximumHeight(self.textedit_max_height)
                lidar_txt_max_angle.setMaximumHeight(self.textedit_max_height)
                lidar_txt_samples.setStyleSheet("background-color: white")
                lidar_txt_resolution.setStyleSheet("background-color: white")
                lidar_txt_min_angle.setStyleSheet("background-color: white")
                lidar_txt_max_angle.setStyleSheet("background-color: white")
                lidar_txt_samples.textChanged.connect(self.update_data)
                lidar_txt_resolution.textChanged.connect(self.update_data)
                lidar_txt_min_angle.textChanged.connect(self.update_data)
                lidar_txt_max_angle.textChanged.connect(self.update_data)
                self.lidar_txt = {
                    "Samples": lidar_txt_samples,
                    "Resolution": lidar_txt_resolution,
                    "Min Angle": lidar_txt_min_angle,
                    "Max Angle": lidar_txt_max_angle
                }
                res0_box.addWidget(lidar_lable_samples)
                res0_box.addWidget(lidar_txt_samples)
                res0_box.addWidget(lidar_lable_resolution)
                res0_box.addWidget(lidar_txt_resolution)
                self.vbox.addLayout(res0_box)
                res_box.addWidget(lidar_lable_min_angle)
                res_box.addWidget(lidar_txt_min_angle)
                res_box.addWidget(lidar_lable_max_angle)
                res_box.addWidget(lidar_txt_max_angle)
            # cameras
            else:
                res_box = QtWidgets.QHBoxLayout()
                lable4 = QtWidgets.QLabel("[Resolution]")
                cam_lable_w = QtWidgets.QLabel("Width", self.win)
                cam_txt_w = QtWidgets.QLineEdit(f"{sensor_data['Width']}", self.win)
                cam_lable_h = QtWidgets.QLabel("Height", self.win)
                cam_txt_h = QtWidgets.QLineEdit(f"{sensor_data['Height']}", self.win)
                cam_txt_w.setMaximumHeight(self.textedit_max_height)
                cam_txt_h.setMaximumHeight(self.textedit_max_height)
                cam_txt_w.setStyleSheet("background-color: white")
                cam_txt_h.setStyleSheet("background-color: white")
                cam_txt_w.textChanged.connect(self.update_data)
                cam_txt_h.textChanged.connect(self.update_data)
                self.cam_txt = {
                    "Width": cam_txt_w,
                    "Height": cam_txt_h
                }
                res_box.addWidget(lable4)
                res_box.addWidget(cam_lable_w)
                res_box.addWidget(cam_txt_w)
                res_box.addWidget(cam_lable_h)
                res_box.addWidget(cam_txt_h)
                if select_text == "Stereo Camera":
                    dis_box = QtWidgets.QHBoxLayout()
                    lable_stereo = QtWidgets.QLabel("[Distance (m)]", self.win)
                    text_stereo = QtWidgets.QLineEdit(f"{sensor_data['Distance']}", self.win)
                    text_stereo.setMaximumHeight(self.textedit_max_height)
                    text_stereo.setStyleSheet("background-color: white")
                    text_stereo.textChanged.connect(self.update_data)
                    self.cam_stereo_txt = text_stereo
                    self.cam_txt['Distance'] = text_stereo
                    dis_box.addWidget(lable_stereo)
                    dis_box.addWidget(text_stereo)
            # add parameter box
            self.vbox.addLayout(res_box)
            if select_text == "Stereo Camera":
                self.vbox.addLayout(dis_box)
            # update rate box
            update_box = QtWidgets.QHBoxLayout()
            label_update = QtWidgets.QLabel("[Update Rate]", self.win)
            text_update = QtWidgets.QLineEdit(f"{sensor_data['Update Rate']}", self.win)
            text_update.setMaximumHeight(self.textedit_max_height)
            text_update.setStyleSheet("background-color: white")
            text_update.textChanged.connect(self.update_data)
            self.update_rate_txt = text_update
            update_box.addWidget(label_update)
            update_box.addWidget(text_update)
            self.vbox.addLayout(update_box)
            if select_text == "Stereo Camera":
                self.vbox.addStretch(3)
            else:
                self.vbox.addStretch(4)
        else:
            self.vbox.addStretch(7)
        # save box
        save_box = QtWidgets.QHBoxLayout()
        save_box.setAlignment(QtCore.Qt.AlignHCenter)
        export_button = QtWidgets.QPushButton("Export", self.win)
        load_button = QtWidgets.QPushButton("Load", self.win)
        load_button.setStyleSheet("background-color: rgb(50,191,255); font-weight: bold; font-size: 16pt")
        load_button.setMinimumSize(150, 40)
        export_button.setStyleSheet("background-color: rgb(84,255,159); font-weight: bold; font-size: 16pt")
        export_button.setMinimumSize(150, 40)
        save_button = QtWidgets.QPushButton("Save", self.win)
        save_button.setStyleSheet("background-color: rgb(84,255,159); font-weight: bold; font-size: 16pt")
        save_button.setMinimumSize(150, 40)
        save_box.addWidget(load_button)
        save_box.addWidget(save_button)
        save_box.addStretch(1)
        save_box.addWidget(export_button)
        self.vbox.addLayout(save_box)
        # set trigger
        save_button.clicked.connect(self.save)
        export_button.clicked.connect(self.export)
        load_button.clicked.connect(self.load)


    # check input format, data, data_format: 0 - float 1 - int
    def check_data(self, data: str, data_format: int = 0, error_msg: str = "") -> int or float or None:
        if data == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', f'Parameter [{error_msg}] Can not Be Empty!')
            msg_box.exec_()
            return ""
        if data_format:
            try:
                return int(data)
            except:
                msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', f'Parameter [{error_msg}] Only Support Int Type!')
                msg_box.exec_()
                return None
        try:
            return float(data)
        except:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', f'Parameter [{error_msg}] Only Support Float Type!')
            msg_box.exec_()
            return None


    # get edited data
    def update_data(self) -> None:
        select_text = self.list1.currentText()
        if select_text == self.sensors[0]:
            return
        position = []
        pose = []
        lidar = {}
        camera = {}
        # get position data
        for i, txt in enumerate(self.posistion_txt):
            msgs = 'xyz'
            data = txt.text()
            data = self.check_data(data, data_format=0, error_msg=msgs[i])
            if data == None:
                return
            position.append(data)
        # get pose data
        for i, txt in enumerate(self.pose_txt):
            msgs = 'RPY'
            data = txt.text()
            data = self.check_data(data, data_format=0, error_msg=msgs[i])
            if data == None:
                return
            pose.append(data)
        # get lidar data
        if select_text == self.sensors[1]:
            lidar['Position'] = position
            lidar['Pose'] = pose
            for key in self.lidar_txt.keys():
                data = self.lidar_txt[key].text()
                if key == "Samples":
                    data = self.check_data(data, data_format=1, error_msg=key)
                else:
                    data = self.check_data(data, data_format=0, error_msg=key)
                if data == None:
                    return
                lidar[key] = data
            # update rate
            data = self.update_rate_txt.text()
            data = self.check_data(data, data_format=1, error_msg='Update Rate')
            if data == None:
                return
            lidar['Update Rate'] = data
            # update data
            self.sensors_data[select_text] = lidar
        # get camera data
        else:
            camera['Position'] = position
            camera['Pose'] = pose
            for key in self.cam_txt.keys():
                data = self.cam_txt[key].text()
                data = self.check_data(data, data_format=1, error_msg=key)
                if data == None:
                    return
                camera[key] = data
            # stereo camera
            if select_text == "Stereo Camera":
                data = self.cam_stereo_txt.text()
                data = self.check_data(data, data_format=1, error_msg='Distance')
                if data == None:
                    return
                camera["Distance"] = data
            # update rate
            data = self.update_rate_txt.text()
            data = self.check_data(data, data_format=1, error_msg='Update Rate')
            if data == None:
                return
            camera['Update Rate'] = data
            self.sensors_data[select_text] = camera
        
            
    # clear widgets
    def clear_layout(self, layout: QtWidgets.QLayout, last_index: int = 0, first_index: int = 0) -> None:
        item_list = list(range(layout.count()))
        origin_first_index = len(item_list) - 1
        item_list.reverse()# 倒序删除，避免影响布局顺序
        for i in item_list:
            if i > (origin_first_index - first_index):
                continue
            if i < last_index:
                return
            item = layout.itemAt(i)
            layout.removeItem(item)
            if item.layout():
                self.clear_layout(item.layout())
            if item.widget():
                item.widget().deleteLater()


    # load
    def load(self):
        msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Information, 'Info', 'Load Sensors Configuration Files Will Replace Origin Configuration.')
        msg_box.exec_()
        filename, file_type = QtWidgets.QFileDialog.getOpenFileName(None, "Select Saved Sensors Config File Dir", os.getcwd(), "Json Files (*.json)")
        if not os.path.exists(filename):
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Cannot find the Sensors Configuration Json File.')
            msg_box.exec_()
            return
        filename = filename.split('.')[0]
        filename = filename + ".json"
        with open(filename, 'r') as f:
            sensors_data = json.load(f)
        msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'The Sensors Configuration Json File is Bad, please check the file.')
        for key1 in sensors_data:
            if key1 not in self.sensors_data.keys():
                msg_box.exec_()
                return
            for key in sensors_data[key1]:
                if key not in self.sensors_data[key1]:
                    msg_box.exec_()
                    return
                if key == "Position" or key == "Pose":
                    if len(sensors_data[key1][key]) != 3:
                        msg_box.exec_()
                        return
                    for item in sensors_data[key1][key]:
                        if type(item) != float:
                            msg_box.exec_()
                            return
                    continue
                if key == "Width" or key == "Height" or key == "Samples" or key == "Update Rate":
                    if type(sensors_data[key1][key]) != int:
                        msg_box.exec_()
                        return
                else:
                    if type(sensors_data[key1][key]) != float and type(sensors_data[key1][key]) != int:
                        msg_box.exec_()
                        return
        msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Information, 'Info', 'The Sensors Configuration Json File is Successfully Loaded.')
        msg_box.exec_()
        self.save_sensors_data = self.sensors_data


    # save 
    def export(self):
        self.save_sensors_data = self.sensors_data
        filename, file_type = QtWidgets.QFileDialog.getSaveFileName(None, "Select Saved Sensors Config File Dir", os.getcwd(), "Json Files (*.json)")
        if filename == "":
            return
        filename = filename.split('.')[0]
        filename = filename + ".json"
        with open(filename, 'w') as f:
            json.dump(self.save_sensors_data, f, indent='\t')
        msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Information, 'Info', f'fThe Sensors Configuration Json File is Successfully Saved at {filename}.')
        msg_box.exec_()
    

    # save and exit
    def save(self):
        self.save_sensors_data = self.sensors_data
        msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Information, 'Info', 'fThe Sensors Configuration is Successfully Saved.')
        msg_box.exec_()


    # main
    def setup(self) -> None:
        win = QtWidgets.QDialog(self.main_win)
        win.setWindowTitle("Sensors Setting")
        win.setStyleSheet("background-color: rgb(255,250,250)")
        self.win = win
        self.main_ui()
        self.win.setFixedSize(600,600)
        self.win.show()