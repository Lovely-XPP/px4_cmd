import sys, os
import xml.etree.ElementTree as ET
from PyQt5 import QtWidgets, QtCore, QtGui
from sensors_setting import sensors_setting_window


class launch_generator():
    # initial
    def __init__(self) -> None:
        self.vehicles = []
        self.sensors = []
        self.init_pos = []
        self.output_file = ""
        self.topic_name = {"{Vehicle Type}_{ID}": 0, "uav_{ID}": 1}
        self.local_port = 34580
        self.remote_port = 24540
        self.sitl_port = 18570
        self.tcp_port = 4560
        self.types = ["iris", "typhoon_h480", "plane"]
        self.sensors_to_names = {
            "None": "",
            "Lidar": "_lidar",
            "Depth Camera": "_depth_camera",
            "RGB Camera": "_fpv_camera",
            "Stereo Camera": "_stereo_camera",
            "Realsense Camera": "_realsense_camera"
        }
        pass
        

    # select type
    def add_list(self) -> None:
        xshift = -20
        # lable
        lable1 = QtWidgets.QLabel("Vehicle Type", self.main_win)
        lable1.move(50+xshift, 20)
        lable1.resize(150, 35)
        # select
        list1 = QtWidgets.QComboBox(self.main_win)
        list1.addItems(self.get_type())
        list1.move(150+xshift, 20)
        list1.resize(200, 35)
        list1.setStyleSheet("background-color: rgb(135,206,235)")
        self.type_list = list1

        # select sensor
        # lable
        lable2 = QtWidgets.QLabel("Sensor Type", self.main_win)
        lable2.move(400+xshift, 20)
        lable2.resize(150, 35)
        # select
        list2 = QtWidgets.QComboBox(self.main_win)
        list2.addItems(["None", "Camera", "Downward Camera", "Realsense", "2D-Lidar", "3D-Lidar"])
        list2.move(500+xshift, 20)
        list2.resize(200, 35)
        list2.setStyleSheet("background-color: rgb(155,205,155)")
        self.sensor_list = list2
        self.type_list.currentIndexChanged.connect(self.association)

        # select world
        # lable
        lable3 = QtWidgets.QLabel("World File", self.main_win)
        lable3.move(50+xshift, 70)
        lable3.resize(150, 35)
        # select
        list3 = QtWidgets.QComboBox(self.main_win)
        list3.addItems(self.get_worldfile())
        list3.move(150+xshift, 70)
        list3.resize(200, 35)
        list3.setStyleSheet("background-color: rgb(135,206,235)")
        self.world_list = list3

        # topic name
        # lable
        lable4 = QtWidgets.QLabel("Topic Name", self.main_win)
        lable4.move(400+xshift, 70)
        lable4.resize(150, 35)
        # select
        list4 = QtWidgets.QComboBox(self.main_win)
        list4.addItems(list(self.topic_name.keys()))
        list4.move(500+xshift, 70)
        list4.resize(200, 35)
        list4.setStyleSheet("background-color: rgb(155,205,155)")
        self.name_list = list4


    # Add table
    def add_table(self) -> None:
        xshift = -20
        table = QtWidgets.QTableView(self.main_win)
        table.move(50+xshift, 280)
        table.resize(1020, 400)
        table.setStyleSheet("background-color: white")
        self.table = table
        headers = ["Vehicle", "Sensor", "x", "y", "z", "R", "P", "Y"]
        model = QtGui.QStandardItemModel(0, len(headers))
        self.model = model
        model.setHorizontalHeaderLabels(headers)
        self.table.setModel(model)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.model.itemChanged.connect(self.update_edit_table)


    # get_types
    def get_type(self) -> list:
        return self.types
    

    # get_type_sensors
    def get_sensor(self, type) -> list:
        if "iris" in type:
            sensors = ["None", "Camera", "Downward Camera", "Realsense", "2D-Lidar", "3D-Lidar"]
        if "typhoon_h480" in type:
            sensors = ["None", "Camera", "Realsense", "2D-Lidar"]
        if "plane" in type:
            sensors = ["None", "Camera"]
        return sensors
    

    # input initial position
    def input_init_pos(self) -> None:
        yshift = 10 + 50
        lable00 = QtWidgets.QFrame(self.main_win)
        op = QtWidgets.QGraphicsOpacityEffect()
        op.setOpacity(0.3)
        lable00.setFrameShape(QtWidgets.QFrame.Box)
        lable00.setGraphicsEffect(op)
        lable00.move(30, 63+yshift)
        lable00.resize(800,75)
        lable0 = QtWidgets.QLabel("Initial Position", self.main_win)
        lable0.move(400, 55+yshift)
        lable0.resize(100, 35)
        lable0.setAlignment(QtCore.Qt.AlignHCenter)

        x0 = 50
        dx1 = 20
        dx2 = 15
        x1 = 95
        x = x0
        # x
        # lable
        lable1 = QtWidgets.QLabel("x", self.main_win)
        lable1.move(x, 85+yshift)
        lable1.resize(30, 35)
        # text
        text1 = QtWidgets.QLineEdit(self.main_win)
        text1.setReadOnly(False)
        x = x + dx2
        text1.move(x, 85+yshift)
        text1.resize(x1, 35)
        text1.setStyleSheet("background-color: white")
        self.text_x = text1

        # y
        # lable
        lable2 = QtWidgets.QLabel("y", self.main_win)
        x = x + x1 + dx1
        lable2.move(x, 85+yshift)
        lable2.resize(30, 35)
        # text
        text2 = QtWidgets.QLineEdit(self.main_win)
        text2.setReadOnly(False)
        x = x + dx2
        text2.move(x, 85+yshift)
        text2.resize(x1, 35)
        text2.setStyleSheet("background-color: white")
        self.text_y = text2

        # z
        # lable
        lable3 = QtWidgets.QLabel("z", self.main_win)
        x = x + x1 + dx1
        lable3.move(x, 85+yshift)
        lable3.resize(30, 35)
        # text
        text3 = QtWidgets.QLineEdit(self.main_win)
        text3.setReadOnly(False)
        x = x + dx2
        text3.move(x, 85+yshift)
        text3.resize(x1, 35)
        text3.setStyleSheet("background-color: white")
        self.text_z = text3

        # R
        # lable
        lable4 = QtWidgets.QLabel("R", self.main_win)
        x = x + x1 + dx1
        lable4.move(x, 85+yshift)
        lable4.resize(30, 35)
        # text
        text4 = QtWidgets.QLineEdit(self.main_win)
        text4.setReadOnly(False)
        x = x + dx2
        text4.move(x, 85+yshift)
        text4.resize(x1, 35)
        text4.setStyleSheet("background-color: white")
        self.text_R = text4

        # P
        # lable
        lable5 = QtWidgets.QLabel("P", self.main_win)
        x = x + x1 + dx1
        lable5.move(x, 85+yshift)
        lable5.resize(30, 35)
        # text
        text5 = QtWidgets.QLineEdit(self.main_win)
        text5.setReadOnly(False)
        x = x + dx2
        text5.move(x, 85+yshift)
        text5.resize(x1, 35)
        text5.setStyleSheet("background-color: white")
        self.text_P = text5

        # Y
        # lable
        lable6 = QtWidgets.QLabel("Y", self.main_win)
        x = x + x1 + dx1
        lable6.move(x, 85+yshift)
        lable6.resize(30, 35)
        # text
        text6 = QtWidgets.QLineEdit(self.main_win)
        text6.setReadOnly(False)
        x = x + dx2
        text6.move(x, 85+yshift)
        text6.resize(x1, 35)
        text6.setStyleSheet("background-color: white")
        self.text_Y = text6


    # select dir
    def choose_dir(self) -> None:
        xshift = -20
        yshift = 30 + 45
        # lable
        lable = QtWidgets.QLabel("Output Dir", self.main_win)
        lable.move(50+xshift, 140+yshift)
        lable.resize(100, 35)

        # show dir
        text = QtWidgets.QLineEdit(self.main_win)
        text.setReadOnly(True)
        text.move(155+xshift, 140+yshift)
        text.resize(580, 35)
        text.setStyleSheet("background-color: white")
        self.text = text

        # browse button
        button = QtWidgets.QPushButton("Browse", self.main_win)
        button.move(750+xshift, 140+yshift)
        button.resize(100, 35)
        button.setStyleSheet("background-color: rgb(255,235,230)")
        self.browse_button = button
        self.browse_button.clicked.connect(self.browse_dir)
    

    # browse dir
    def browse_dir(self) -> None:
        filename, file_type = QtWidgets.QFileDialog.getSaveFileName(None, "Select Saved Launch File Dir", os.getcwd(), "Launch Files (*.launch)")
        if filename == "":
            self.text.setText("")
            self.output_file = ""
            return
        filename = filename.split('.')[0]
        filename = filename + ".launch"
        self.text.setText(filename)
        self.output_file = filename


    # Associate sensor with type
    def association(self) -> None:
        type = self.type_list.currentText()
        self.sensor_list.clear()
        self.sensor_list.addItems(self.get_sensor(type))


    # Add buttons
    def add_buttons(self) -> None:
        xshfit = 70
        # add vehicle button
        button1 = QtWidgets.QPushButton("Add Vehicle", self.main_win)
        button1.move(780+xshfit, 20)
        button1.resize(200, 45)
        button1.setStyleSheet("background-color: rgb(50,191,255); font-size: 13pt")
        # del vehicle button
        button2 = QtWidgets.QPushButton("Del Vehicle", self.main_win)
        button2.move(780+xshfit, 75)
        button2.resize(200, 45)
        button2.setStyleSheet("background-color: rgb(255,106,106); font-size: 13pt")
        # clear vehicle button
        button3 = QtWidgets.QPushButton("Clear Vehicles", self.main_win)
        button3.move(780+xshfit, 130)
        button3.resize(200, 45)
        button3.setStyleSheet("background-color: rgb(224,102,255); font-size: 13pt")
        # generate button
        button4 = QtWidgets.QPushButton("Generate Launch", self.main_win)
        button4.move(780+xshfit, 195)
        button4.resize(200, 65)
        button4.setStyleSheet("background-color: rgb(84,255,159); font-weight: bold; font-size: 16pt")
        # sensors setting
        button5 = QtWidgets.QPushButton("Sensors\nSetting", self.main_win)
        button5.move(700, 20)
        button5.resize(130, 85)
        button5.setStyleSheet("background-color: rgb(255,250,205); font-size: 13pt")
        # set trigger
        self.add_button = button1
        self.del_button = button2
        self.clc_button = button3
        self.generate_button = button4
        self.sensors_set_button = button5
        self.add_button.clicked.connect(self.add_vehicle)
        self.del_button.clicked.connect(self.del_vehicle)
        self.clc_button.clicked.connect(self.clc_vehicle)
        self.generate_button.clicked.connect(self.generate_launch)
        self.sensors_set_button.clicked.connect(self.sensors_set)
        

    # Update table
    def update_table(self) -> None:
        numbers = len(self.vehicles)
        headers = ["Vehicle", "Sensor", "x", "y", "z", "R", "P", "Y"]
        model = QtGui.QStandardItemModel(numbers, len(headers))
        model.setHorizontalHeaderLabels(headers)
        for row in range(numbers):
            item1 = QtGui.QStandardItem(self.vehicles[row])
            item2 = QtGui.QStandardItem(self.sensors[row])
            item3 = QtGui.QStandardItem(self.init_pos[row]['x'])
            item4 = QtGui.QStandardItem(self.init_pos[row]['y'])
            item5 = QtGui.QStandardItem(self.init_pos[row]['z'])
            item6 = QtGui.QStandardItem(self.init_pos[row]['R'])
            item7 = QtGui.QStandardItem(self.init_pos[row]['P'])
            item8 = QtGui.QStandardItem(self.init_pos[row]['Y'])
            item1.setEditable(False)
            item2.setEditable(False)
            model.setItem(row, 0, item1)
            model.setItem(row, 1, item2)
            model.setItem(row, 2, item3)
            model.setItem(row, 3, item4)
            model.setItem(row, 4, item5)
            model.setItem(row, 5, item6)
            model.setItem(row, 6, item7)
            model.setItem(row, 7, item8)
        self.model = model
        self.table.setModel(model)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.model.itemChanged.connect(self.update_edit_table)


    # Update Edit table
    def update_edit_table(self) -> None:
        row = self.table.currentIndex().row()
        column = self.table.currentIndex().column()
        data = self.table.currentIndex().data()
        data = self.check_input_data(data)
        if data == "":
            self.update_table()
            return
        init_pos_str = "xyzRPY"
        self.init_pos[row][init_pos_str[column-2]] = data
        self.update_table()


    # check input data
    def check_input_data(self, txt) -> str:
        if txt == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Initial Postion Can not Be Empty!')
            msg_box.exec_()
            return ""
        try:
            return str(float(txt))
        except:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Input Initial Postion Only Support Float Type!')
            msg_box.exec_()
            return ""


    # Add vehicle button signal handle
    def add_vehicle(self) -> None:
        init_pos = {}
        x = self.check_input_data(self.text_x.text())
        if x == "":
            return
        init_pos['x'] = x
        y = self.check_input_data(self.text_y.text())
        if y == "":
            return
        init_pos['y'] = y
        z = self.check_input_data(self.text_z.text())
        if z == "":
            return
        init_pos['z'] = z
        P = self.check_input_data(self.text_P.text())
        if P == "":
            return
        init_pos['P'] = P
        R = self.check_input_data(self.text_R.text())
        if R == "":
            return
        init_pos['R'] = R
        Y = self.check_input_data(self.text_Y.text())
        if Y == "":
            return
        init_pos['Y'] = Y
        self.init_pos.append(init_pos)
        self.vehicles.append(self.type_list.currentText())
        self.sensors.append(self.sensor_list.currentText())
        self.update_table()


    # Del vehicle button signal handle
    def del_vehicle(self) -> None:
        indexes = self.table.selectionModel().selectedIndexes()
        if len(indexes) == 0:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Information, 'Info', 'Please Select Vehicle First!')
            msg_box.exec_()
            return
        rows = []
        for index in indexes:
            rows.append(index.row())
        rows.sort()
        for num, row in enumerate(rows):
            row = row - num
            self.vehicles.pop(row)
            self.sensors.pop(row)
            self.init_pos.pop(row)
        self.update_table()


    # clear vehicle button signal handle
    def clc_vehicle(self) -> None:
        self.vehicles = []
        self.sensors = []
        self.init_pos = []
        self.update_table()

    
    # get world file
    def get_worldfile(self) -> list:
        file_dir = "echo $(rospack find mavlink_sitl_gazebo)/worlds/"
        file_dir = os.popen(file_dir)
        file_dir = file_dir.read().strip('\n').strip()
        files = os.listdir(file_dir)
        files.sort()
        return files


    # open sensors setting window
    def sensors_set(self) -> None:
        self.sensors_setting_win = sensors_setting_window(self.main_win)
        self.sensors_setting_win.setup()


    # generate launch
    def generate_launch(self) -> None:
        if len(self.vehicles) == 0:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Add Vehicle First!')
            msg_box.exec_()
            return
        if self.output_file == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Browse Output Dir!')
            msg_box.exec_()
            return
        # init new xml
        launch = ET.Element('launch')
        # arg config
        worldfile = self.world_list.currentText()
        ET.SubElement(launch, 'arg', attrib={"name": "est", "default": "ekf2"})
        ET.SubElement(launch, 'arg', attrib={"name": "world", "default": f"$(find mavlink_sitl_gazebo)/worlds/{worldfile}"})
        ET.SubElement(launch, 'arg', attrib={"name": "gui", "default": "true"})
        ET.SubElement(launch, 'arg', attrib={"name": "debug", "default": "false"})
        ET.SubElement(launch, 'arg', attrib={"name": "verbose", "default": "false"})
        ET.SubElement(launch, 'arg', attrib={"name": "paused", "default": "false"})

        # gazebo_simulation config
        gazebo_sim = ET.SubElement(launch, 'include', attrib={"file": "$(find gazebo_ros)/launch/empty_world.launch"})
        ET.SubElement(gazebo_sim, 'arg', attrib={"name": "gui", "value": "$(arg gui)"})
        ET.SubElement(gazebo_sim, 'arg', attrib={"name": "world_name", "value": "$(arg world)"})
        ET.SubElement(gazebo_sim, 'arg', attrib={"name": "debug", "value": "$(arg debug)"})
        ET.SubElement(gazebo_sim, 'arg', attrib={"name": "verbose", "value": "$(arg verbose)"})
        ET.SubElement(gazebo_sim, 'arg', attrib={"name": "paused", "value": "$(arg paused)"})

        # config for vehicles
        local_port = self.local_port
        remote_port = self.remote_port
        sitl_port = self.sitl_port
        tcp_port = self.tcp_port
        topic_type = self.topic_name[self.name_list.currentText()]
        for i in range(len(self.vehicles)):
            vehicle = self.vehicles[i]
            sensor = self.sensors[i]
            model = vehicle + self.sensors_to_names[sensor]
            init_pos = self.init_pos[i]
            if topic_type == 0:
                topic_name = f"{vehicle}_{i}"
            if topic_type == 1:
                topic_name = f"uav_{i}"
            # init arg
            agent = ET.SubElement(launch, 'group', attrib={"ns": topic_name})
            ET.SubElement(agent, 'arg', attrib={"name": "ID", "value": f"{i}"})
            #ET.SubElement(agent, 'arg', attrib={"name": "ID_in_group", "value": f"{i}"})
            ET.SubElement(agent, 'arg', attrib={"name": "fcu_url", "value": f"udp://:{remote_port}@localhost:{local_port}"})
            for pos_str in "xyzRPY":
                ET.SubElement(agent, 'arg', attrib={"name": pos_str, "default": str(init_pos[pos_str])})
            ET.SubElement(agent, 'arg', attrib={"name": "sdf", "default": f"$(find px4_cmd)/models/{model}/{model}.sdf"})
            # px4 config
            px4 = ET.SubElement(agent, 'include', attrib={"file": "$(find px4)/launch/px4.launch"})
            ET.SubElement(px4, 'arg', attrib={"name": "est", "value": "$(arg est)"})
            ET.SubElement(px4, 'arg', attrib={"name": "vehicle", "value": vehicle})
            #ET.SubElement(px4, 'arg', attrib={"name": "sdf", "value": f"$(find mavlink_sitl_gazebo)/models/{vehicle}/{vehicle}.sdf"})
            #ET.SubElement(px4, 'arg', attrib={"name": "mavlink_udp_port", "value": str(sitl_port)})
            #ET.SubElement(px4, 'arg', attrib={"name": "mavlink_tcp_port", "value": str(tcp_port)})
            ET.SubElement(px4, 'arg', attrib={"name": "ID", "value": "$(arg ID)"})
            #ET.SubElement(px4, 'arg', attrib={"name": "ID_in_group", "value": "$(arg ID_in_group)"})
            # spawn model
            ET.SubElement(agent, 'node', attrib={"name": "$(anon vehicle_spawn)", "pkg": "gazebo_ros", "type": "spawn_model", "output": "screen", "args": f"-sdf -file $(arg sdf) -model {topic_name} -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)"})
            # marvros
            mavros = ET.SubElement(agent, 'include', attrib={"file": "$(find mavros)/launch/px4.launch"})
            ET.SubElement(mavros, 'arg', attrib={"name": "fcu_url", "value": "$(arg fcu_url)"})
            ET.SubElement(mavros, 'arg', attrib={"name": "gcs_url", "value": ""})
            ET.SubElement(mavros, 'arg', attrib={"name": "tgt_system", "value": "$(eval 1 + arg('ID'))"})
            ET.SubElement(mavros, 'arg', attrib={"name": "tgt_component", "value": "1"})
            local_port = local_port + 1
            remote_port = remote_port + 1
            sitl_port = sitl_port + 1
            tcp_port = tcp_port + 1
        # generate tree 
        tree = ET.ElementTree(launch)
        # pretty xml
        self.pretty_xml(tree.getroot(), indent='\t', newline='\n')
        # write to file
        tree.write(self.output_file, encoding="utf-8", xml_declaration=True)
        with open(self.output_file, 'a+') as f:
            f.write("\n<!--the launch file is generated by PX4 Cmd generator.py  -->")
        msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Information, 'Info', f'Generate Launch File to {self.output_file} Successfully!')
        msg_box.exec_()

    
    # elemnt为传进来的Elment类，参数indent用于缩进，newline用于换行
    def pretty_xml(self, element, indent, newline, level = 0) -> None:  
        if element:  # 判断element是否有子元素    
            if (element.text is None) or element.text.isspace():  # 如果element的text没有内容
                element.text = newline + indent * (level + 1)
            else:
                element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * (level + 1)
        temp = list(element)  # 将element转成list
        for subelement in temp:
            if temp.index(subelement) < (len(temp) - 1):  # 如果不是list的最后一个元素，说明下一个行是同级别元素的起始，缩进应一致
                subelement.tail = newline + indent * (level + 1)
            else:  # 如果是list的最后一个元素， 说明下一行是母元素的结束，缩进应该少一个    
                subelement.tail = newline + indent * level
            self.pretty_xml(subelement, indent, newline, level=level + 1)  # 对子元素进行递归操作


    # detect enviroment
    def detect_env(self) -> str:
        error_msg = ""
        ros_detect = "which rospack"
        pack_list = "rospack list-names"
        ros_detect = os.popen(ros_detect).read().strip('\n').strip()
        error_header = "This Programme requires ROS enviroment and PX4 installation and px4_cmd ROS Package.\n"
        if ros_detect == "":
            error_msg = error_header + "Please Check ROS Installation & rospack command."
            return error_msg
        pack_list = os.popen(pack_list).read().strip('\n').strip()
        pack_list = pack_list.split('\n')
        if "px4" not in pack_list:
            error_msg = error_header + "Please Check PX4 ROS Package Installation."
            return error_msg
        if "mavlink_sitl_gazebo" not in pack_list:
            error_msg = error_header + "Please Check mavlink_sitl_gazebo ROS Package Installation."
            return error_msg
        if "px4_cmd" not in pack_list:
            error_msg = error_header + "Please Check px4_cmd ROS Package Installation.\nGithub: https://github.com/Lovely-XPP/PX4_cmd/"
            return error_msg
        return error_msg

        
    # main
    def setup(self) -> None:
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        app = QtWidgets.QApplication(sys.argv)
        app.setStyle('Fusion')
        icon = QtWidgets.QApplication.style().standardIcon(QtWidgets.QStyle.SP_FileIcon)
        main_win = QtWidgets.QMainWindow()
        main_win.setFixedSize(1080, 700)
        main_win.setWindowIcon(icon)
        main_win.setWindowTitle("PX4 Cmd Launch File Generator")
        main_win.setStyleSheet("background-color: rgb(255,250,250)")
        self.main_win = main_win
        err = self.detect_env()
        if err != "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', err)
            msg_box.exec_()
            sys.exit()
        self.add_list()
        self.input_init_pos()
        self.choose_dir()
        self.add_buttons()
        self.add_table()
        self.main_win.show()
        sys.exit(app.exec_())


if __name__ == "__main__":
    ui = launch_generator()
    ui.setup()