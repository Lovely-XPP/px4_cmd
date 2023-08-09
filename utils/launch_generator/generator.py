import sys
import xml.etree.ElementTree as ET
from PyQt5 import QtWidgets, QtCore, QtGui


class launch_generator():
    # initial
    def __init__(self) -> None:
        self.vehicles = []
        self.sensors = []
        self.init_pos = []
        self.output_folder = ""
        self.local_port = 34580
        self.remote_port = 24540
        self.sitl_port = 18570
        self.tcp_port = 4560
        self.types = ["iris", "typhon_h480", "plane"]
        pass


    # get_types
    def get_type(self) -> list:
        return self.types
    

    # get_type_sensors
    def get_sensor(self, type) -> list:
        if "iris" in type:
            sensors = ["None", "Camera", "Realsense", "2D-Lidar", "3D-Lidar"]
        if "typhon_h480" in type:
            sensors = ["None", "Camera", "3D-Lidar"]
        if "plane" in type:
            sensors = ["None", "Camera"]
        return sensors
        

    # choose type
    def set_type(self, main_win) -> None:
        # lable
        lable = QtWidgets.QLabel("Vehicle Type",main_win)
        lable.move(50, 20)
        lable.resize(150, 35)

        # select
        list = QtWidgets.QComboBox(main_win)
        list.addItems(self.get_type())
        list.move(150, 20)
        list.resize(200, 35)
        list.setStyleSheet("background-color: rgb(135,206,235)")
        self.type_list = list

    
    # choose sensor
    def set_sensor(self, main_win) -> None:
        # lable
        lable = QtWidgets.QLabel("Sensor Type", main_win)
        lable.move(450, 20)
        lable.resize(150, 35)

        # select
        list = QtWidgets.QComboBox(main_win)
        list.addItems(["None", "Camera", "Realsense", "2D-Lidar", "3D-Lidar"])
        list.move(550, 20)
        list.resize(200, 35)
        list.setStyleSheet("background-color: rgb(155,205,155)")
        self.sensor_list = list
        self.type_list.currentIndexChanged.connect(self.association)
    

    # input initial position
    def input_init_pos(self, main_win):
        lable0 = QtWidgets.QLabel("------------------------------------------------------------------ Initial Position ------------------------------------------------------------------", main_win)
        lable0.move(50, 55)
        lable0.resize(700, 35)
        # x
        # lable
        lable1 = QtWidgets.QLabel("x", main_win)
        lable1.move(50, 90)
        lable1.resize(30, 35)
        # text
        text1 = QtWidgets.QLineEdit(main_win)
        text1.setReadOnly(False)
        text1.move(70, 90)
        text1.resize(80, 35)
        self.text_x = text1

        # y
        # lable
        lable2 = QtWidgets.QLabel("y", main_win)
        lable2.move(170, 90)
        lable2.resize(30, 35)
        # text
        text2 = QtWidgets.QLineEdit(main_win)
        text2.setReadOnly(False)
        text2.move(190, 90)
        text2.resize(80, 35)
        self.text_y = text2

        # z
        # lable
        lable3 = QtWidgets.QLabel("z", main_win)
        lable3.move(290, 90)
        lable3.resize(30, 35)
        # text
        text3 = QtWidgets.QLineEdit(main_win)
        text3.setReadOnly(False)
        text3.move(310, 90)
        text3.resize(80, 35)
        self.text_z = text3

        # R
        # lable
        lable4 = QtWidgets.QLabel("R", main_win)
        lable4.move(410, 90)
        lable4.resize(30, 35)
        # text
        text4 = QtWidgets.QLineEdit(main_win)
        text4.setReadOnly(False)
        text4.move(430, 90)
        text4.resize(80, 35)
        self.text_R = text4

        # P
        # lable
        lable5 = QtWidgets.QLabel("P", main_win)
        lable5.move(530, 90)
        lable5.resize(30, 35)
        # text
        text5 = QtWidgets.QLineEdit(main_win)
        text5.setReadOnly(False)
        text5.move(550, 90)
        text5.resize(80, 35)
        self.text_P = text5

        # Y
        # lable
        lable6 = QtWidgets.QLabel("Y", main_win)
        lable6.move(650, 90)
        lable6.resize(30, 35)
        # text
        text6 = QtWidgets.QLineEdit(main_win)
        text6.setReadOnly(False)
        text6.move(670, 90)
        text6.resize(80, 35)
        self.text_Y = text6


    # choose dir
    def choose_dir(self, main_win) -> None:
        # lable
        lable = QtWidgets.QLabel("Output Dir", main_win)
        lable.move(50, 140)
        lable.resize(100, 35)

        # show dir
        text = QtWidgets.QLineEdit(main_win)
        text.setReadOnly(True)
        text.move(150, 140)
        text.resize(500, 35)
        self.text = text

        # browse button
        button = QtWidgets.QPushButton("Browse", main_win)
        button.move(660, 140)
        button.resize(90, 35)
        button.setStyleSheet("background-color: rgb(255,235,230)")
        self.browse_button = button
        self.browse_button.clicked.connect(self.browse_dir)
    

    # browse dir
    def browse_dir(self):
        folder = QtWidgets.QFileDialog.getExistingDirectory(None,"Choose Saved Launch File Dir")
        self.text.setText(folder)
        self.output_folder = folder


    # Associate sensor with type
    def association(self) -> None:
        type = self.type_list.currentText()
        self.sensor_list.clear()
        self.sensor_list.addItems(self.get_sensor(type))


    # Add buttons
    def add_buttons(self, main_win) -> None:
        # add vehicle button
        button1 = QtWidgets.QPushButton("Add Vehicle", main_win)
        button1.move(50, 190)
        button1.resize(300, 55)
        button1.setStyleSheet("background-color: rgb(176,226,255)")
        # generate button
        button2 = QtWidgets.QPushButton("Generate Launch", main_win)
        button2.move(450, 190)
        button2.resize(300, 55)
        button2.setStyleSheet("background-color: rgb(84,255,159)")
        # set trigger
        self.add_button = button1
        self.generate_button = button2
        self.add_button.clicked.connect(self.add_vehicle)
        self.generate_button.clicked.connect(self.generate_launch)

    
    # Add table
    def add_table(self, main_win):
        table = QtWidgets.QTableView(main_win)
        table.move(50, 260)
        table.resize(700, 320)
        table.setStyleSheet("background-color: white")
        self.table = table


    # Update table
    def update_table(self):
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
            model.setItem(row, 0, item1)
            model.setItem(row, 1, item2)
            model.setItem(row, 2, item3)
            model.setItem(row, 3, item4)
            model.setItem(row, 4, item5)
            model.setItem(row, 5, item6)
            model.setItem(row, 6, item7)
            model.setItem(row, 7, item8)
        self.table.setModel(model)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)


    # Add vehicle button signal handle
    def add_vehicle(self) -> None:
        self.vehicles.append(self.type_list.currentText())
        self.sensors.append(self.sensor_list.currentText())
        init_pos = {}
        if self.text_x.text() == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Input Initial Postion!')
            msg_box.exec_()
            return
        init_pos['x'] = self.text_x.text()
        if self.text_y.text() == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Input Initial Postion!')
            msg_box.exec_()
            return
        init_pos['y'] = self.text_y.text()
        if self.text_z.text() == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Input Initial Postion!')
            msg_box.exec_()
            return
        init_pos['z'] = self.text_z.text()
        if self.text_R.text() == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Input Initial Postion!')
            msg_box.exec_()
            return
        init_pos['R'] = self.text_R.text()
        if self.text_P.text() == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Input Initial Postion!')
            msg_box.exec_()
            return
        init_pos['P'] = self.text_P.text()
        if self.text_Y.text() == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Input Initial Postion!')
            msg_box.exec_()
            return
        init_pos['Y'] = self.text_Y.text()
        self.init_pos.append(init_pos)
        self.update_table()

    
    # generate launch
    def generate_launch(self) -> None:
        if len(self.vehicles) == 0:
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Add Vehicle First!')
            msg_box.exec_()
            return
        if self.output_folder == "":
            msg_box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Critical, 'Error', 'Please Browse Output Dir!')
            msg_box.exec_()
            return
        # init new xml
        launch = ET.Element('launch')
        # arg config
        ET.SubElement(launch, 'arg', attrib={"name": "est", "default": "ekf2"})
        ET.SubElement(launch, 'arg', attrib={"name": "world", "default": "$(find mavlink_sitl_gazebo)/worlds/outdoor2.world"})
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
        for i in range(len(self.vehicles)):
            vehicle = self.vehicles[i]
            sensor = self.sensors[i]
            init_pos = self.init_pos[i]
            # init arg
            agent = ET.SubElement(gazebo_sim, 'group', attrib={"ns": f"{vehicle}_{i}"})
            ET.SubElement(agent, 'arg', attrib={"name": "ID", "value": f"{i}"})
            ET.SubElement(agent, 'arg', attrib={"name": "ID_in_group", "value": f"{i}"})
            ET.SubElement(agent, 'arg', attrib={"name": "fcu_url", "value": f"udp://:{remote_port}@localhost:{local_port}"})
            # px4 config
            px4 = ET.SubElement(agent, 'include', attrib={"file": "$(find px4)/launch/single_vehicle_spawn_xtd.launch"})
            for pos_str in "xyzRPY":
                ET.SubElement(px4, 'arg', attrib={"name": pos_str, "value": str(init_pos[pos_str])})
            ET.SubElement(px4, 'arg', attrib={"name": "vehicle", "value": vehicle})
            ET.SubElement(px4, 'arg', attrib={"name": "sdf", "value": vehicle})
            ET.SubElement(px4, 'arg', attrib={"name": "mavlink_udp_port", "value": str(sitl_port)})
            ET.SubElement(px4, 'arg', attrib={"name": "mavlink_tcp_port", "value": str(tcp_port)})
            ET.SubElement(px4, 'arg', attrib={"name": "ID", "value": "$(arg ID)"})
            ET.SubElement(px4, 'arg', attrib={"name": "ID_in_group", "value": "$(arg ID_in_group)"})
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
        tree.write(self.output_folder + "/test.xml", encoding="utf-8")
        with open(self.output_folder + "/test.xml", 'a+') as f:
            f.write("\n<!--the launch file is generated by PX4 Cmd generator.py  -->")

    
    # elemnt为传进来的Elment类，参数indent用于缩进，newline用于换行
    def pretty_xml(self, element, indent, newline, level = 0):  
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

        
    # main
    def setup(self) -> None:
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        app = QtWidgets.QApplication(sys.argv)
        main_win = QtWidgets.QMainWindow()
        main_win.resize(800, 600)
        main_win.setWindowTitle("PX4 Cmd Launch Generator")
        main_win.setStyleSheet("background-color: rgb(255,250,250)")
        self.set_type(main_win)
        self.set_sensor(main_win)
        self.input_init_pos(main_win)
        self.choose_dir(main_win)
        self.add_buttons(main_win)
        self.add_table(main_win)
        main_win.show()
        sys.exit(app.exec_())


if __name__ == "__main__":
    ui = launch_generator()
    ui.setup()