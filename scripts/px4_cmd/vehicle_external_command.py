# -*- coding: utf-8 -*-
# Copyright (c) 2023 易鹏 中山大学航空航天学院
# Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

import time
import rospy
import threading
from typing import List, Optional
from tf.transformations import euler_from_quaternion
from px4_cmd.msg import Command
from px4_cmd.custom_command import *
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

PI = 3.14159265358979323846

class vehicle_external_command:
    def __init__(self) -> None:
        self.update_time: float = 0.02
        self.init_x: float = 0
        self.init_y: float = 0
        self.init_z: float = 0
        self.init_R: float = 0
        self.init_P: float = 0
        self.init_Y: float = 0
        self.external_cmd: Command = Command()
        self.position: List[float] = [0.0, 0.0, 0.0]
        self.attitude: List[float] = [0.0, 0.0, 0.0]
        self.velocity: List[float] = [0.0, 0.0, 0.0]
        self.angle_rate: List[float] = [0.0, 0.0, 0.0]
        self.pos_cb_mutex = threading.Lock()
        self.vel_cb_mutex = threading.Lock()
        self.change_cmd_mutex = threading.Lock()
        pass

    def start(self, node: Optional[str] = None) -> None:
        """start API node

        Args:
            node (str | None): node name for vehicle, defined in topic name: /{node}/mavros/.... if None means single vehicle simulation
        """        
        node_name = node
        if node_name == None:
            node_name: str = ""
            topics = rospy.get_published_topics()
            for topic in topics:
                if "/mavros" in topic[0]:
                    node_name = topic[0].split("/mavros")[0].strip('/').strip()
                    break
        # start external command thread
        topic_header = "/" + node_name + "/mavros/"
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = Command.ENU
        self.external_cmd.Move_mode = Command.XYZ_POS
        self.init_x = rospy.get_param("/" + node_name + "/init_x")
        self.init_y = rospy.get_param("/" + node_name + "/init_y")
        self.init_z = rospy.get_param("/" + node_name + "/init_z")
        self.init_R = rospy.get_param("/" + node_name + "/init_R")
        self.init_P = rospy.get_param("/" + node_name + "/init_P")
        self.init_Y = rospy.get_param("/" + node_name + "/init_Y")
        rospy.Subscriber(topic_header + "local_position/pose", PoseStamped, self.pos_cb, queue_size=20)
        rospy.Subscriber(topic_header + "local_position/velocity_local", TwistStamped, self.vel_cb, queue_size=20)
        self.ext_cmd_pub = rospy.Publisher("/" + node_name + "/px4_cmd/external_command", Command, queue_size=50)
        while rospy.is_shutdown():
            time.sleep(self.update_time)
        progress_1 = threading.Thread(target=self.ros_sub_thread)
        progress_1.start()
        progress_2 = threading.Thread(target=self.ros_pub_thread)
        progress_2.start()

    def ros_sub_thread(self):
        """
        subscribe thread function
        """
        rospy.spin()

    def ros_pub_thread(self):
        """
        publish thread function
        """
        while not rospy.is_shutdown():
            self.ext_cmd_pub.publish(self.external_cmd)
            time.sleep(self.update_time)

    def pos_cb(self, msg: PoseStamped):
        """
        position and pose subscriber callback function
        """
        self.pos_cb_mutex.acquire()
        self.position[0] = msg.pose.position.x + self.init_x
        self.position[1] = msg.pose.position.y + self.init_y
        self.position[2] = msg.pose.position.z + self.init_z
        (R, P, Y) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.attitude[0] = P + self.init_P
        self.attitude[1] = R + self.init_R
        self.attitude[2] = Y + self.init_Y
        self.pos_cb_mutex.release()

    def vel_cb(self, msg: TwistStamped):
        """
        velocity and angle rate subscriber callback function
        """
        self.vel_cb_mutex.acquire()
        self.velocity[0] = msg.twist.linear.x
        self.velocity[1] = msg.twist.linear.y
        self.velocity[2] = msg.twist.linear.z
        self.angle_rate[0] = msg.twist.angular.x
        self.angle_rate[1] = msg.twist.angular.y
        self.angle_rate[2] = msg.twist.angular.z
        self.vel_cb_mutex.release()

    def set_position(self, x: float, y: float, z: float, yaw: Optional[float] = None, frame: int = Command.ENU):
        """setting position command in 3 axis for vehicle

        Args:
            x (float): desire position in x axis
            y (float): desire position in y axis
            z (float): desire position in z axis
            yaw (float | None, optional): desire yaw command. Defaults to None.
            frame (int, optional): position in which frame, px4_cmd.Command.ENU / px4_cmd.Command.BODY. Defaults to Command.ENU.
        """
        self.change_cmd_mutex.acquire()
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = frame
        self.external_cmd.Move_mode = Command.XYZ_POS
        self.external_cmd.desire_cmd[0] = x
        self.external_cmd.desire_cmd[1] = y
        self.external_cmd.desire_cmd[2] = z
        if yaw != None:
            self.external_cmd.yaw_cmd = yaw
        self.change_cmd_mutex.release()
    
    def set_velocity(self, vx: float, vy: float, vz: float, yaw: Optional[float] = None, frame: int = Command.ENU):
        """setting velocity command in 3 axis for vehicle

        Args:
            vx (float): desire velocity in x axis
            vy (float): desire velocity in y axis
            vz (float): desire velocity in z axis
            yaw (float | None, optional): desire yaw command. Defaults to None.
            frame (int, optional): velocity in which frame, px4_cmd.Command.ENU / px4_cmd.Command.BODY. Defaults to Command.ENU.
        """
        self.change_cmd_mutex.acquire()
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = frame
        self.external_cmd.Move_mode = Command.XYZ_VEL
        self.external_cmd.desire_cmd[0] = vx
        self.external_cmd.desire_cmd[1] = vy
        self.external_cmd.desire_cmd[2] = vz
        if yaw != None:
            self.external_cmd.yaw_cmd = yaw
        self.change_cmd_mutex.release()

    def set_velocity_with_height(self, vx: float, vy: float, z: float, yaw: Optional[float] = None, frame: int = Command.ENU):
        """setting velocity command in 2 axis with height command for vehicle

        Args:
            vx (float): desire velocity in x axis
            vy (float): desire velocity in y axis
            z (float): desire height
            yaw (float | None, optional): desire yaw command. Defaults to None.
            frame (int, optional): velocity in which frame, px4_cmd.Command.ENU / px4_cmd.Command.BODY. Defaults to Command.ENU.
        """
        self.change_cmd_mutex.acquire()
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = frame
        self.external_cmd.Move_mode = Command.XY_VEL_Z_POS
        self.external_cmd.desire_cmd[0] = vx
        self.external_cmd.desire_cmd[1] = vy
        self.external_cmd.desire_cmd[2] = z
        if yaw != None:
            self.external_cmd.yaw_cmd = yaw
        self.change_cmd_mutex.release()
    
    def set_custom_command(self, cmd: CustomCommand) -> None:
        """setting custom command

        Args:
            cmd (CustomCommand): custom command struct
        """        
        self.change_cmd_mutex.acquire()
        self.external_cmd = custom_command_to_px4_msg(cmd)
        self.change_cmd_mutex.release()

    def set_hover(self, yaw: Optional[float] = None) -> None:
        """setting vehicle to hover mode

        Args:
            yaw (float | None, optional): desire yaw command. Defaults to None.
        """
        self.change_cmd_mutex.acquire()
        self.external_cmd.Mode = Command.Hover
        self.external_cmd.Move_frame = Command.ENU
        self.external_cmd.Move_mode = Command.XY_VEL_Z_POS
        if yaw != None:
            self.external_cmd.yaw_cmd = yaw
        self.change_cmd_mutex.release()