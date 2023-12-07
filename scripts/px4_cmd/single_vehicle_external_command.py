# Copyright (c) 2023 易鹏 中山大学航空航天学院
# Copyright (c) 2023 Peng Yi, Sun Yat-Sen University, School of Aeronautics and Astronautics

import time
import rospy
import threading
from typing import overload
from px4_cmd.msg import Command
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool

PI = 3.14159265358979323846

class single_vehicle_external_command:
    def __init__(self) -> None:
        self.update_time = 0.02
        self.external_cmd = Command()
        self.position = [0.0, 0.0, 0.0]
        self.attitude = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.angle_rate = [0.0, 0.0, 0.0]
        self.ext_on = False
        self.total_time = -1
        pass

    def start(self) -> None:
        rospy.init_node("Ext_cmd")
        topic_header = "/mavros/"
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = Command.ENU
        self.external_cmd.Move_mode = Command.XYZ_POS
        rospy.Subscriber(topic_header + "local_position/pose", PoseStamped, self.pos_cb, queue_size=20)
        rospy.Subscriber(topic_header + "local_position/velocity_local", TwistStamped, self.vel_cb, queue_size=20)
        rospy.Subscriber("/px4_cmd/ext_on", Bool, self.ext_state_cb, queue_size=20)
        self.ext_cmd_pub = rospy.Publisher("/px4_cmd/ext_command", Command, queue_size=50)
        while rospy.is_shutdown():
            time.sleep(self.update_time)
        progress_1 = threading.Thread(target=self.ros_sub_thread)
        progress_1.start()
        progress_2 = threading.Thread(target=self.ros_pub_thread)
        progress_2.start()

    def ros_sub_thread(self):
        rospy.spin()

    def ros_pub_thread(self):
        t = 0
        while not rospy.is_shutdown():
            while self.ext_cmd_pub.get_num_connections < 1:
                rospy.INFO("External Command: Waiting for user-define mode!")
                time.sleep(1)
                t = 0
            t = t + self.update_time
            self.external_cmd.ext_time = t
            self.external_cmd.ext_total_time = self.total_time
            self.ext_cmd_pub.publish(self.external_cmd)
            time.sleep(self.update_time)

    def pos_cb(self, msg: PoseStamped):
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z
        (self.attitude[0], self.attitude[1], self.attitude[2]) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    def vel_cb(self, msg: TwistStamped):
        self.velocity[0] = msg.twist.linear.x
        self.velocity[1] = msg.twist.linear.y
        self.velocity[2] = msg.twist.linear.z
        self.angle_rate[0] = msg.twist.angular.x
        self.angle_rate[1] = msg.twist.angular.y
        self.angle_rate[2] = msg.twist.angular.z

    @overload
    def set_position(self, x: float, y: float, z: float, frame: int = Command.ENU):
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = frame
        self.external_cmd.Move_mode = Command.XYZ_POS
        self.external_cmd.desire_cmd[0] = x
        self.external_cmd.desire_cmd[1] = y
        self.external_cmd.desire_cmd[2] = z

    @overload
    def set_position(self, x: float, y: float, z: float, yaw: float, frame: int = Command.ENU):
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = frame
        self.external_cmd.Move_mode = Command.XYZ_POS
        self.external_cmd.desire_cmd[0] = x
        self.external_cmd.desire_cmd[1] = y
        self.external_cmd.desire_cmd[2] = z
        self.external_cmd.yaw_cmd = yaw

    @overload
    def set_velocity(self, vx: float, vy: float, vz: float, frame: int = Command.ENU):
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = frame
        self.external_cmd.Move_mode = Command.XYZ_VEL
        self.external_cmd.desire_cmd[0] = vx
        self.external_cmd.desire_cmd[1] = vy
        self.external_cmd.desire_cmd[2] = vz
    
    @overload
    def set_velocity(self, vx: float, vy: float, vz: float, yaw: float, frame: int = Command.ENU):
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = frame
        self.external_cmd.Move_mode = Command.XYZ_VEL
        self.external_cmd.desire_cmd[0] = vx
        self.external_cmd.desire_cmd[1] = vy
        self.external_cmd.desire_cmd[2] = vz
        self.external_cmd.yaw_cmd = yaw

    @overload
    def set_velocity_with_height(self, vx: float, vy: float, z: float, frame: int = Command.ENU):
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = frame
        self.external_cmd.Move_mode = Command.XY_VEL_Z_POS
        self.external_cmd.desire_cmd[0] = vx
        self.external_cmd.desire_cmd[1] = vy
        self.external_cmd.desire_cmd[2] = z
    
    @overload
    def set_velocity_with_height(self, vx: float, vy: float, z: float, yaw: float, frame: int = Command.ENU):
        self.external_cmd.Mode = Command.Move
        self.external_cmd.Move_frame = frame
        self.external_cmd.Move_mode = Command.XY_VEL_Z_POS
        self.external_cmd.desire_cmd[0] = vx
        self.external_cmd.desire_cmd[1] = vy
        self.external_cmd.desire_cmd[2] = z
        self.external_cmd.yaw_cmd = yaw

    def ext_state_cb(self, msg: Bool):
        self.ext_on = msg.data