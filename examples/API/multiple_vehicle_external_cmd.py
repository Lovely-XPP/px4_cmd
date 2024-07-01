import rospy
from typing import List
from px4_cmd.msg import Command
from px4_cmd.vehicle_external_command import vehicle_external_command
import os, time, math

def detect_px4() -> List[str]:
    tmp = os.popen("rosnode list | grep 'mavros'")
    nodes_tmp = tmp.read().split("/mavros\n")
    if len(nodes_tmp) > 0:
        for i in range(len(nodes_tmp)):
            nodes_tmp[i] = nodes_tmp[i].strip('/').strip()
        nodes_tmp.pop()
        return nodes_tmp
    return []

if __name__ == "__main__":
    # init ros node
    rospy.init_node("External_Command_Center_Python")
    nodes = detect_px4()
    while (rospy.is_shutdown() or not len(nodes)):
        rospy.logwarn("Not Detected PX4 Running! Waiting for PX4 running ...")
        time.sleep(1)

    # init all agent
    uav :List[vehicle_external_command] = []
    
    for i in range(len(nodes)):
        uav.append(vehicle_external_command())
        uav[i].start(nodes[i])
    
    rospy.loginfo("External Command Start! Vehicle Count: " + str(len(nodes)))
    
    ############################## Edit  Here ##############################
    update_time = 0.02
    t = 0
    while (not rospy.is_shutdown()):
        for i in range(len(nodes)):
            uav[i].set_position(uav[i].init_x + 2 * math.sin(0.4 * t), uav[i].init_y + 2 * math.cos(0.4 * t), 3, Command.ENU)
        time.sleep(update_time)
        t = t + 0.02
