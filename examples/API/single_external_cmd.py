import rospy
from px4_cmd.msg import Command
from px4_cmd.vehicle_external_command import vehicle_external_command
import os, time, math

if __name__ == "__main__":
    # init ros node
    rospy.init_node("External_Command_Center_Python")
    
    uav = vehicle_external_command()
    uav.start()
    
    rospy.loginfo("External Command Start! Vehicle Count: 1")
    
    ############################## Edit  Here ##############################
    update_time = 0.02
    t = 0
    while (not rospy.is_shutdown()):
        uav.set_position(uav.init_x + 2 * math.sin(0.4 * t), uav.init_y + 2 * math.cos(0.4 * t), 3, Command.ENU)
        time.sleep(update_time)
        t = t + 0.02
