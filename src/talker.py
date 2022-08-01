#!/usr/bin/env python Software License Agreement (BSD License)

import rospy

from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandRequest

def sendCommand():
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:
        dynamixel_command = rospy.ServiceProxy('dynamixel_workbench/dynamixel_command', DynamixelCommand)
        resp1 = dynamixel_command(DynamixelCommandRequest('',1,'Torque_Limit',0))
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        


if __name__ == '__main__':
    try:
        sendCommand()
    except rospy.ROSInterruptException:
        pass
