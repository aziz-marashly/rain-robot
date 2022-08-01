#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandRequest

file_dir = '/home/pi/catkin_ws/src/dynamixel-workbench/dynamixel_workbench_operators/config/motion.yaml'
with open(file_dir,'w') as f:
    f.write('joint:\n')
    f.write('  names: [elbow, shoulder, waist, wrist_roll, wrist_swing]\n')
    f.write('motion:\n')
    


seq_list = []
last_position=None
last_stamp= None
packet_buffer = []
def callback(data):
    global start_time
    global last_position
    global last_stamp
    global seq_list
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', f'{data}')
    print(f'{data.header.seq}')
    print(f'{(data.header.stamp - start_time).to_sec() }')
    print(f'{list(data.position) }')
    
    
    
    
    if last_position is not None:
        for i,position in enumerate(list(data.position)):
            if(abs(last_position[i] - position) > ((data.header.stamp.to_sec() - last_stamp.to_sec())*6)):
                print('WOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOW so fast')
                return
    # packet_buffer.append(data)    
    
    # if len(packet_buffer) == 5:
    #     temp_array=[]
    #     median_position =[len(value.position)]
    #     for motors in len(value.position):
    #         for i, value in enumerate(packet_buffer):
    #             temp_array.append( value.position[motor])
    #         temp_array.sort()
    #         median_position[motors] = temp_array[3]

    #     temp_array=[]
    #     print('hhhhhhhhhhhhhhhiiiiiiiiiiiiiiiiiiii')
    #     print(median_position)

    with open(file_dir,'a') as f:
        f.write(f'  A{data.header.seq}:\n')
        f.write(f'    step: {list(data.position)}\n')
        f.write(f'    time_from_start: {(data.header.stamp - start_time).to_sec()/2 }\n')
        seq_list.append(f'A{data.header.seq}')
        
    last_position = list(data.position)
    last_stamp = data.header.stamp
    
def listener():
    rospy.init_node('listener', anonymous=True)
    global start_time
    torque(0)
    num1 = input('press enter to start then enter to end')
    start_time = rospy.Time.now()
    sub = rospy.Subscriber('dynamixel_workbench/joint_states', JointState, callback)
    num1 = input('')
    sub.unregister()
    
    with open(file_dir,'a') as f:
        f.write(f'  names: {seq_list}\n')    # spin() simply keeps python from exiting until this node is stopped
    torque(1023)

    # rospy.spin()

def torque(a):
    motor_list=[1,2,4,16,5]
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:
        dynamixel_command = rospy.ServiceProxy('dynamixel_workbench/dynamixel_command', DynamixelCommand)
        for motor in motor_list:
            resp1 = dynamixel_command(DynamixelCommandRequest('',motor,'Torque_Limit',a))
            print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    

if __name__ == '__main__':
    listener()
