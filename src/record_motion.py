#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
file_dir = '/home/pi/catkin_ws/src/dynamixel-workbench/dynamixel_workbench_operators/config/motion.yaml'
with open(file_dir,'w') as f:
    f.write('joint:\n')
    f.write('  names: [pan, tilt]\n')
    f.write('motion:\n')
    


seq_list = []
last_position=None
last_stamp= None
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
    
    num1 = input('press enter to start then enter to end')
    start_time = rospy.Time.now()
    sub = rospy.Subscriber('dynamixel_workbench/joint_states', JointState, callback)
    num1 = input('')
    sub.unregister()
    with open(file_dir,'a') as f:
        f.write(f'  names: {seq_list}\n')    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    listener()
