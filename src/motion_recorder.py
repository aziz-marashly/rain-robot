#!/usr/bin/env python

import sys

import rospy
import yaml
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandRequest
from sensor_msgs.msg import JointState

recorded_motion_file_dir = "/home/pi/catkin_ws/src/dynamixel-workbench/dynamixel_workbench_operators/config/motion.yaml.base"
robot_yamel = rospy.get_param("/dynamixel_info")

seq_list = []
motor_name_id = {}
names = []
last_joint_state = None
last_trajectory = None
is_first_message = True
last_dynamixel_state = None


def get_motor_name_id():
    global motor_name_id
    # get motor id from the yamel file
    with open(robot_yamel) as f:
        data = yaml.load_all(f)
        for doc in data:
            for k, v in doc.items():
                motor_name_id[k] = v.get("ID")


def write_motion_file_header(names_data):
    global recorded_motion_file_dir
    global names
    names = names_data
    with open(recorded_motion_file_dir, "w") as f:
        f.write("joint:\n")
        f.write(f"  names: {names}\n")
        f.write("motion:\n")


def is_motion_a_noise(data):
    global last_joint_state
    if last_joint_state is not None:
        for i, position in enumerate(list(data.position)):
            if abs(last_joint_state.position[i] - position) > (
                (data.header.stamp.to_sec() - last_joint_state.header.stamp.to_sec())
                * 6
            ):
                return True


def write_point_to_yamel_file(data):
    global last_trajectory
    global last_joint_state

    if (data.header.stamp - last_joint_state.header.stamp).to_sec() == 0:
        return

    acceleration = []
    for i, v in enumerate(data.velocity):
        acceleration.append(
            (
                (v - last_joint_state.velocity[i])
                / (data.header.stamp - last_joint_state.header.stamp).to_sec()
            )
        )
    with open(recorded_motion_file_dir, "a") as f:
        f.write(f"  M{data.header.seq}:\n")
        f.write(f"    step: {list(data.position)}\n")
        f.write(f"    effort: {list(data.effort)}\n")
        f.write(f"    velocity: {list(data.velocity)}\n")
        f.write(f"    accelerations: {acceleration}\n")
        f.write(f"    time_from_start: {(data.header.stamp - start_time).to_sec()}\n")
        seq_list.append(f"M{data.header.seq}")
    sys.stdout.write(
        f"\r position num {len(seq_list)+1} {list(data.position)} after {(data.header.stamp - start_time).to_sec()}\n"
    )
    sys.stdout.flush()
    last_trajectory = data.position
    last_joint_state = data
    return last_joint_state


def add_motion_to_motion_file(data):
    global recorded_motion_file_dir
    global start_time
    global seq_list
    global last_trajectory
    global last_joint_state
    global minimum_time

    if last_joint_state is None or last_trajectory is None:
        last_joint_state = data
        last_trajectory = data.position
        return

    if is_motion_a_noise(data):
        print(f"seq {data.header.seq} is FFF noise\n")
        return

    last_joint_state = write_point_to_yamel_file(data)

    print(f"last_joint_state {last_joint_state.header.seq}\n")


def on_joint_state_message(message):
    print(f"getting new message with seq {message.header.seq} \n")
    global is_first_message

    if is_first_message:
        is_first_message = False
        write_motion_file_header(message.name)

    add_motion_to_motion_file(message)


def write_motion_file_footer():
    global seq_list
    global recorded_motion_file_dir
    with open(recorded_motion_file_dir, "a") as f:
        f.write(f"  names: {seq_list}\n")


def on_dynamixel_state_message(data):
    global last_dynamixel_state
    last_dynamixel_state = data


def main():
    global start_time
    get_motor_name_id()
    rospy.init_node("motion_recorder", anonymous=True)
    set_torque_limit(0)
    input("Press enter to start then enter to end")
    start_time = rospy.Time.now()
    sub = rospy.Subscriber(
        "dynamixel_workbench/joint_states", JointState, on_joint_state_message
    )
    rospy.Subscriber(
        "dynamixel_workbench/dynamixel_state",
        DynamixelStateList,
        on_dynamixel_state_message,
    )
    input("")
    sub.unregister()
    write_motion_file_footer()
    print(f"Motion recorded to {recorded_motion_file_dir}")
    reset_goal_position_to_current_position()
    # set current position as goal position
    set_torque_limit(1023)


def reset_goal_position_to_current_position():
    global names
    global last_dynamixel_state
    rospy.wait_for_service("dynamixel_workbench/dynamixel_command")
    try:
        dynamixel_command = rospy.ServiceProxy(
            "dynamixel_workbench/dynamixel_command", DynamixelCommand
        )
        for motor_name, motor_id in motor_name_id.items():

            goal_position = next(
                filter(
                    lambda state: state.id == motor_id,
                    last_dynamixel_state.dynamixel_state,
                ),
                None,
            ).present_position
            resp1 = dynamixel_command(
                DynamixelCommandRequest("", motor_id, "Goal_Position", goal_position)
            )
            print(
                f"set {goal_position} as the goal position for motor with id {motor_id}, {resp1}"
            )
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def set_torque_limit(a):
    global motor_name_id

    # set torque limit using dynamixel_command service
    rospy.wait_for_service("dynamixel_workbench/dynamixel_command")
    try:
        dynamixel_command = rospy.ServiceProxy(
            "dynamixel_workbench/dynamixel_command", DynamixelCommand
        )
        for motor in motor_name_id.values():
            resp1 = dynamixel_command(
                DynamixelCommandRequest("", motor, "Torque_Limit", a)
            )
            print(f"set {a} as the torque limit for motor with id {motor}, {resp1}")
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    main()
