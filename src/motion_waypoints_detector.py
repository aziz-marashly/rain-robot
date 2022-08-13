# get working directory
import os

import yaml
from scipy.interpolate import UnivariateSpline  # for interpolation

recorded_motion_file_dir = "/home/pi/catkin_ws/src/dynamixel-workbench/dynamixel_workbench_operators/config/motion.yaml"
joint_names = []
# get current working directory
time_slicing_period = 0.085
playback_speed = 1
smoothing_factor = 0.5


def open_yaml():

    # read yaml file
    with open(recorded_motion_file_dir + ".base", "r") as stream:
        try:
            data_loaded = yaml.safe_load(stream)
            return data_loaded
        except yaml.YAMLError as exc:
            return exc


def yaser():
    data = open_yaml()
    global joint_names
    joint_names = data["joint"]["names"]
    times = [[] for i in range(len(joint_names))]
    points = [[] for i in range(len(joint_names))]
    new_points = []
    new_times = []
    last_time = []
    positions = []
    velocities = []
    accelerations = []
    step = []
    vel = []
    acc = []
    for j, point in enumerate(data["motion"]["names"]):
        for i, motion in enumerate(data["motion"][point]["step"]):
            points[i].append(motion)
            times[i].append(data["motion"][point]["time_from_start"])
    for i, name in enumerate(joint_names):
        position = UnivariateSpline(times[i], points[i], k=4, s=1)
        velocity = position.derivative()
        acceleration = velocity.derivative()
        positions.append(position)
        velocities.append(velocity)
        accelerations.append(acceleration)
        new_time = list(position.derivative().roots())
        new_times += new_time
    first_time = data["motion"][data["motion"]["names"][0]]["time_from_start"]
    last_time = (
        data["motion"][data["motion"]["names"][-1]]["time_from_start"] / playback_speed
    )
    new_times = [first_time] + new_times + [last_time]
    new_times.sort()
    temp = 0
    t_pair_count = int(last_time / time_slicing_period)
    for t in range(t_pair_count):
        for i, name in enumerate(joint_names):
            step.append(float(positions[i](temp)))
            vel.append(float(velocities[i](t)))
            acc.append(float(accelerations[i](t)))
        point = {
            "step": step,
            "velocity": vel,
            "accelerations": acc,
            "time_from_start": temp,
        }
        new_points.append(point)
        temp += time_slicing_period
        step = []
        vel = []
        acc = []
    return new_points


def main():
    global joint_names
    new_motion_steps = yaser()
    # yaser put your code here

    motion_data = {}
    motion_data["joint"] = {}
    motion_data["motion"] = {}
    motion_data["joint"]["names"] = joint_names
    motion_names = []
    for i, generated_point in enumerate(new_motion_steps):
        motion_name = f"M{i}"
        motio_point = {}
        motio_point["step"] = list(generated_point["step"])
        motio_point["velocity"] = list(generated_point["velocity"])
        motio_point["accelerations"] = list(generated_point["accelerations"])
        motio_point["time_from_start"] = float(generated_point["time_from_start"])
        motion_data["motion"][motion_name] = motio_point
        motion_names.append(motion_name)

    motion_data["motion"]["names"] = motion_names
    os.rename(recorded_motion_file_dir, recorded_motion_file_dir + ".old")
    with open(recorded_motion_file_dir, "a") as f:
        yaml.dump(motion_data, f)


if __name__ == "__main__":
    main()


# array of array of positions [[positions],[positions],[positions],[positions],[positions],[positions]...]
# array of array of times [[times],[times],[times],[times],[times],[times]...]

# array of points [point1, point2, point3, point4, point5, point6 ...]
# time_of_start


# yamlFile = [
#     {
#         'joint':{
#             'names':joint_names,
#         }
#     },
#     {
#         'motion':
#         {
#             'names':['M'+str(i) for i in range(1,len(points[0])+1)],
#         }
#     }
# ]
"""
joint:
  names: [pan, tilt]
motion:
  names: [right, zero, left]
  right:
    step: [-3.14, -3.14]  # radian
    time_from_start: 1.0  # sec
  zero:
    step: [0.0, 0.0]
    time_from_start: 2.0
  left:
    step: [3.14, 3.14]
    time_from_start: 3.0
"""
