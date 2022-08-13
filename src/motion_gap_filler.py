import operator
import os

import yaml

recorded_motion_file_dir = "/home/pi/catkin_ws/src/dynamixel-workbench/dynamixel_workbench_operators/config/motion.yaml"
minimum_time = 0.085
motion_data = None
new_motion_motion_data = {}


def main():
    global motion_data
    global new_motion_motion_data
    with open(recorded_motion_file_dir, "r") as f:
        motion_data = yaml.safe_load(f)

    last_motion = None
    last_motion_key = None
    for k, v in motion_data["motion"].items():
        if k == "names":
            continue
        if last_motion is None:
            last_motion = v
            last_motion_key = k
            continue
        time_diff = v["time_from_start"] - last_motion["time_from_start"]
        if time_diff > minimum_time:
            add_gap_points(last_motion, v, last_motion_key, k)
        last_motion = v
        last_motion_key = k

    motion_data["motion"].update(new_motion_motion_data)

    os.rename(recorded_motion_file_dir, recorded_motion_file_dir + ".old")
    with open(recorded_motion_file_dir, "a") as f:
        yaml.dump(motion_data, f)


def add_gap_points(point1, point2, point1key, point2key):
    global minimum_time
    global motion_data
    global new_motion_motion_data

    time_gap = point2["time_from_start"] - point1["time_from_start"]
    position_gap = tuple(map(operator.sub, point2["step"], point1["step"]))
    effort_gap = tuple(map(operator.sub, point2["effort"], point1["effort"]))
    velocity_gap = tuple(map(operator.sub, point2["velocity"], point1["velocity"]))
    accelerations_gap = tuple(
        map(operator.sub, point2["accelerations"], point1["accelerations"])
    )

    division_count = time_gap / minimum_time
    num_of_fillers = time_gap // minimum_time
    if division_count == num_of_fillers:
        num_of_fillers -= 1
    for i in range(1, int(num_of_fillers + 1)):
        filler_position = tuple(
            map(
                lambda p1, pg: p1 + ((pg / division_count) * (i)),
                point1["step"],
                position_gap,
            )
        )
        filler_effort = tuple(
            map(
                lambda p1, pg: p1 + ((pg / division_count) * (i)),
                point1["effort"],
                effort_gap,
            )
        )
        filler_velocity = tuple(
            map(
                lambda p1, pg: p1 + ((pg / division_count) * (i)),
                point1["velocity"],
                velocity_gap,
            )
        )
        filler_accelerations = tuple(
            map(
                lambda p1, pg: p1 + ((pg / division_count) * (i)),
                point1["accelerations"],
                accelerations_gap,
            )
        )

        filler_time_from_start = point1["time_from_start"] + (
            (time_gap / division_count) * (i)
        )
        filler_data_seq = str(point1key) + "-" + str(int(i))

        filler_point = {
            "step": list(filler_position),
            "effort": list(filler_effort),
            "velocity": list(filler_velocity),
            "accelerations": list(filler_accelerations),
            "time_from_start": filler_time_from_start,
        }

        new_motion_motion_data[filler_data_seq] = filler_point
        motion_data["motion"]["names"].insert(
            motion_data["motion"]["names"].index(point2key), filler_data_seq
        )
        print("\n filler added ", filler_data_seq)


if __name__ == "__main__":
    main()
