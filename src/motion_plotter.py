import matplotlib.pyplot as plt
import yaml

recorded_motion_file_dir = "/home/pi/catkin_ws/src/dynamixel-workbench/dynamixel_workbench_operators/config/motion.yaml"


def open_yaml():
    # read yaml file
    with open(recorded_motion_file_dir, "r") as stream:
        try:
            data_loaded = yaml.safe_load(stream)
            return data_loaded
        except yaml.YAMLError as exc:
            return exc


def main():
    data = open_yaml()
    for motor_pos, motor_name in enumerate(data["joint"]["names"]):
        motor_position = []
        motor_velocity = []
        motor_acceleration = []
        motor_time_from_start = []
        for i, motion_name in enumerate(data["motion"]["names"]):
            motion_data = data["motion"][motion_name]
            motor_position.append(motion_data["step"][motor_pos])
            motor_velocity.append(motion_data["velocity"][motor_pos])
            motor_acceleration.append(motion_data["accelerations"][motor_pos])
            motor_time_from_start.append(motion_data["time_from_start"])
        plt.subplots(motor_pos + 1, 1)
        plt.plot(motor_time_from_start, motor_position)
        plt.subplots(motor_pos + 1, 2)
        plt.plot(motor_time_from_start, motor_velocity)
        plt.subplots(motor_pos + 1, 3)
        plt.plot(motor_time_from_start, motor_acceleration)
        plt.show()


if __name__ == "__main__":
    main()
