#!/usr/bin/env python3

import rospy
import sys
import rospkg
import yaml

rospack = rospkg.RosPack()

gui_yaml_path = rospack.get_path("chargepal_monitor_gui") + "/cfg/gui.yaml"


def load_yaml_params(file_path):
    with open(file_path, "r") as stream:
        try:
            params = yaml.safe_load(stream)
            return params
        except yaml.YAMLError as exc:
            rospy.logerr(f"Error loading config.yaml file: {exc}")
            return None


def set_ros_params(params):
    if params:
        for key, value in params.items():
            rospy.set_param(key, value)


def set_gui_params(updated_values):

    with open(gui_yaml_path, "r") as file:
        data = yaml.safe_load(file)

    # Update the specific values
    for key, value in updated_values.items():
        data[key] = value

    with open(gui_yaml_path, "w") as file:
        yaml.dump(data, file)


if __name__ == "__main__":
    rospy.init_node("chargepal_load_params_node")
    config_file_path = rospack.get_path("chargepal_bundle") + "/cfg/config.yaml"
    config_params = load_yaml_params(config_file_path)
    set_ros_params(config_params)

    updated_values = {
        "ongoing_action": "",
        "ongoing_job": "",
        "gui_demo_resume": False,
        "gui_demo_start": False,
        "gui_demo_stop": True,
        "gui_log": "",
        "gui_loop_input": 0,
        "robot_current_loop": 0,
    }

    set_gui_params(updated_values)
    rospy.spin()
