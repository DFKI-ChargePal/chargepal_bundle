#!/usr/bin/env python3

import rospy
import sys
import rospkg
import yaml

rospack = rospkg.RosPack()

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
            if "_path" in key:
                seperator = value.find('/')
                if seperator != -1:
                    pkg_name = value[:seperator]
                    pkg_path = value[seperator:]
                    value = rospack.get_path(pkg_name) + pkg_path
            rospy.set_param(key, value)


if __name__ == "__main__":
    rospy.init_node("chargepal_load_params_node")
    config_file_path = rospack.get_path("chargepal_bundle") + "/cfg/config.yaml"
    config_params = load_yaml_params(config_file_path)
    set_ros_params(config_params)
    rospy.spin()
