#!/usr/bin/env python3

import rospy
import sys
import rospkg
import yaml

rospack = rospkg.RosPack()
sys.path.insert(0,rospack.get_path("chargepal_client")+"/src/chargepal_client")

from chargepal_client.client import Grpc_Client

def load_yaml_params(file_path):
    with open(file_path, 'r') as stream:
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

def pull_ldb_to_rdb_rdb_copy():
    client_instance = Grpc_Client()
    client_instance.pull_ldb()


if __name__ == '__main__':
    rospy.init_node('chargepal_startup_node')
    config_file_path = '../cfg/config.yaml'
    config_params = load_yaml_params(config_file_path)
    set_ros_params(config_params)
    pull_ldb_to_rdb_rdb_copy()
    rospy.spin()
