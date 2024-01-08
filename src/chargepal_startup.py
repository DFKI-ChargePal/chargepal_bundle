#!/usr/bin/env python3

import rospy
import sys
import rospkg

rospack = rospkg.RosPack()
sys.path.insert(0,rospack.get_path("chargepal_client")+"/src/chargepal_client")

from chargepal_client.client import Grpc_Client

def pull_ldb_to_rdb_rdb_copy():
    client_instance = Grpc_Client()
    client_instance.pull_ldb()


if __name__ == '__main__':
    rospy.init_node('chargepal_startup_node')
    pull_ldb_to_rdb_rdb_copy()
    rospy.spin()
