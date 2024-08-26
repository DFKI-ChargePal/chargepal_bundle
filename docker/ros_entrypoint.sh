#!/bin/bash

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source /root/catkin_ws/devel/setup.bash


config_branch=$(printenv CHARGEPAL_MAP_CONFIG_BRANCH)

if ! [ -z $config_branch ]
then
config_branch=er_flex_00041/map/twist_lock_plug
fi
roscd chargepal_map/config
git checkout ${config_branch}
cd -
exec "$@"
