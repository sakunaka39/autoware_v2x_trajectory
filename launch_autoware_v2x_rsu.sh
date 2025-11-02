#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source install/setup.bash
export AWID=1
#ros2 launch autoware_v2x v2x.launch.xml network_interface:=wlo1 &
ros2 launch autoware_v2x_trajectory v2x.launch.xml network_interface:=wlo1 is_sender:=false

#source ./src/v2x/autowarev2x/setup.sh
#sudo bash -c "source /opt/ros/humble/setup.bash && source /autoware_v2x/install/setup.bash && ros2 launch autoware_v2x v2x.launch.xml network_interface:=wlo1"
