# shellcheck shell=bash
source /usr/local/share/mullet-ros/setup.bash  # ROS 2 Jazzy + the base workspace (px4_msgs)

if [[ -f /opt/openvins_ws/install/setup.bash ]]; then
  source /opt/openvins_ws/install/setup.bash
fi
