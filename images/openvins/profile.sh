# shellcheck shell=bash
source /opt/ros/humble/setup.bash
source /opt/ros2_base_ws/install/setup.bash

if [[ -f /opt/openvins_ws/install/setup.bash ]]; then
  source /opt/openvins_ws/install/setup.bash
fi
