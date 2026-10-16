#!/usr/bin/env bash
set -e

source /usr/local/share/mullet-ros/setup.bash  # ROS 2 Jazzy + the base workspace (px4_msgs)

if [[ -f /opt/openvins_ws/install/setup.bash ]]; then
  source /opt/openvins_ws/install/setup.bash
else
  echo "OpenVINS overlay is not installed; running the container scaffold." >&2
fi

exec "$@"
