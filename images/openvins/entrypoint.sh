#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/ros2_base_ws/install/setup.bash

if [[ -f /opt/openvins_ws/install/setup.bash ]]; then
  source /opt/openvins_ws/install/setup.bash
else
  echo "OpenVINS overlay is not installed; running the container scaffold." >&2
fi

exec "$@"
