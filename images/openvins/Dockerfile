ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-ros2-base:v0.5.0
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Pin OPENVINS_REF to a full commit SHA for reproducible production images.
ARG OPENVINS_REPOSITORY=https://github.com/rpng/open_vins.git
ARG OPENVINS_REF=69488123ed9362dd44b6f28e7f4680abbff1442b

LABEL org.opencontainers.image.title="mullet ROS OpenVINS"
LABEL org.opencontainers.image.description="ROS 2 OpenVINS runtime for the Mullet Jetson platform"

RUN apt-get update && apt-get install -y --no-install-recommends \
  libeigen3-dev libboost-all-dev libceres-dev \
  libopencv-dev python3-opencv \
  python3-dev python3-matplotlib python3-numpy \
  python3-psutil python3-tk \
  ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/openvins_ws/src/ \
  && cd /opt/openvins_ws/src/ \
  && git clone "${OPENVINS_REPOSITORY}" open_vins \
  && cd open_vins \
  && git checkout "${OPENVINS_REF}" \
  # Jazzy ships only the .hpp headers of these three; Humble has both.
  && grep -rlE '(cv_bridge/cv_bridge|image_transport/image_transport|tf2_geometry_msgs/tf2_geometry_msgs)\.h[>"]' . \
     | xargs -r sed -i -E 's#(cv_bridge/cv_bridge|image_transport/image_transport|tf2_geometry_msgs/tf2_geometry_msgs)\.h([>"])#\1.hpp\2#'

RUN source /usr/local/share/mullet-ros/setup.bash \
  && cd /opt/openvins_ws/ \
  && CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
    --merge-install \
    --executor sequential \
    --packages-up-to ov_msckf \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && source install/setup.bash \
  && ros2 pkg prefix ov_msckf \
  && rm -rf build log

COPY images/openvins/entrypoint.sh /usr/local/bin/mullet-openvins-entrypoint
COPY images/openvins/profile.sh /etc/profile.d/10-mullet-openvins.sh

RUN chmod 0755 /usr/local/bin/mullet-openvins-entrypoint && \
    chmod 0644 /etc/profile.d/10-mullet-openvins.sh

ENTRYPOINT ["/usr/local/bin/mullet-openvins-entrypoint"]
CMD ["ros2", "launch", "ov_msckf", "subscribe.launch.py", "config_path:=/config/estimator_config.yaml", "namespace:=uav0/openvins", "rviz_enable:=false"]
