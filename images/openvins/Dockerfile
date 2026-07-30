ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-ros2-base:v0.4.0
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

LABEL org.opencontainers.image.title="mullet ROS OpenVINS"
LABEL org.opencontainers.image.description="OpenVINS container scaffold for the Mullet Jetson platform"

# OpenVINS is intentionally not installed here yet.
#
# Integration contract for the implementation owner:
#   1. Pin the OpenVINS source revision.
#   2. Declare and install its dependencies in this image, not in ros2-base.
#   3. Build its ROS 2 packages under /opt/openvins_ws.
#   4. Leave /opt/openvins_ws/install/setup.bash for the entrypoint to source.
#   5. Replace the scaffold command with the validated ROS 2 launch command.

RUN apt-get update && apt-get install -y --no-install-recommends \
  libeigen3-dev libboost-all-dev libceres-dev \
  libopencv-dev python3-opencv \
  python3-dev python3-matplotlib python3-numpy \
  python3-psutil python3-tk \
  ros-humble-cv-bridge ros-humble-image-transport \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/openvins_ws/src/ \
  && cd /opt/openvins_ws/src/ \
  && git clone https://github.com/rpng/open_vins/

 RUN source /opt/ros/humble/setup.bash \
&& cd /opt/openvins_ws/ \
&& colcon build --event-handlers console_cohesion+ \
--packages-select ov_core ov_init ov_eval \
--parallel-workers 1 \
--cmake-args -DCMAKE_BUILD_TYPE=Release

RUN source /opt/ros/humble/setup.bash \
  && cd /opt/openvins_ws/ \
  && colcon build \
    --packages-select ov_msckf \
    --parallel-workers 1 \
    --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O0" \
  || (cd /opt/openvins_ws/build/ov_msckf && make -j1 CXXFLAGS="-O0")

COPY images/openvins/entrypoint.sh /usr/local/bin/mullet-openvins-entrypoint
COPY images/openvins/profile.sh /etc/profile.d/10-mullet-openvins.sh

RUN chmod 0755 /usr/local/bin/mullet-openvins-entrypoint && \
    chmod 0644 /etc/profile.d/10-mullet-openvins.sh

ENTRYPOINT ["/usr/local/bin/mullet-openvins-entrypoint"]
CMD ["bash", "-lc", "exec sleep infinity"]
