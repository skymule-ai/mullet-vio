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

COPY images/openvins/entrypoint.sh /usr/local/bin/mullet-openvins-entrypoint
COPY images/openvins/profile.sh /etc/profile.d/10-mullet-openvins.sh

RUN chmod 0755 /usr/local/bin/mullet-openvins-entrypoint && \
    chmod 0644 /etc/profile.d/10-mullet-openvins.sh

ENTRYPOINT ["/usr/local/bin/mullet-openvins-entrypoint"]
CMD ["bash", "-lc", "exec sleep infinity"]
