# OpenVINS image handoff

This directory currently defines only the container boundary. It does not
download, install, or build OpenVINS or any of its dependencies.

The implementation owner should extend `Dockerfile` while preserving these
contracts:

- Inherit an immutable `mullet-ros2-base` release through `BASE_IMAGE`.
- Pin all upstream source revisions.
- Keep OpenVINS and implementation-specific dependencies outside the shared
  ROS base image.
- Build the ROS overlay at `/opt/openvins_ws`.
- Ensure `/opt/openvins_ws/install/setup.bash` exists.
- Consume standard ROS image, camera-info, and IMU topics where possible.
- Do not add RealSense USB access to the OpenVINS service.
- Add a smoke test that verifies package discovery and launch-file parsing.
- Add a Jetson hardware test that verifies image/IMU consumption and odometry
  publication before creating a release tag.

RealSense remains a separate runtime container selected by `compose.yaml`.
