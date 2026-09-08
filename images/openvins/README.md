# OpenVINS image

The Dockerfile builds the pinned upstream OpenVINS revision in
`/opt/openvins_ws` and installs the `ov_msckf` estimator and its dependencies.
The image starts `subscribe.launch.py` by default and reads the mounted
`/config/mullet_vio/estimator_config.yaml` configuration.

Preserve these integration contracts:

- Inherit an immutable `mullet-ros2-base` release through `BASE_IMAGE`.
- Pin all upstream source revisions.
- Keep OpenVINS and implementation-specific dependencies outside the shared
  ROS base image.
- Build and verify the ROS overlay at `/opt/openvins_ws`.
- Consume standard ROS image, camera-info, and IMU topics where possible.
- Do not add RealSense USB access to the OpenVINS service.
- Keep camera and IMU calibration outside the image and mount it read-only.
- Run `scripts/smoke-openvins.sh` before publishing an image.
- Verify image/IMU consumption and odometry publication on Jetson hardware
  before creating a release tag.

RealSense remains a separate runtime container selected by `compose.yaml`.
