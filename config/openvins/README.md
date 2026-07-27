# OpenVINS runtime configuration

This directory is mounted read-only at `/config` in the OpenVINS container.

The OpenVINS integrator should add example/default configuration here after
selecting the camera model, ROS launch interface, topic remappings, and
calibration-file format.

Aircraft-specific camera/IMU intrinsics, extrinsics, time offsets, and serial
numbers must live in the deployment configuration repository, not in this
reusable image repository.
