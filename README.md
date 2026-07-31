# mullet_ros_vio

Container and deployment scaffolding for ROS 2 visual-inertial odometry on the
Mullet Jetson platform.

No VIO implementation or third-party dependency is installed by the current
scaffold. OpenVINS is the first planned implementation; other implementations,
such as VINS-Fusion, should receive their own image and workspace.

## Architecture

At build time, each VIO image inherits the shared ROS base:

```text
ghcr.io/skymule-ai/mullet-ros2-base:v0.4.0
├── ghcr.io/skymule-ai/mullet-ros-vio-openvins
└── ghcr.io/skymule-ai/mullet-ros-vio-vins-fusion  (future)
```

At runtime, Compose runs the RealSense driver and selected VIO implementation
as separate containers:

```text
D435i -> RealSense container -> ROS 2 image/IMU topics -> OpenVINS container
```

The VIO image does not inherit from the RealSense image. Docker images have one
parent, and VIO needs only the RealSense ROS topic contract. Compose supplies
the runtime relationship.

## Layout

```text
images/openvins/       OpenVINS image shell and integration contract
config/openvins/       Runtime configuration mount point
compose.yaml           RealSense + selectable VIO services
scripts/validate.sh    Structural checks that do not build images
.github/workflows/     ARM64 image build and publishing
```

Future implementations should be isolated:

```text
images/vins-fusion/
config/vins-fusion/
```

Add a separate Compose service and CI matrix entry rather than installing both
implementations into the same image.

## Current scaffold

Create the local environment file:

```bash
cp .env.example .env
```

Validate the Docker and Compose structure without building or installing
anything:

```bash
./scripts/validate.sh
```

The current OpenVINS container deliberately runs `sleep infinity`

Build the local image:

```bash
docker compose --profile openvins build openvins
```

This command will start both RealSense and OpenVINS:

```bash
docker compose --profile openvins up
```

Both services use host networking, the same ROS domain, and the same RMW
implementation. Only the RealSense container receives USB device access.
`depends_on` controls container start order only; the VIO node must tolerate
camera topics becoming available after it starts.

## Dependency ownership

Each VIO implementation must declare its ROS and system dependencies in its
own package metadata and image build. Do not add OpenVINS-specific dependencies
to `mullet-ros2-base`.

If several perception images later need the same large or custom-built
dependency, introduce a separately versioned `mullet-ros-perception-base`
between `mullet-ros2-base` and those images.

## Releases

CI builds for ARM64. Branch pushes publish branch and immutable SHA tags;
release tags such as `v0.1.0` publish matching image tags.

The initial image name is:

```text
ghcr.io/skymule-ai/mullet-ros-vio-openvins
```

Do not create a release tag until OpenVINS and its runtime smoke tests have been
implemented and validated on the target Jetson.
