# mullet-vio

Container and deployment configuration for ROS 2 visual-inertial odometry on
the Mullet Jetson platform. The current image builds OpenVINS for ROS 2 Jazzy;
other implementations, such as VINS-Fusion, should receive their own image and
workspace.

## Architecture

At build time, each VIO image inherits the shared ROS base:

```text
ghcr.io/skymule-ai/mullet-base:v0.5.1
├── ghcr.io/skymule-ai/mullet-vio-openvins
└── ghcr.io/skymule-ai/mullet-vio-vins-fusion  (future)
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
images/openvins/       OpenVINS source build and runtime entrypoint
config/openvins/       Example runtime configuration mount point
config/realsense/      Example D435i driver configuration (infrared projector off)
docs/openvins.md       How OpenVINS works, our settings, failure modes
compose.yaml           RealSense + selectable VIO services
scripts/validate.sh    Structural checks that do not build images
scripts/smoke-openvins.sh  Camera-free image startup smoke test
.github/workflows/     ARM64 image build and publishing
```

[docs/openvins.md](docs/openvins.md) explains the estimator, its initialisation, what it publishes,
the settings we changed and the failure modes seen on mullet1. Read it before tuning.

Future implementations should be isolated:

```text
images/vins-fusion/
config/vins-fusion/
```

Add a separate Compose service and CI matrix entry rather than installing both
implementations into the same image.

## Build OpenVINS

Create the local environment file:

```bash
cp .env.example .env
```

Validate the Docker and Compose structure without building or installing
anything:

```bash
./scripts/validate.sh
```

The OpenVINS image builds the upstream `rpng/open_vins` repository. Its source
and revision are build arguments:

```dotenv
OPENVINS_REPOSITORY=https://github.com/rpng/open_vins.git
OPENVINS_REF=69488123ed9362dd44b6f28e7f4680abbff1442b
```

`OPENVINS_REF` should be a full commit SHA for reproducible images. To build a
different repository or revision locally, set these values in `.env` before
building.

Build the local image:

```bash
docker compose --profile openvins build openvins
```

Run the camera-free image smoke test after building:

```bash
./scripts/smoke-openvins.sh
```

The smoke test verifies package discovery, executable installation, launch-file
arguments, mounted configuration files, and that OpenVINS remains alive while
waiting for sensor topics. It does not validate calibration or odometry quality.

Changing `OPENVINS_REF` also invalidates the Docker layer that checks out
OpenVINS, ensuring the requested revision is rebuilt.

This command starts both RealSense and OpenVINS. OpenVINS loads
`/config/estimator_config.yaml`; that file refers to its IMU and
camera calibration files in the same directory.

```bash
docker compose --profile openvins up
```

Both services use host networking, the same ROS domain, and the same RMW
implementation. Only the RealSense container receives USB device access.
`depends_on` controls container start order only; the VIO node must tolerate
camera topics becoming available after it starts.

## D435i infrared projector

Keep it off whenever VIO uses the infrared images, as `config/realsense/d435i.yaml` does. The projector's
dot pattern moves with the camera, and OpenVINS tracks the dots as if they were fixed to the scene: drift
and lost tracking indoors and near walls (4.5% drift with it on, 1.4% off, on the same walk). Set
`depth_module.emitter_enabled: 0` (0 off, 1 on, 2 auto). It must be an **integer**: the driver rejects
`false` with only a WARN and leaves the projector on. Check with
`ros2 param get /uav0/d435i depth_module.emitter_enabled` (`Integer value is: 0`). The full guide is in
[mullet-localization](https://github.com/skymule-ai/mullet-localization#d435i-infrared-projector).

## Dependency ownership

Each VIO implementation must declare its ROS and system dependencies in its
own package metadata and image build. Do not add OpenVINS-specific dependencies
to `mullet-base`.

If several perception images later need the same large or custom-built
dependency, introduce a separately versioned `mullet-ros-perception-base`
between `mullet-base` and those images.

## Releases

CI builds for ARM64. Branch pushes publish branch and immutable SHA tags;
release tags such as `v0.1.0` publish matching image tags.

To build and publish a different OpenVINS revision without changing this
repository, use **Actions → Build and Push VIO Images → Run workflow**. Supply
the repository URL as `openvins_repository` and its full commit SHA as
`openvins_ref`. A manually dispatched run logs in to GHCR and publishes the
image just like a push-triggered run.

Normal CI builds use the pinned OpenVINS revision shown above. GitHub Actions
repository variables `OPENVINS_REPOSITORY` and `OPENVINS_REF`, when set,
override those CI defaults.

The initial image name is:

```text
ghcr.io/skymule-ai/mullet-vio-openvins
```

Do not create a release tag until the image smoke test passes in CI and a
hardware test on the target Jetson verifies infrared image and IMU consumption
plus pose or odometry publication.
