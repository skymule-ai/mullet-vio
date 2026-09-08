#!/usr/bin/env bash
set -euo pipefail

image="${OPENVINS_IMAGE:-ghcr.io/skymule-ai/mullet-ros-vio-openvins:local}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config_dir="${repo_root}/config/openvins"
container_name="mullet-openvins-smoke-${RANDOM}-$$"

cleanup() {
  docker rm --force "${container_name}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker image inspect "${image}" >/dev/null

docker run --rm "${image}" ros2 pkg prefix ov_msckf >/dev/null
docker run --rm "${image}" bash -c \
  'ros2 pkg executables ov_msckf | grep -q "ov_msckf run_subscribe_msckf"'

docker run --rm \
  --volume "${config_dir}:/config:ro" \
  "${image}" \
  bash -c '
    test -r /config/mullet_vio/estimator_config.yaml
    test -r /config/mullet_vio/kalibr_imu_chain.yaml
    test -r /config/mullet_vio/kalibr_imucam_chain.yaml
    ros2 launch ov_msckf subscribe.launch.py --show-args \
      | grep -q "config_path"
  '

docker run --detach \
  --name "${container_name}" \
  --network host \
  --volume "${config_dir}:/config:ro" \
  "${image}" >/dev/null

sleep 5

if [[ "$(docker inspect --format '{{.State.Running}}' "${container_name}")" != "true" ]]; then
  docker logs "${container_name}" >&2
  echo "OpenVINS exited instead of waiting for sensor topics." >&2
  exit 1
fi

echo "OpenVINS image smoke test passed."
