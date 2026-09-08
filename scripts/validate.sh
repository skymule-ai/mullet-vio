#!/usr/bin/env bash
set -euo pipefail

docker compose \
  --env-file .env.example \
  --profile openvins \
  config \
  --quiet

bash -n \
  images/openvins/entrypoint.sh \
  images/openvins/profile.sh \
  scripts/smoke-openvins.sh

test -f config/openvins/mullet_vio/estimator_config.yaml
test -f config/openvins/mullet_vio/kalibr_imu_chain.yaml
test -f config/openvins/mullet_vio/kalibr_imucam_chain.yaml
