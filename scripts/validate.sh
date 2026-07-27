#!/usr/bin/env bash
set -euo pipefail

docker compose \
  --env-file .env.example \
  --profile openvins \
  config \
  --quiet

bash -n \
  images/openvins/entrypoint.sh \
  images/openvins/profile.sh
