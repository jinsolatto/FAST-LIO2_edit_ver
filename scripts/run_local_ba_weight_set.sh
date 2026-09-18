#!/usr/bin/env bash
# Reproducible Step-2A IMU-weight run. Play the same bag separately for A/B/C.
# Usage: ./scripts/run_local_ba_weight_set.sh A [mid360.yaml]
set -euo pipefail

WEIGHT_SET="${1:-}"
CONFIG_FILE="${2:-mid360.yaml}"
case "$WEIGHT_SET" in A|B|C) ;; *) echo "usage: $0 {A|B|C} [config_file]" >&2; exit 2;; esac

REPO_ROOT="/home/jschoi/edit_fastlio2"
CONFIG_PATH="$REPO_ROOT/src/FAST_LIO_ROS2/config/$CONFIG_FILE"
[[ "$CONFIG_FILE" == /* ]] && CONFIG_PATH="$CONFIG_FILE"
[[ -f "$CONFIG_PATH" ]] || { echo "config file not found: $CONFIG_PATH" >&2; exit 1; }

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
echo "Step-2A $WEIGHT_SET: map/EKF feedback forced off; CSV: FAST-LIO2_local_ba_${WEIGHT_SET}.csv"
ros2 launch fast_lio mapping.launch.py config_file:="$CONFIG_PATH" use_sim_time:=true \
    local_ba_weight_set:="$WEIGHT_SET"
