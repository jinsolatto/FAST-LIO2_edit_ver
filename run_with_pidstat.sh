#!/bin/bash
# run_with_pidstat.sh
#
# Launch fast_lio and log fastlio_mapping's CPU/mem usage (via pidstat) to a
# named file, so each experiment run gets its own log without hunting for
# the PID by hand.
#
# Usage:
#   ./run_with_pidstat.sh <experiment_name> [config_file] [interval_sec]
#
# Examples:
#   ./run_with_pidstat.sh baseline
#   ./run_with_pidstat.sh deskew_off mid360_imu.yaml
#   ./run_with_pidstat.sh deskew_off mid360_imu.yaml 0.5
#
# Then in another terminal: ros2 bag play <your_bag>
# Stop with Ctrl+C here when the run is done — launch and pidstat both get
# cleaned up.

LOGNAME="${1:?usage: $0 <experiment_name> [config_file] [interval_sec]}"
CONFIG_FILE="${2:-mid360.yaml}"
INTERVAL="${3:-1}"

if [[ "$LOGNAME" == */* ]]; then
    echo "error: <experiment_name> ('$LOGNAME') looks like a path, not a name." >&2
    echo "usage: $0 <experiment_name> [config_file] [interval_sec]" >&2
    echo "  e.g. $0 baseline mid360.yaml" >&2
    exit 1
fi

REPO_ROOT="/home/jschoi/edit_fastlio2"
CONFIG_DIR="$REPO_ROOT/src/FAST_LIO_ROS2/config"
LOG_DIR="$REPO_ROOT/cpu_logs"
mkdir -p "$LOG_DIR"

# accept either a bare filename (resolved against CONFIG_DIR) or a full path
if [[ "$CONFIG_FILE" == /* ]]; then
    CONFIG_PATH="$CONFIG_FILE"
else
    CONFIG_PATH="$CONFIG_DIR/$CONFIG_FILE"
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
    echo "config file not found: $CONFIG_PATH" >&2
    exit 1
fi

LOG_FILE="$LOG_DIR/cpu_log_${LOGNAME}.txt"

echo "config: $CONFIG_PATH"
echo "log:    $LOG_FILE"

source /opt/ros/*/setup.bash 2>/dev/null
source "$REPO_ROOT/install/setup.bash" 2>/dev/null

ros2 launch fast_lio mapping.launch.py config_file:="$CONFIG_PATH" use_sim_time:=true &
LAUNCH_PID=$!

# wait for the actual fastlio_mapping process to appear (up to ~15s)
PID=""
for _ in $(seq 1 30); do
    PID=$(pgrep -f "$REPO_ROOT/install/fast_lio/lib/fast_lio/fastlio_mapping" | head -1)
    [[ -n "$PID" ]] && break
    sleep 0.5
done

if [[ -z "$PID" ]]; then
    echo "fastlio_mapping never showed up — killing launch and aborting." >&2
    kill "$LAUNCH_PID" 2>/dev/null
    exit 1
fi

echo "Node PID: $PID"

pidstat -p "$PID" -u -r "$INTERVAL" -h > "$LOG_FILE" &
PIDSTAT_PID=$!

cleanup() {
    echo "stopping..."
    kill -INT "$LAUNCH_PID" 2>/dev/null
    # ros2 launch doesn't always cascade SIGTERM/SIGINT to its children
    # reliably, so poll for the real target process instead of blocking
    # on `wait` forever, then force-kill anything still alive.
    for _ in $(seq 1 15); do
        pgrep -f "$REPO_ROOT/install/fast_lio/lib/fast_lio/fastlio_mapping" >/dev/null 2>&1 || break
        sleep 1
    done
    kill "$PIDSTAT_PID" 2>/dev/null
    pkill -9 -f fastlio_mapping 2>/dev/null
    pkill -9 -f "rviz2 -d.*fastlio.rviz" 2>/dev/null
    pkill -9 -f "image_transport/republish" 2>/dev/null
    pkill -9 -f "ros2 launch fast_lio" 2>/dev/null
    pkill -9 -f "pidstat -p" 2>/dev/null
    echo "log saved: $LOG_FILE"
}
trap cleanup EXIT INT TERM

echo "준비 완료. 다른 터미널에서 bag을 재생하세요. Ctrl+C로 종료."
wait "$LAUNCH_PID"
