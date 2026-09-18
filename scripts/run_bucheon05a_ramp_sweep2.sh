#!/bin/bash
# Sweep of bucheon05a.yaml's voxel_size_fine over VALUES, using the current
# two-segment "total" config (dip to fine during [110,140]s and [305,425]s,
# ramp in/out over voxel_ramp_duration, normal (0.5) elsewhere). Saves
# cpu_log, csv, tum, map.png, path.png per value into BUCHEON/bucheon_05a/<v>/,
# matching the path convention already set in the file for 0.1. Restores
# bucheon05a.yaml to its current on-disk state when done, regardless of
# success/failure.

set -e

REPO=/home/jschoi/edit_fastlio2
CONFIG=$REPO/src/FAST_LIO_ROS2/config/bucheon05a.yaml
BAG=/home/jschoi/bag_file/05_a_minimize
VALUES=(0.1 0.2 0.3)
DIAG=$REPO/diag_test/ramp_sweep2
mkdir -p "$DIAG"

cp "$CONFIG" /tmp/bucheon05a_orig_backup2.yaml
restore_config() {
    cp /tmp/bucheon05a_orig_backup2.yaml "$CONFIG"
    echo "[restore] bucheon05a.yaml restored to its pre-sweep state"
}

source /opt/ros/*/setup.bash 2>/dev/null
source "$REPO/install/setup.bash" 2>/dev/null

( while true; do date +%s >> "$DIAG/heartbeat.log"; sleep 1; done ) &
HEARTBEAT_PID=$!
( while true; do nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used,temperature.gpu,power.draw --format=csv,noheader >> "$DIAG/gpu.log" 2>&1; sleep 2; done ) &
GPU_PID=$!
trap 'kill "$HEARTBEAT_PID" "$GPU_PID" 2>/dev/null || true; restore_config' EXIT

for v in "${VALUES[@]}"; do
    OUTDIR="$REPO/BUCHEON/bucheon_05a/${v}"
    mkdir -p "$OUTDIR"
    echo "=== fine=$v : starting $(date) ==="

    python3 - "$CONFIG" "$v" "$OUTDIR" <<'PYEOF'
import sys
cfg, v, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
with open(cfg) as f:
    lines = f.readlines()
for i, line in enumerate(lines):
    if line.strip().startswith("voxel_size_fine:") and "enable" not in line:
        lines[i] = f"            voxel_size_fine: {v}\n"
    elif line.strip().startswith("tum_trajectory_log_path:"):
        lines[i] = f'            tum_trajectory_log_path: "{outdir}/FAST-LIO2.tum"\n'
with open(cfg, "w") as f:
    f.writelines(lines)
PYEOF

    TAG=$(echo "$v" | tr -d '.')
    "$REPO/run_with_pidstat.sh" "bucheon05a_${TAG}" bucheon05a.yaml > "/tmp/bucheon05a_sweep2_${TAG}_launch.log" 2>&1 &
    LAUNCH_BG_PID=$!

    until grep -q "준비 완료" "/tmp/bucheon05a_sweep2_${TAG}_launch.log" 2>/dev/null; do
        sleep 1
    done
    echo "[fine=$v] node ready $(date)"

    ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/bucheon05a_sweep2_${TAG}_bagplay.log" 2>&1
    echo "[fine=$v] bag playback finished $(date)"

    sleep 2

    WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
    if [ -n "$WIN_ID" ]; then
        DISPLAY=:1 import -window "$WIN_ID" "$OUTDIR/map.png"
        echo "[fine=$v] map.png saved (window $WIN_ID)"
    else
        echo "[fine=$v] WARNING: could not find rviz viewport window for screenshot"
    fi

    python3 "$REPO/scripts/plot_path_topdown.py" "$OUTDIR/FAST-LIO2.tum" "$OUTDIR/path.png"

    kill -INT "$LAUNCH_BG_PID" 2>/dev/null || true
    for i in $(seq 1 15); do
        pgrep -f "install/fast_lio/lib/fast_lio/fastlio_mapping" >/dev/null 2>&1 || break
        sleep 1
    done
    pkill -9 -f fastlio_mapping 2>/dev/null || true
    pkill -9 -f "rviz2 -d.*fastlio.rviz" 2>/dev/null || true
    pkill -9 -f "ros2 launch fast_lio" 2>/dev/null || true
    pkill -9 -f "pidstat -p" 2>/dev/null || true
    pkill -9 -f "image_transport/republish" 2>/dev/null || true
    sleep 2

    LEFTOVER=$(pgrep -af "fastlio_mapping|rviz2 -d.*fastlio.rviz|image_transport/republish|pidstat -p" 2>/dev/null || true)
    if [ -n "$LEFTOVER" ]; then
        echo "[fine=$v] WARNING: leftover processes still alive after cleanup:"
        echo "$LEFTOVER"
    else
        echo "[fine=$v] confirmed: node/rviz/republish/pidstat all dead"
    fi

    cp "$REPO/cpu_logs/cpu_log_bucheon05a_${TAG}.txt" "$OUTDIR/cpu_log.txt"
    echo "=== fine=$v : done $(date) ==="
done

echo "ALL RAMP-FINE-VALUE RUNS COMPLETE (sweep2) $(date)"
