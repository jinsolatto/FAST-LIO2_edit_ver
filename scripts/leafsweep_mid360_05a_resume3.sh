#!/bin/bash
# Resume of leafsweep_mid360_05a after the 2026-09-02 freeze at leaf=0.3.
# rviz2 stays ON (as before) — CloudMap display is already saved OFF
# (Value: false, Decay Time: 1000 kept as-is) in fastlio.rviz, so each
# fresh rviz2 launch starts with it off automatically. Heartbeat + GPU
# telemetry logged continuously across the whole sweep so any future
# freeze leaves hard evidence of exactly when/where it happened.
# Redoes 0.3 (previous attempt was cut short mid-bag), then 0.2, 0.1.

set -e

REPO=/home/jschoi/edit_fastlio2
CONFIG=$REPO/src/FAST_LIO_ROS2/config/mid360.yaml
OUTROOT=$REPO/BUCHEON/bucheon_05a_total
BAG=/home/jschoi/bag_file/05_a_minimize
VALUES=(0.3 0.2 0.1)
DIAG=$REPO/diag_test/resume3
mkdir -p "$DIAG"

cp "$CONFIG" /tmp/mid360_orig_backup_05a_r3.yaml
restore_config() {
    cp /tmp/mid360_orig_backup_05a_r3.yaml "$CONFIG"
    echo "[restore] mid360.yaml restored to original pure state"
}

source /opt/ros/*/setup.bash 2>/dev/null
source "$REPO/install/setup.bash" 2>/dev/null

( while true; do date +%s >> "$DIAG/heartbeat.log"; sleep 1; done ) &
HEARTBEAT_PID=$!
( while true; do nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used,temperature.gpu,power.draw --format=csv,noheader >> "$DIAG/gpu.log" 2>&1; sleep 2; done ) &
GPU_PID=$!
cleanup_monitors() {
    kill "$HEARTBEAT_PID" "$GPU_PID" 2>/dev/null || true
}
trap 'cleanup_monitors; restore_config' EXIT

for v in "${VALUES[@]}"; do
    echo "=== leaf=$v : starting $(date) ==="
    OUTDIR="$OUTROOT/$v"
    mkdir -p "$OUTDIR"

    python3 - "$CONFIG" "$v" "$OUTDIR" <<'PYEOF'
import sys
cfg, v, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
with open(cfg) as f:
    lines = f.readlines()
for i, line in enumerate(lines):
    if line.strip().startswith("filter_size_surf:"):
        lines[i] = f"        filter_size_surf: {v} #0.5\n"
    elif line.strip().startswith("tum_trajectory_log_path:"):
        lines[i] = f'            tum_trajectory_log_path: "{outdir}/FAST-LIO2.tum"\n'
with open(cfg, "w") as f:
    f.writelines(lines)
PYEOF

    "$REPO/run_with_pidstat.sh" "leafsweep05a_${v}" mid360.yaml > "/tmp/leafsweep05a_r3_${v}_launch.log" 2>&1 &
    LAUNCH_BG_PID=$!

    until grep -q "준비 완료" "/tmp/leafsweep05a_r3_${v}_launch.log" 2>/dev/null; do
        sleep 1
    done
    echo "[leaf=$v] node ready $(date)"

    ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/leafsweep05a_r3_${v}_bagplay.log" 2>&1
    echo "[leaf=$v] bag playback finished $(date)"

    sleep 2

    WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
    if [ -n "$WIN_ID" ]; then
        DISPLAY=:1 import -window "$WIN_ID" "$OUTDIR/map.png"
        echo "[leaf=$v] map.png saved (window $WIN_ID)"
    else
        echo "[leaf=$v] WARNING: could not find rviz viewport window for screenshot"
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
        echo "[leaf=$v] WARNING: leftover processes still alive after cleanup:"
        echo "$LEFTOVER"
    else
        echo "[leaf=$v] confirmed: node/rviz/republish/pidstat all dead"
    fi

    cp "$REPO/cpu_logs/cpu_log_leafsweep05a_${v}.txt" "$OUTDIR/cpu_log.txt"
    echo "=== leaf=$v : done $(date) ==="
done

echo "ALL LEAF SWEEP RUNS COMPLETE (resume3) $(date)"
