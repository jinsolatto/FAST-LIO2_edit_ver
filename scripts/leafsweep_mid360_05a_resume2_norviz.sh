#!/bin/bash
# Resume of leafsweep_mid360_05a: rviz2 disabled (suspected cause of the
# system-freeze crashes hit at leaf=0.3 on 2026-09-02), heartbeat + GPU
# telemetry logged continuously across the whole sweep so any future freeze
# leaves hard evidence of exactly when it happened.
# Redoes 0.3 (previous attempt was cut short mid-bag, csv/tum incomplete),
# then 0.2, 0.1.

set -e

REPO=/home/jschoi/edit_fastlio2
CONFIG=$REPO/src/FAST_LIO_ROS2/config/mid360.yaml
OUTROOT=$REPO/BUCHEON/bucheon_05a_total
BAG=/home/jschoi/bag_file/05_a_minimize
VALUES=(0.3 0.2 0.1)
DIAG=$REPO/diag_test/resume2

mkdir -p "$DIAG"

cp "$CONFIG" /tmp/mid360_orig_backup_05a_r2.yaml
restore_config() {
    cp /tmp/mid360_orig_backup_05a_r2.yaml "$CONFIG"
    echo "[restore] mid360.yaml restored to original pure state"
}
trap restore_config EXIT

source /opt/ros/*/setup.bash 2>/dev/null
source "$REPO/install/setup.bash" 2>/dev/null

# continuous heartbeat + GPU telemetry for the whole sweep
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

    source /opt/ros/*/setup.bash 2>/dev/null
    source "$REPO/install/setup.bash" 2>/dev/null
    ros2 launch fast_lio mapping.launch.py config_file:="$CONFIG" use_sim_time:=true rviz:=false > "/tmp/leafsweep05a_r2_${v}_launch.log" 2>&1 &
    LAUNCH_BG_PID=$!

    PID=""
    for _ in $(seq 1 30); do
        PID=$(pgrep -f "$REPO/install/fast_lio/lib/fast_lio/fastlio_mapping" | head -1)
        [[ -n "$PID" ]] && break
        sleep 0.5
    done
    if [[ -z "$PID" ]]; then
        echo "[leaf=$v] ERROR: fastlio_mapping never came up, aborting sweep"
        exit 1
    fi
    echo "[leaf=$v] node ready (pid=$PID)"

    pidstat -p "$PID" -u -r 1 -h > "$REPO/cpu_logs/cpu_log_leafsweep05a_${v}.txt" 2>&1 &
    PIDSTAT_PID=$!

    ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/leafsweep05a_r2_${v}_bagplay.log" 2>&1
    echo "[leaf=$v] bag playback finished $(date)"

    python3 "$REPO/scripts/plot_path_topdown.py" "$OUTDIR/FAST-LIO2.tum" "$OUTDIR/path.png"

    kill -INT "$LAUNCH_BG_PID" 2>/dev/null || true
    for i in $(seq 1 15); do
        pgrep -f "install/fast_lio/lib/fast_lio/fastlio_mapping" >/dev/null 2>&1 || break
        sleep 1
    done
    kill "$PIDSTAT_PID" 2>/dev/null || true
    pkill -9 -f fastlio_mapping 2>/dev/null || true
    pkill -9 -f "ros2 launch fast_lio" 2>/dev/null || true
    pkill -9 -f "image_transport/republish" 2>/dev/null || true
    pkill -9 -f "pidstat -p" 2>/dev/null || true
    sleep 2

    LEFTOVER=$(pgrep -af "fastlio_mapping|image_transport/republish|pidstat -p" 2>/dev/null || true)
    if [ -n "$LEFTOVER" ]; then
        echo "[leaf=$v] WARNING: leftover processes still alive after cleanup:"
        echo "$LEFTOVER"
    else
        echo "[leaf=$v] confirmed: node/republish/pidstat all dead"
    fi

    cp "$REPO/cpu_logs/cpu_log_leafsweep05a_${v}.txt" "$OUTDIR/cpu_log.txt"
    echo "=== leaf=$v : done $(date) ==="
done

echo "ALL LEAF SWEEP RUNS COMPLETE (no-rviz resume2) $(date)"
