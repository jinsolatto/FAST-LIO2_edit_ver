#!/bin/bash
# One-off run of bucheon10b.yaml with voxel_ramp_duration widened from 5s to
# 20s (same single segment [260,1000], voxel_size_normal=0.3,
# voxel_size_fine=0.1) against bag 10_b_minimize, to test whether a slower
# coarse->fine transition reduces trajectory instability at the 260s
# boundary compared to the existing 5s-ramp result in
# BUCHEON/bucheon_10b/partially/0.1/. Saves to BUCHEON/bucheon_10b/ramp20/0.1/.

set -e

REPO=/home/jschoi/edit_fastlio2
CONFIG=$REPO/src/FAST_LIO_ROS2/config/bucheon10b.yaml
OUTDIR=$REPO/BUCHEON/bucheon_10b/ramp20/0.1
BAG=/home/jschoi/bag_file/10_b_minimize
DIAG=$REPO/diag_test/bucheon10b_ramp20
mkdir -p "$OUTDIR" "$DIAG"

source /opt/ros/*/setup.bash 2>/dev/null
source "$REPO/install/setup.bash" 2>/dev/null

( while true; do date +%s >> "$DIAG/heartbeat.log"; sleep 1; done ) &
HEARTBEAT_PID=$!
( while true; do nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used,temperature.gpu,power.draw --format=csv,noheader >> "$DIAG/gpu.log" 2>&1; sleep 2; done ) &
GPU_PID=$!
trap 'kill "$HEARTBEAT_PID" "$GPU_PID" 2>/dev/null || true' EXIT

echo "=== bucheon10b ramp20 (fine=0.1) : starting $(date) ==="

"$REPO/run_with_pidstat.sh" "bucheon10b_ramp20_01" bucheon10b.yaml > "/tmp/bucheon10b_ramp20_01_launch.log" 2>&1 &
LAUNCH_BG_PID=$!

until grep -q "준비 완료" "/tmp/bucheon10b_ramp20_01_launch.log" 2>/dev/null; do
    sleep 1
done
echo "[ramp20 0.1] node ready $(date)"

ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/bucheon10b_ramp20_01_bagplay.log" 2>&1
echo "[ramp20 0.1] bag playback finished $(date)"

sleep 2

WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
if [ -n "$WIN_ID" ]; then
    DISPLAY=:1 import -window "$WIN_ID" "$OUTDIR/map.png"
    echo "[ramp20 0.1] map.png saved (window $WIN_ID)"
else
    echo "[ramp20 0.1] WARNING: could not find rviz viewport window for screenshot"
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
    echo "[ramp20 0.1] WARNING: leftover processes still alive after cleanup:"
    echo "$LEFTOVER"
else
    echo "[ramp20 0.1] confirmed: node/rviz/republish/pidstat all dead"
fi

cp "$REPO/cpu_logs/cpu_log_bucheon10b_ramp20_01.txt" "$OUTDIR/cpu_log.txt"
echo "=== bucheon10b ramp20 (fine=0.1) : done $(date) ==="
