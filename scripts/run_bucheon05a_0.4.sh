#!/bin/bash
# One-off run of bucheon05a.yaml as currently configured on disk
# (voxel_size_fine=0.4, output paths pointed at
# BUCHEON/bucheon_05a/partially/0.4/).

set -e

REPO=/home/jschoi/edit_fastlio2
CONFIG=$REPO/src/FAST_LIO_ROS2/config/bucheon05a.yaml
OUTDIR=$REPO/BUCHEON/bucheon_05a/partially/0.4
BAG=/home/jschoi/bag_file/05_a_minimize
DIAG=$REPO/diag_test/run_04
mkdir -p "$OUTDIR" "$DIAG"

source /opt/ros/*/setup.bash 2>/dev/null
source "$REPO/install/setup.bash" 2>/dev/null

( while true; do date +%s >> "$DIAG/heartbeat.log"; sleep 1; done ) &
HEARTBEAT_PID=$!
( while true; do nvidia-smi --query-gpu=timestamp,utilization.gpu,utilization.memory,memory.used,temperature.gpu,power.draw --format=csv,noheader >> "$DIAG/gpu.log" 2>&1; sleep 2; done ) &
GPU_PID=$!
trap 'kill "$HEARTBEAT_PID" "$GPU_PID" 2>/dev/null || true' EXIT

echo "=== bucheon05a (fine=0.4) : starting $(date) ==="

"$REPO/run_with_pidstat.sh" "bucheon05a_04" bucheon05a.yaml > "/tmp/bucheon05a_04_launch.log" 2>&1 &
LAUNCH_BG_PID=$!

until grep -q "준비 완료" "/tmp/bucheon05a_04_launch.log" 2>/dev/null; do
    sleep 1
done
echo "[fine=0.4] node ready $(date)"

ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/bucheon05a_04_bagplay.log" 2>&1
echo "[fine=0.4] bag playback finished $(date)"

sleep 2

WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
if [ -n "$WIN_ID" ]; then
    DISPLAY=:1 import -window "$WIN_ID" "$OUTDIR/map.png"
    echo "[fine=0.4] map.png saved (window $WIN_ID)"
else
    echo "[fine=0.4] WARNING: could not find rviz viewport window for screenshot"
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
    echo "[fine=0.4] WARNING: leftover processes still alive after cleanup:"
    echo "$LEFTOVER"
else
    echo "[fine=0.4] confirmed: node/rviz/republish/pidstat all dead"
fi

cp "$REPO/cpu_logs/cpu_log_bucheon05a_04.txt" "$OUTDIR/cpu_log.txt"
echo "=== bucheon05a (fine=0.4) : done $(date) ==="
