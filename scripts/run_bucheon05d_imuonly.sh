#!/bin/bash
# Test of the new map_update_skip_* feature: skip map_incremental() during the
# time windows where the robot was physically lifted while replaying
# 05_d_minimize (305-310s, 327-337s, 370-373s), to see whether the resulting
# trajectory stays stable instead of diverging as it did without the skip.

set -e

REPO=/home/jschoi/edit_fastlio2
CONFIG=$REPO/src/FAST_LIO_ROS2/config/bucheon05d_imuonly.yaml
OUTDIR=$REPO/BUCHEON/bucheon_05d_imuonly
BAG=/home/jschoi/bag_file/05_d_minimize
DIAG=$REPO/diag_test/bucheon05d_imuonly
mkdir -p "$OUTDIR" "$DIAG"

source /opt/ros/*/setup.bash 2>/dev/null
source "$REPO/install/setup.bash" 2>/dev/null

( while true; do date +%s >> "$DIAG/heartbeat.log"; sleep 1; done ) &
HEARTBEAT_PID=$!
trap 'kill "$HEARTBEAT_PID" 2>/dev/null || true' EXIT

echo "=== bucheon05d_imuonly : starting $(date) ==="

"$REPO/run_with_pidstat.sh" "bucheon05d_imuonly" bucheon05d_imuonly.yaml > "/tmp/bucheon05d_imuonly_launch.log" 2>&1 &
LAUNCH_BG_PID=$!

until grep -q "준비 완료" "/tmp/bucheon05d_imuonly_launch.log" 2>/dev/null; do
    sleep 1
done
echo "[imuonly] node ready $(date)"

ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/bucheon05d_imuonly_bagplay.log" 2>&1
echo "[imuonly] bag playback finished $(date)"

sleep 2

WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
if [ -n "$WIN_ID" ]; then
    DISPLAY=:1 import -window "$WIN_ID" "$OUTDIR/map.png"
    echo "[imuonly] map.png saved (window $WIN_ID)"
else
    echo "[imuonly] WARNING: could not find rviz viewport window for screenshot"
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
    echo "[imuonly] WARNING: leftover processes still alive after cleanup:"
    echo "$LEFTOVER"
else
    echo "[imuonly] confirmed: node/rviz/republish/pidstat all dead"
fi

cp "$REPO/cpu_logs/cpu_log_bucheon05d_imuonly.txt" "$OUTDIR/cpu_log.txt"
echo "=== bucheon05d_imuonly : done $(date) ==="
