#!/bin/bash
# One-off: turtleisland 15degree/course3 bag (SMB share), siheung.yaml only
# (voxelslam-export matching, --rate 1.0), saved to
# turtleisland_260915/15degree/course3/siheung.yaml/voxel0.5/

set -eo pipefail

REPO=/home/jschoi/edit_fastlio2
BAG="/run/user/1000/gvfs/smb-share:server=murosnas.local,share=sharefiles/DevDATA/LG_project/bag_file/LG_dataset/2026_dataset/turtleisland/15degree/course3"
OUTDIR="$REPO/turtleisland_260915/15degree/course3/siheung.yaml/voxel0.5"
RUNNAME="15deg_course3_siheung_voxel0.5"

mkdir -p "$OUTDIR"
export FASTLIO_USE_VOXEL_MATCHING=1
export FASTLIO_DEGENERACY_LOG_DIR="$OUTDIR"

LEFTOVER=$(pgrep -af "fastlio_mapping|rviz2 -d.*fastlio|image_transport/republish|pidstat -p|ros2 bag play" 2>/dev/null || true)
if [ -n "$LEFTOVER" ]; then
    echo "ABORT: leftover ROS/fastlio process(es) detected:" >&2
    echo "$LEFTOVER" >&2
    exit 1
fi

"$REPO/run_with_pidstat.sh" "$RUNNAME" siheung.yaml > "/tmp/${RUNNAME}_launch.log" 2>&1 &
LAUNCH_BG_PID=$!

until grep -q "준비 완료" "/tmp/${RUNNAME}_launch.log" 2>/dev/null; do
    kill -0 "$LAUNCH_BG_PID" 2>/dev/null || { echo "ABORT: launcher died before becoming ready" >&2; cat "/tmp/${RUNNAME}_launch.log" >&2; exit 1; }
    sleep 1
done
echo "[15deg/course3/siheung.yaml/voxel0.5] node ready"

ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/${RUNNAME}_bagplay.log" 2>&1
echo "[15deg/course3/siheung.yaml/voxel0.5] bag playback finished"

sleep 2

WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
if [ -n "$WIN_ID" ]; then
    DISPLAY=:1 import -window "$WIN_ID" "$OUTDIR/map.png"
    echo "[15deg/course3/siheung.yaml/voxel0.5] map.png saved (window $WIN_ID)"
else
    echo "[15deg/course3/siheung.yaml/voxel0.5] WARNING: could not find rviz viewport window for screenshot"
fi

python3 "$REPO/scripts/plot_path_topdown.py" "$OUTDIR/trajectory.tum" "$OUTDIR/path.png"
echo "[15deg/course3/siheung.yaml/voxel0.5] path.png saved"

kill -INT "$LAUNCH_BG_PID" 2>/dev/null || true
for _ in $(seq 1 30); do
    pgrep -f "install/fast_lio/lib/fast_lio/fastlio_mapping" >/dev/null 2>&1 || break
    sleep 1
done
pkill -9 -f fastlio_mapping 2>/dev/null || true
pkill -9 -f "rviz2 -d.*fastlio" 2>/dev/null || true
pkill -9 -f "ros2 launch fast_lio" 2>/dev/null || true
pkill -9 -f "pidstat -p" 2>/dev/null || true
pkill -9 -f "image_transport/republish" 2>/dev/null || true
sleep 2

LEFTOVER=$(pgrep -af "fastlio_mapping|rviz2 -d.*fastlio|image_transport/republish|pidstat -p" 2>/dev/null || true)
if [ -n "$LEFTOVER" ]; then
    echo "WARNING: leftover processes still alive after cleanup:" >&2
    echo "$LEFTOVER" >&2
else
    echo "[15deg/course3/siheung.yaml/voxel0.5] confirmed: node/rviz/republish/pidstat all dead"
fi

echo "DONE"
