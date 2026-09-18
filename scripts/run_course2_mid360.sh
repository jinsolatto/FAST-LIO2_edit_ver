#!/bin/bash
# One-off: course2_minimize bag, mid360.yaml only (voxelslam-export
# matching, --rate 1.0), saved to
# turtleisland_260915/10degree/course2_minimize/mid360.yaml/
# (filter_size_surf now unified to 0.5, matching siheung.yaml).

set -eo pipefail

REPO=/home/jschoi/edit_fastlio2
BAG=/home/jschoi/bag_file/10degree_/course2_minimize
OUTDIR="$REPO/turtleisland_260915/10degree/course2_minimize/mid360.yaml"
RUNNAME="course2_mid360_rerun"

mkdir -p "$OUTDIR"
export FASTLIO_USE_VOXEL_MATCHING=1
export FASTLIO_DEGENERACY_LOG_DIR="$OUTDIR"

LEFTOVER=$(pgrep -af "fastlio_mapping|rviz2 -d.*fastlio|image_transport/republish|pidstat -p|ros2 bag play" 2>/dev/null || true)
if [ -n "$LEFTOVER" ]; then
    echo "ABORT: leftover ROS/fastlio process(es) detected:" >&2
    echo "$LEFTOVER" >&2
    exit 1
fi

"$REPO/run_with_pidstat.sh" "$RUNNAME" mid360.yaml > "/tmp/${RUNNAME}_launch.log" 2>&1 &
LAUNCH_BG_PID=$!

until grep -q "준비 완료" "/tmp/${RUNNAME}_launch.log" 2>/dev/null; do
    kill -0 "$LAUNCH_BG_PID" 2>/dev/null || { echo "ABORT: launcher died before becoming ready" >&2; cat "/tmp/${RUNNAME}_launch.log" >&2; exit 1; }
    sleep 1
done
echo "[course2/mid360.yaml] node ready"

ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/${RUNNAME}_bagplay.log" 2>&1
echo "[course2/mid360.yaml] bag playback finished"

sleep 2

WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
if [ -n "$WIN_ID" ]; then
    DISPLAY=:1 import -window "$WIN_ID" "$OUTDIR/map.png"
    echo "[course2/mid360.yaml] map.png saved (window $WIN_ID)"
else
    echo "[course2/mid360.yaml] WARNING: could not find rviz viewport window for screenshot"
fi

python3 "$REPO/scripts/plot_path_topdown.py" "$OUTDIR/trajectory.tum" "$OUTDIR/path.png"
echo "[course2/mid360.yaml] path.png saved"

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
    echo "[course2/mid360.yaml] confirmed: node/rviz/republish/pidstat all dead"
fi

echo "DONE"
