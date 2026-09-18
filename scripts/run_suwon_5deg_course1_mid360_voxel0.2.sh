#!/bin/bash
# bag_file/5degree/course1, mid360.yaml only (voxelslam-export matching,
# --rate 1.0, filter_size_surf=0.2), saved to
# suwon_260914/5degree/course1_minimize/mid360.yaml/voxel0.2/

set -eo pipefail

REPO=/home/jschoi/edit_fastlio2
BAG=/home/jschoi/bag_file/5degree/course1
OUTDIR="$REPO/suwon_260914/5degree/course1_minimize/mid360.yaml/voxel0.2"
RUNNAME="suwon_5deg_course1_mid360_voxel0.2"
MID360_CFG="$REPO/src/FAST_LIO_ROS2/config/mid360.yaml"

mkdir -p "$OUTDIR"
python3 - "$MID360_CFG" "$OUTDIR/trajectory.tum" <<'PYEOF'
import sys
cfg, newpath = sys.argv[1], sys.argv[2]
with open(cfg) as f:
    lines = f.readlines()
for i, line in enumerate(lines):
    if line.strip().startswith("tum_trajectory_log_path:"):
        indent = line[:len(line) - len(line.lstrip())]
        lines[i] = f'{indent}tum_trajectory_log_path: "{newpath}"\n'
with open(cfg, "w") as f:
    f.writelines(lines)
PYEOF

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
echo "[course1/mid360.yaml/voxel0.2] node ready"

ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/${RUNNAME}_bagplay.log" 2>&1
echo "[course1/mid360.yaml/voxel0.2] bag playback finished"

sleep 2

WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
if [ -n "$WIN_ID" ]; then
    DISPLAY=:1 import -window "$WIN_ID" "$OUTDIR/map.png"
    echo "[course1/mid360.yaml/voxel0.2] map.png saved (window $WIN_ID)"
else
    echo "[course1/mid360.yaml/voxel0.2] WARNING: could not find rviz viewport window for screenshot"
fi

python3 "$REPO/scripts/plot_path_topdown.py" "$OUTDIR/trajectory.tum" "$OUTDIR/path.png"
echo "[course1/mid360.yaml/voxel0.2] path.png saved"

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
    echo "[course1/mid360.yaml/voxel0.2] confirmed: node/rviz/republish/pidstat all dead"
fi

echo "DONE"
