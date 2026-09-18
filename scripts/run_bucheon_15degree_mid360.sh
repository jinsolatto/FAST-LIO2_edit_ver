#!/bin/bash
# bucheon_260914 15degree batch: 15_a -> 15_b, mid360.yaml only
# (voxelslam-export matching, --rate 1.5), saved to
# bucheon_260914/15degree/<bag>/mid360.yaml/
#
# Strictly sequential with a leftover-process check before each run, per
# the project rule against ever running two bag-play/node processes on the
# same ROS topics concurrently.

set -eo pipefail

REPO=/home/jschoi/edit_fastlio2
BAGROOT=/home/jschoi/bag_file
OUTROOT="$REPO/bucheon_260914/15degree"
MID360_CFG="$REPO/src/FAST_LIO_ROS2/config/mid360.yaml"

export FASTLIO_USE_VOXEL_MATCHING=1

set_tum_path() {
    local cfg="$1" newpath="$2"
    python3 - "$cfg" "$newpath" <<'PYEOF'
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
}

run_bag() {
    local bagname="$1"
    local bag="$BAGROOT/$bagname"
    local outdir="$OUTROOT/$bagname/mid360.yaml"
    local runname="bucheon_15deg_${bagname}_mid360"

    echo "=== $bagname : starting ==="
    mkdir -p "$outdir"
    set_tum_path "$MID360_CFG" "$outdir/trajectory.tum"

    LEFTOVER=$(pgrep -af "fastlio_mapping|rviz2 -d.*fastlio|image_transport/republish|pidstat -p|ros2 bag play" 2>/dev/null || true)
    if [ -n "$LEFTOVER" ]; then
        echo "ABORT: leftover ROS/fastlio process(es) detected before starting $bagname:" >&2
        echo "$LEFTOVER" >&2
        exit 1
    fi

    export FASTLIO_DEGENERACY_LOG_DIR="$outdir"

    "$REPO/run_with_pidstat.sh" "$runname" mid360.yaml > "/tmp/${runname}_launch.log" 2>&1 &
    LAUNCH_BG_PID=$!

    until grep -q "준비 완료" "/tmp/${runname}_launch.log" 2>/dev/null; do
        kill -0 "$LAUNCH_BG_PID" 2>/dev/null || { echo "ABORT: launcher died before becoming ready ($bagname)" >&2; cat "/tmp/${runname}_launch.log" >&2; exit 1; }
        sleep 1
    done
    echo "[$bagname] node ready"

    ros2 bag play "$bag" --rate 1.5 --clock > "/tmp/${runname}_bagplay.log" 2>&1
    echo "[$bagname] bag playback finished"

    sleep 2

    WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
    if [ -n "$WIN_ID" ]; then
        DISPLAY=:1 import -window "$WIN_ID" "$outdir/map.png"
        echo "[$bagname] map.png saved (window $WIN_ID)"
    else
        echo "[$bagname] WARNING: could not find rviz viewport window for screenshot"
    fi

    python3 "$REPO/scripts/plot_path_topdown.py" "$outdir/trajectory.tum" "$outdir/path.png"
    echo "[$bagname] path.png saved"

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
        echo "WARNING [$bagname]: leftover processes still alive after cleanup:" >&2
        echo "$LEFTOVER" >&2
    else
        echo "[$bagname] confirmed: node/rviz/republish/pidstat all dead"
    fi

    echo "=== $bagname : done ==="
}

for bagname in 15_a 15_b; do
    run_bag "$bagname"
done

echo "ALL RUNS COMPLETE"
