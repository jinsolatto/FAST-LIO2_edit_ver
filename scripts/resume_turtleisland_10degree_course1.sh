#!/bin/bash
# Re-run turtleisland_260915 10degree/course1_minimize (mid360.yaml then
# siheung.yaml, voxelslam-export matching) so the output dir matches the
# same 4-file pattern as the other completed turtleisland/siheung/suwon
# sweeps: fastlio2_degeneracy.csv, trajectory.tum, map.png, path.png.
# mid360.yaml/siheung.yaml already point tum_trajectory_log_path at
# <outdir>/trajectory.tum (pose.txt redirected to /tmp, out of the way).
#
# Sequential only (mid360.yaml, then siheung.yaml) with an explicit
# leftover-process check between runs, per the project rule against ever
# running two bag-play/node processes on the same ROS topics concurrently.

set -eo pipefail

REPO=/home/jschoi/edit_fastlio2
BAG=/home/jschoi/bag_file/10degree_/course1_minimize
OUTROOT="$REPO/turtleisland_260915/10degree/course1_minimize"

export FASTLIO_USE_VOXEL_MATCHING=1

run_config() {
    local config="$1"
    local runname="$2"
    local outdir="$OUTROOT/$config"

    echo "=== $config : starting ==="

    LEFTOVER=$(pgrep -af "fastlio_mapping|rviz2 -d.*fastlio|image_transport/republish|pidstat -p|ros2 bag play" 2>/dev/null || true)
    if [ -n "$LEFTOVER" ]; then
        echo "ABORT: leftover ROS/fastlio process(es) detected before starting $config:" >&2
        echo "$LEFTOVER" >&2
        exit 1
    fi

    export FASTLIO_DEGENERACY_LOG_DIR="$outdir"

    "$REPO/run_with_pidstat.sh" "$runname" "$config" > "/tmp/${runname}_launch.log" 2>&1 &
    LAUNCH_BG_PID=$!

    until grep -q "준비 완료" "/tmp/${runname}_launch.log" 2>/dev/null; do
        kill -0 "$LAUNCH_BG_PID" 2>/dev/null || { echo "ABORT: launcher died before becoming ready ($config)" >&2; cat "/tmp/${runname}_launch.log" >&2; exit 1; }
        sleep 1
    done
    echo "[$config] node ready"

    ros2 bag play "$BAG" --rate 1.0 --clock > "/tmp/${runname}_bagplay.log" 2>&1
    echo "[$config] bag playback finished"

    sleep 2

    WIN_ID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep 'rviz2".*1522x867+373+70' | grep -oE '0x[0-9a-f]+' | head -1)
    if [ -n "$WIN_ID" ]; then
        DISPLAY=:1 import -window "$WIN_ID" "$outdir/map.png"
        echo "[$config] map.png saved (window $WIN_ID)"
    else
        echo "[$config] WARNING: could not find rviz viewport window for screenshot"
    fi

    python3 "$REPO/scripts/plot_path_topdown.py" "$outdir/trajectory.tum" "$outdir/path.png"
    echo "[$config] path.png saved"

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
        echo "WARNING [$config]: leftover processes still alive after cleanup:" >&2
        echo "$LEFTOVER" >&2
    else
        echo "[$config] confirmed: node/rviz/republish/pidstat all dead"
    fi

    echo "=== $config : done ==="
}

run_config "mid360.yaml" "turtleisland_course1_minimize_mid360_pretty"
run_config "siheung.yaml" "turtleisland_course1_minimize_siheung_pretty"

echo "ALL RUNS COMPLETE"
