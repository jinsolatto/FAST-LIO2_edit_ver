#!/usr/bin/env python3
"""
One-shot, no-ROS-node-needed video generator: reads /livox/points directly
out of the bag file, classifies every point in every scan as
surviving/rejected using the EXACT same rule as Preprocess::mid360_handler
(front-reject sector + blind distance, both read from the fast_lio config
yaml), and renders a top-down view per scan -- raw points dim gray,
surviving points highlighted -- straight into an mp4.

No rviz, no bag play, no fast_lio launch required. Just:

    python3 bag_survivors_video.py

Defaults match the current session's bag/config; override with flags if
those change.
"""
import argparse
import os
import subprocess

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def load_preprocess_params(config_path: str):
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    params = data.get("/**", {}).get("ros__parameters", {})
    pre = params.get("preprocess", {})
    mapping = params.get("mapping", {})
    return {
        "blind": float(pre.get("blind", 0.01)),
        "front_reject_enable": bool(mapping.get("front_reject_enable", False)),
        "front_reject_start_time": float(mapping.get("front_reject_start_time", 0.0)),
        "front_reject_end_time": float(mapping.get("front_reject_end_time", 0.0)),
    }


def open_bag_reader(bag_path: str, topic: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise RuntimeError(f"Topic {topic} not found in bag. Available: {list(type_map)}")
    msg_type = get_message(type_map[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return reader, msg_type


def render_frame(x, y, survived, canvas, range_m, scan_idx, t_rel, front_active,
                  raw_n, survived_n):
    H, W = canvas, canvas
    img = np.zeros((H, W, 3), dtype=np.uint8)

    px = (W / 2 + (y / range_m) * (W / 2)).astype(np.int32)
    py = (H / 2 - (x / range_m) * (H / 2)).astype(np.int32)
    in_bounds = (px >= 0) & (px < W) & (py >= 0) & (py < H)

    # raw (dim gray) -- every point, drawn first, lightly dilated so faint
    # points stay visible against the black background
    raw_mask = np.zeros((H, W), dtype=np.uint8)
    m = in_bounds
    raw_mask[py[m], px[m]] = 255
    raw_mask = cv2.dilate(raw_mask, np.ones((2, 2), np.uint8), iterations=1)
    img[raw_mask > 0] = (95, 95, 95)

    # survivors (highlighted) -- drawn on top, dilated so they stand out
    surv_mask = np.zeros((H, W), dtype=np.uint8)
    m2 = in_bounds & survived
    surv_mask[py[m2], px[m2]] = 255
    surv_mask = cv2.dilate(surv_mask, np.ones((3, 3), np.uint8), iterations=1)
    img[surv_mask > 0] = (40, 40, 255)  # BGR bright red

    # sensor origin marker
    cv2.drawMarker(img, (W // 2, H // 2), (0, 255, 0), cv2.MARKER_CROSS, 12, 1)

    rejected_n = raw_n - survived_n
    lines = [
        f"scan {scan_idx:5d}  t = {t_rel:7.2f}s",
        f"raw = {raw_n}   survived = {survived_n}   rejected = {rejected_n}",
    ]
    color0 = (0, 0, 0)
    cv2.rectangle(img, (0, 0), (W, 58), (20, 20, 20), -1)
    cv2.putText(img, lines[0], (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, lines[1], (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    if front_active:
        cv2.putText(img, "FRONT-REJECT ACTIVE", (W - 260, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2, cv2.LINE_AA)
    return img


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", default="/home/jschoi/bag_file/10_b_pointcloud2/")
    parser.add_argument(
        "--config",
        default="/home/jschoi/edit_fastlio2/src/FAST_LIO_ROS2/config/mid360_front_reject.yaml",
    )
    parser.add_argument("--topic", default="/livox/points")
    parser.add_argument("--out", default="/home/jschoi/edit_fastlio2/surviving_points.mp4")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--canvas", type=int, default=900)
    parser.add_argument("--range-m", type=float, default=4.0, help="view half-extent in meters")
    parser.add_argument("--start-time", type=float, default=None, help="only render scans after this relative time [s]")
    parser.add_argument("--end-time", type=float, default=None, help="only render scans before this relative time [s]")
    parser.add_argument("--log-every", type=int, default=100)
    parser.add_argument("--no-compress", action="store_true",
                         help="skip the libx264 re-encode pass (keeps the much larger raw mp4v file)")
    args = parser.parse_args()

    p = load_preprocess_params(args.config)
    print(f"preprocess params: {p}")

    reader, msg_type = open_bag_reader(args.bag, args.topic)

    raw_out = args.out if args.no_compress else args.out + ".raw.mp4"
    writer = cv2.VideoWriter(
        raw_out, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (args.canvas, args.canvas)
    )

    first_time = None
    scan_idx = 0
    written = 0
    field_offsets = None
    point_step = None

    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, msg_type)
        n = msg.width * msg.height
        if n == 0:
            continue

        if field_offsets is None:
            field_offsets = {f.name: f.offset for f in msg.fields}
            point_step = msg.point_step

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if first_time is None:
            first_time = stamp_sec
        t_rel = stamp_sec - first_time
        scan_idx += 1

        if args.start_time is not None and t_rel < args.start_time:
            continue
        if args.end_time is not None and t_rel > args.end_time:
            break

        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [field_offsets["x"], field_offsets["y"], field_offsets["z"]],
            "itemsize": point_step,
        })
        arr = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
        x = arr["x"].astype(np.float64)
        y = arr["y"].astype(np.float64)
        z = arr["z"].astype(np.float64)

        front_active = p["front_reject_enable"] and (p["front_reject_start_time"] <= t_rel <= p["front_reject_end_time"])
        front_azimuth_deg = np.degrees(np.arctan2(x, y))
        front_sector = (front_azimuth_deg >= -30.0) & (front_azimuth_deg <= 30.0)
        blind = p["blind"]
        within_blind = (x * x + y * y + z * z) <= (blind * blind)
        rejected = within_blind | (front_active & front_sector)
        survived = ~rejected

        frame = render_frame(x, y, survived, args.canvas, args.range_m, scan_idx, t_rel,
                              front_active, n, int(survived.sum()))
        writer.write(frame)
        written += 1

        if scan_idx % args.log_every == 0:
            print(f"scan {scan_idx}  t={t_rel:7.2f}s  raw={n}  survived={int(survived.sum())}  front_active={front_active}")

    writer.release()

    if args.no_compress:
        print(f"Wrote {written} frames -> {args.out}")
        return

    print(f"Wrote {written} frames -> {raw_out}, re-encoding with libx264...")
    subprocess.run(
        ["ffmpeg", "-y", "-i", raw_out, "-c:v", "libx264", "-preset", "medium",
         "-crf", "23", "-pix_fmt", "yuv420p", args.out, "-loglevel", "error"],
        check=True,
    )
    os.remove(raw_out)
    print(f"Compressed -> {args.out}")


if __name__ == "__main__":
    main()
