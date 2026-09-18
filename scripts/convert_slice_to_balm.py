"""Convert a fastlio2 slice (pose.txt + world-frame pcd/*.pcd) into balm2's
benchmark_realworld input format (alidarPose.csv + full{i}.pcd, sensor-local frame).

Usage:
    python3 convert_slice_to_balm.py <slice_dir> <output_dir>

<slice_dir> must contain:
    pose.txt   - lines of "wall_ts elapsed_t x y z qx qy qz qw"
    pcd/       - scan_XXXXXX.pcd files, world-frame, one per pose.txt line (same order)
"""
import numpy as np
import os
import sys

if len(sys.argv) != 3:
    print(__doc__)
    sys.exit(1)

SRC, DST = sys.argv[1], sys.argv[2]
os.makedirs(DST, exist_ok=True)

poses = []
with open(os.path.join(SRC, "pose.txt")) as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        wall_ts, elapsed_t, x, y, z, qx, qy, qz, qw = (float(v) for v in line.split())
        poses.append((elapsed_t, x, y, z, qx, qy, qz, qw))

print("num poses:", len(poses))

pcd_files = sorted(f for f in os.listdir(os.path.join(SRC, "pcd")) if f.endswith(".pcd"))
print("num pcd files:", len(pcd_files))
assert len(pcd_files) == len(poses), "pose/pcd count mismatch"


def quat_to_R(qx, qy, qz, qw):
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def read_pcd_xyzi(path):
    with open(path, "rb") as f:
        data = f.read()
    idx = data.find(b"DATA binary")
    header = data[:idx].decode(errors="ignore")
    n_points = n_fields = None
    for line in header.splitlines():
        if line.startswith("POINTS"):
            n_points = int(line.split()[1])
        if line.startswith("FIELDS"):
            n_fields = len(line.split()) - 1
    start = data.find(b"\n", idx) + 1
    arr = np.frombuffer(data, dtype=np.float32, count=n_points * n_fields, offset=start)
    return arr.reshape(n_points, n_fields)[:, 0:4]  # x,y,z,intensity


def write_pcd_xyzi(path, arr):
    n = arr.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(arr.astype(np.float32).tobytes())


csv_lines = []
for i, (fn, pose) in enumerate(zip(pcd_files, poses)):
    elapsed_t, x, y, z, qx, qy, qz, qw = pose
    R = quat_to_R(qx, qy, qz, qw)
    t = np.array([x, y, z])

    xyzi = read_pcd_xyzi(os.path.join(SRC, "pcd", fn))
    pts_local = (xyzi[:, 0:3].astype(np.float64) - t) @ R  # world -> sensor-local
    out = np.empty_like(xyzi)
    out[:, 0:3] = pts_local.astype(np.float32)
    out[:, 3] = xyzi[:, 3]

    write_pcd_xyzi(os.path.join(DST, f"full{i}.pcd"), out)

    csv_lines.append(f"{R[0,0]:.6f},{R[0,1]:.6f},{R[0,2]:.6f},{x:.6f},")
    csv_lines.append(f"{R[1,0]:.6f},{R[1,1]:.6f},{R[1,2]:.6f},{y:.6f},")
    csv_lines.append(f"{R[2,0]:.6f},{R[2,1]:.6f},{R[2,2]:.6f},{z:.6f},")
    csv_lines.append(f"0.000000,0.000000,0.000000,{elapsed_t:.6f},")

with open(os.path.join(DST, "alidarPose.csv"), "w") as f:
    f.write("\n".join(csv_lines) + "\n")

print("wrote", len(pcd_files), "scans to", DST)
