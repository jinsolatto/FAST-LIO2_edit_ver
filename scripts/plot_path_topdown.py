#!/usr/bin/env python3
"""
Render a top-down XY trajectory from a TUM file in the same visual style
as rviz's Path display (green line, pure black background, no axes/ticks),
so a clean "path" screenshot can be produced without touching rviz's
Displays panel.

Usage:
    python3 plot_path_topdown.py <tum_path> <out_png> [max_time_s] [--smooth=N]

If max_time_s is given, only poses within that many seconds of the first
pose are plotted (useful to exclude a divergent tail from the render).

--smooth=N applies an N-sample centered moving average to x/y before
plotting, for display only (the underlying TUM file is never touched).
Use it when small-scale (cm-level) pose jitter, invisible at rviz's normal
zoom, gets visually exaggerated by this script's tight crop to the path's
own bounding box.
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_tum(path):
    data = []
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 8:
                continue
            data.append([float(x) for x in p[:8]])
    return np.array(data)


def smooth(a, window):
    if window <= 1:
        return a
    kernel = np.ones(window) / window
    pad = window // 2
    padded = np.pad(a, (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")[: len(a)]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    tum_path, out_path = args[0], args[1]
    max_time = float(args[2]) if len(args) > 2 else None
    smooth_window = 1
    for f in flags:
        if f.startswith("--smooth="):
            smooth_window = int(f.split("=", 1)[1])

    arr = load_tum(tum_path)
    if max_time is not None:
        t = arr[:, 0] - arr[0, 0]
        arr = arr[t <= max_time]
    x, y = arr[:, 1], arr[:, 2]
    if smooth_window > 1:
        x, y = smooth(x, smooth_window), smooth(y, smooth_window)

    fig, ax = plt.subplots(figsize=(10.14, 7.8), dpi=100)
    fig.patch.set_facecolor("black")
    ax.set_facecolor("black")
    ax.plot(x, y, color="#00ff00", linewidth=1.2)
    ax.set_aspect("equal")
    ax.axis("off")
    plt.tight_layout(pad=0)
    plt.savefig(out_path, facecolor="black")
    print(f"saved: {out_path}")


if __name__ == "__main__":
    main()
