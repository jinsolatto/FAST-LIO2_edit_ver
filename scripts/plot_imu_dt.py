#!/usr/bin/env python3
"""Plot /livox/imu header-stamp intervals from an MCAP rosbag."""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("output")
    args = parser.parse_args()

    with open(args.bag, "rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        stamps = [
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            for _, _, _, msg in reader.iter_decoded_messages(topics=["/livox/imu"])
        ]

    seconds = [(stamp - stamps[0]) / 1e9 for stamp in stamps]
    dt_seconds = [(right - left) / 1e9 for left, right in zip(stamps, stamps[1:])]
    duration = seconds[-1]
    edges = np.arange(0, np.ceil(duration) + 1, 1)
    rates, _ = np.histogram(seconds, bins=edges)
    time = edges[:-1]
    # A full-second rate below 180 Hz marks a sustained departure from 200 Hz.
    first_drop = next(index for index, rate in enumerate(rates) if rate < 180)
    mean_after_drop = rates[first_drop:].mean()

    bag_name = Path(args.bag).stem.removesuffix("_minimize")
    fig, axis = plt.subplots(figsize=(20, 10))
    fig.subplots_adjust(top=.82, left=.065, right=.96, bottom=.12)
    fig.text(0.012, .965, f"bag: {bag_name}   |   {duration:.0f}s, {len(stamps)} msgs   |   mean {mean_after_drop:.1f} Hz after drop", color="#555555", fontsize=14)
    axis.plot(time, rates, color="#2878d4", linewidth=2.6)
    axis.axhline(200, color="#777777", linestyle=(0, (1.5, 2.5)), linewidth=1.2)
    axis.text(duration + duration * .018, 200, "200 Hz", color="#555555", va="center", fontsize=13)
    axis.axvline(first_drop, color="#ef5350", linestyle=(0, (4, 4)), linewidth=2)
    axis.annotate(f"drop\n{first_drop:.1f}s", xy=(first_drop, 150), xytext=(first_drop + duration * .03, 150), color="#ef5350", fontsize=14, fontweight="bold", va="center", arrowprops={"arrowstyle": "-", "color": "#ef5350", "linewidth": 1.6})
    axis.set_title("/livox/imu average rate per second", fontsize=20, fontweight="bold", loc="left", pad=24)
    axis.set_xlabel("time", fontsize=14)
    axis.set_ylabel("Hz", fontsize=14)
    axis.set_xlim(0, duration)
    axis.set_ylim(0, 220)
    axis.grid(axis="y", color="#dddddd", linewidth=1)
    axis.spines[["top", "right"]].set_visible(False)
    fig.savefig(args.output, dpi=160)


if __name__ == "__main__":
    main()
