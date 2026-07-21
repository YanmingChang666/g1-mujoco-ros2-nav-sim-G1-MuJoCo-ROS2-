#!/usr/bin/env python3
"""Analyze a navigation debug bag for phantom-obstacle causes.

Checks, in order of likelihood for this workspace:
1. Lidar self-hits: cloud points closer than --self-radius to the sensor
   (robot arms/legs seen by the simulated Mid360).
2. Ground strikes: points whose implied world height falls inside the
   costmap's obstacle band (0.15-1.8 m) even though they lie at long range
   near the ground plane (odometry pitch/roll error smears the floor up).
3. Costmap accumulation: occupied-cell count of the global costmap over
   time; monotonic growth while driving means phantom marks are never
   cleared.
4. Odometry health: roll/pitch oscillation and publish rate of FAST-LIO.

Usage:
    python3 analyze_nav_bag.py <bag_dir> [--sensor-height 1.2] [--self-radius 0.9]

Requires a sourced ROS2 environment (uses rosbag2_py).
"""

import argparse
import math
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def open_reader(bag_dir):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def cloud_xyz(msg):
    """Extract Nx3 float32 xyz from the sim bridge's PointCloud2 layout."""
    if msg.point_step % 4 != 0 or len(msg.data) < msg.point_step:
        return np.empty((0, 3), dtype=np.float32)
    floats_per_point = msg.point_step // 4
    arr = np.frombuffer(bytes(msg.data), dtype=np.float32)
    n = len(arr) // floats_per_point
    return arr[: n * floats_per_point].reshape(n, floats_per_point)[:, :3]


def quat_roll_pitch(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir")
    parser.add_argument("--sensor-height", type=float, default=1.2,
                        help="lidar height above ground when standing [m]")
    parser.add_argument("--self-radius", type=float, default=0.9,
                        help="points closer than this are considered self-hits [m]")
    args = parser.parse_args()

    reader = open_reader(args.bag_dir)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_classes = {}
    for name, type_name in type_map.items():
        try:
            msg_classes[name] = get_message(type_name)
        except (LookupError, ModuleNotFoundError):
            pass

    cloud_frames = 0
    cloud_points = 0
    self_hits = []            # per-frame count of points within self-radius
    band_far_points = []      # per-frame: obstacle-band points at >2 m range
    z_hist = np.zeros(5, dtype=np.int64)
    z_edges = [0.10, 0.50, 1.00, 1.80]

    rolls, pitches = [], []
    odom_stamps = []

    costmap_occ = []          # (t, occupied cells) for /global_costmap/costmap

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        cls = msg_classes.get(topic)
        if cls is None:
            continue

        if topic == "/livox/lidar":
            msg = deserialize_message(raw, cls)
            xyz = cloud_xyz(msg)
            if xyz.size == 0:
                continue
            cloud_frames += 1
            cloud_points += len(xyz)
            rng = np.linalg.norm(xyz, axis=1)
            self_hits.append(int(np.sum(rng < args.self_radius)))
            world_z = xyz[:, 2] + args.sensor_height
            z_hist += np.histogram(world_z, bins=[-np.inf] + z_edges + [np.inf])[0]
            in_band = (world_z > 0.15) & (world_z < 1.8)
            band_far_points.append(int(np.sum(in_band & (rng > 2.0))))

        elif topic == "/Odometry":
            msg = deserialize_message(raw, cls)
            r, p = quat_roll_pitch(msg.pose.pose.orientation)
            rolls.append(math.degrees(r))
            pitches.append(math.degrees(p))
            odom_stamps.append(t_ns * 1e-9)

        elif topic == "/global_costmap/costmap":
            msg = deserialize_message(raw, cls)
            grid = np.asarray(msg.data, dtype=np.int16)
            costmap_occ.append((t_ns * 1e-9, int(np.sum(grid >= 99))))

    print("=" * 64)
    print(f"Bag: {args.bag_dir}")
    print("=" * 64)

    if cloud_frames:
        sh = np.asarray(self_hits)
        bf = np.asarray(band_far_points)
        print(f"\n[/livox/lidar]  {cloud_frames} frames, "
              f"{cloud_points // cloud_frames} pts/frame avg")
        print(f"  self-hits (<{args.self_radius:.2f} m): "
              f"mean {sh.mean():.1f}/frame, max {sh.max()}")
        if sh.mean() > 5:
            print("  >> VERDICT: robot sees its own body. Rebuild "
                  "unitree_mujoco/simulate with the IsSelfHit fix.")
        else:
            print("  >> self-hits negligible.")
        labels = ["<0.10 (ground)", "0.10-0.50", "0.50-1.00 (table)",
                  "1.00-1.80", ">1.80"]
        total = max(1, int(z_hist.sum()))
        print(f"  implied world-z distribution (sensor height "
              f"{args.sensor_height:.2f} m):")
        for lab, cnt in zip(labels, z_hist):
            print(f"    {lab:<18} {cnt:>9}  ({100.0 * cnt / total:5.1f}%)")
        print(f"  obstacle-band points at >2 m range: mean {bf.mean():.0f}/frame")
    else:
        print("\n[/livox/lidar]  no messages found!")

    if rolls:
        r = np.asarray(rolls)
        p = np.asarray(pitches)
        dt = np.diff(np.asarray(odom_stamps))
        rate = 1.0 / dt.mean() if len(dt) else 0.0
        print(f"\n[/Odometry]  {len(rolls)} msgs at ~{rate:.1f} Hz")
        print(f"  roll:  rms {np.sqrt((r ** 2).mean()):.2f} deg, "
              f"max |{np.abs(r).max():.2f}| deg")
        print(f"  pitch: rms {np.sqrt((p ** 2).mean()):.2f} deg, "
              f"max |{np.abs(p).max():.2f}| deg")
        tilt = max(np.abs(r).max(), np.abs(p).max())
        smear = 6.0 * math.tan(math.radians(tilt))
        print(f"  worst-case ground-height error at 6 m range: {smear:.2f} m")
        if smear > 0.15:
            print("  >> walking sway can push floor points above the 0.15 m "
                  "obstacle threshold at long range; consider lowering the "
                  "cloud's obstacle_max_range or raising min_obstacle_height.")
    else:
        print("\n[/Odometry]  no messages found!")

    if costmap_occ:
        t0 = costmap_occ[0][0]
        print(f"\n[/global_costmap/costmap]  {len(costmap_occ)} snapshots")
        for t, occ in costmap_occ[:: max(1, len(costmap_occ) // 8)]:
            print(f"    t={t - t0:6.1f}s  occupied cells: {occ}")
        growth = costmap_occ[-1][1] - costmap_occ[0][1]
        print(f"  net growth over bag: {growth} cells")
        if growth > 500:
            print("  >> costmap accumulates marks; if these are phantom, "
                  "they come from the cloud source (see verdicts above).")
    else:
        print("\n[/global_costmap/costmap]  no messages found!")

    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
