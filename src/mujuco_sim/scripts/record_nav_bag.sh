#!/usr/bin/env bash
# Record a short diagnostic bag while reproducing a navigation problem.
#
# Usage:
#   ./record_nav_bag.sh [duration_seconds] [output_dir]
#
# Defaults: 30 seconds, <workspace>/bags/nav_debug_<timestamp>.
# Run it in a separate terminal while nav.launch.py is running, then
# reproduce the problem (drive toward the phantom obstacles).
set -euo pipefail

DURATION="${1:-30}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
OUT="${2:-$WS_DIR/bags/nav_debug_$(date +%Y%m%d_%H%M%S)}"

# Join the simulation's private DDS network (see README "DDS Notes").
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$WS_DIR/src/mujuco_sim/config/cyclonedds_lo.xml"

mkdir -p "$(dirname "$OUT")"

echo "Recording ${DURATION}s of navigation topics to: $OUT"
echo "Reproduce the problem now (send a goal / drive the robot)."

# ros2 bag record has no total-duration option on Humble; use timeout
# with SIGINT so the bag closes cleanly.
timeout --signal=INT "${DURATION}" ros2 bag record -o "$OUT" \
  /livox/lidar \
  /scan \
  /imu/data \
  /Odometry \
  /odom \
  /tf \
  /tf_static \
  /amcl_pose \
  /particle_cloud \
  /plan \
  /local_plan \
  /cmd_vel \
  /map \
  /local_costmap/costmap \
  /global_costmap/costmap \
  || true

echo
echo "Done: $OUT"
echo "Analyze it with:"
echo "  python3 $SCRIPT_DIR/analyze_nav_bag.py $OUT"
