#!/usr/bin/env bash
# Record a bag suitable for timestamp-sync analysis, and prompt the motion that
# makes the analysis possible.
#
# The content-level check cross-correlates what the sensors MEASURED, so the bag
# must contain motion with TIMING FEATURES. A constant-rate spin has none: every
# time-shift of it looks like every other, so the correlation peak is noise. A
# measured case gave corr 0.10 and a confident-looking -275 ms that was garbage.
# Hence the reversals and speed changes below -- they are the point, not filler.
set -euo pipefail
OUT="${1:-sync_bag_$(date +%Y%m%d_%H%M%S)}"
QOS="$(dirname "$0")/record_qos.yaml"

TOPICS="/imu /front_lidar/imu /front_camera/imu /odom/mujoco_odom /livox/lidar /joint_states /tf /tf_static"

echo "recording -> $OUT   (~5.7 MB/s, so ~510 MB for the full 90 s)"
ros2 bag record -o "$OUT" --qos-profile-overrides-path "$QOS" \
  --max-cache-size 536870912 $TOPICS &
BAG=$!
trap 'kill -INT $BAG 2>/dev/null || true; wait $BAG 2>/dev/null || true' EXIT

step() { printf '\n[%3ds] %s\n' "$1" "$2"; }
sleep 3   # let discovery settle before the first cue

step  0 "STAY STILL (baseline for the stamp-level checks)"; sleep 15
step 15 "IN-PLACE ROTATION WITH REVERSALS: left ~2s, stop ~1s, right ~2s, stop ~1s."
echo "        Repeat, and VARY THE SPEED. Do not settle into a constant spin."
sleep 30
step 45 "SHORT YAW PULSES: quick jabs left/right, snap stops. Highest-value segment."; sleep 15
step 60 "TRANSLATE: forward a few metres, then back. Little or no turning."; sleep 15
step 75 "ROTATION WITH REVERSALS again (a second independent window)"; sleep 15
step 90 "done -- stopping the recorder"
sleep 1
