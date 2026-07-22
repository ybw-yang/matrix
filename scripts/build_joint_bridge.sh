#!/usr/bin/env bash
# Build the eCAL -> UDP joint bridge (and probe) into scripts/bin/.
# Requires: eCAL dev libs, protobuf, /usr/include/robot_sdk.pb.h, /usr/lib/librobot_sdk.so
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${DIR}/bin"
mkdir -p "${OUT}"
FLAGS=(-std=c++17 -O2 -I/usr/include -lecal_core -lecal_core_pb -lprotobuf -lrobot_sdk -pthread)

echo "[build] mujoco_joint_bridge"
g++ "${DIR}/mujoco_joint_bridge.cpp" "${FLAGS[@]}" -o "${OUT}/mujoco_joint_bridge"
echo "[build] mujoco_state_probe"
g++ "${DIR}/mujoco_state_probe.cpp" "${FLAGS[@]}" -o "${OUT}/mujoco_state_probe"
echo "[ok] built into ${OUT}"
