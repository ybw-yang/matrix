#!/usr/bin/env bash
# One-key launch: MATRiX simulator + full TF tree.
#
# Starts, in order:
#   1. the simulator itself        (scripts/run_sim.sh <robot> <scene> -> mujoco + UE + mc)
#   2. robot_forward               (odom -> base_link, from /opt/robot)
#   3. scripts/rsp.launch.py       (robot_state_publisher + base_link bridge +
#                                   real joints via eCAL + base_footprint + sensor tf)
#
# The simulator is NEVER torn down on error/timeout — only a deliberate Ctrl-C
# (or the simulator exiting on its own) stops the stack.
#
# Usage:
#   scripts/run_sim_with_tf.sh [robot] [scene_id]
#     robot     xgb (default) | xgw | zgws | go2 | go2w   (legs/sensors tf need a URDF: xgb/xgw only)
#     scene_id  map id passed to run_sim.sh (default 13 = OfficeWorld)
#
# Env overrides:
#   FOOTPRINT_MODE=footplane|projection   (default footplane)
#   RMW_IMPLEMENTATION / ROS_DOMAIN_ID    (default rmw_cyclonedds_cpp / 89 — matches the sim)
#   SIM_WAIT_TIMEOUT=90                    seconds to wait for odom before starting tf anyway
#   ATTACH=1                              do NOT start the sim; attach tf to an already-running sim
#   MUJOCO=1                              (ATTACH=0 only) start the MuJoCo window with keyboard
#                                         control (U=stand, WASD=move); set 0 for UE render only
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

ROBOT="${1:-xgb}"
SCENE="${2:-13}"
FOOTPRINT_MODE="${FOOTPRINT_MODE:-footplane}"
SIM_WAIT_TIMEOUT="${SIM_WAIT_TIMEOUT:-90}"
ATTACH="${ATTACH:-1}"
MUJOCO="${MUJOCO:-0}"   # 1 = enable MuJoCo physics window -> keyboard control

export SDK_CLIENT_IP="${SDK_CLIENT_IP:-127.0.0.1}"

LOG_DIR="$PROJECT_ROOT/bin"
mkdir -p "$LOG_DIR"

# robot -> URDF (only xgb/xgw ship a URDF; others still get odom->base_link)
case "$ROBOT" in
  xgb) URDF="src/robot_mujoco/zsibot_robots/xgb/xg_b.urdf" ;;
  xgw) URDF="src/robot_mujoco/zsibot_robots/xgw/xg_wheel.urdf" ;;
  *)   URDF="" ;;
esac

FORWARD_SETUP="/opt/robot/robot-forward/install/setup.bash"
FORWARD_BIN="/opt/robot/robot-forward/install/robot_forward/lib/robot_forward/robot_forward"

SIM_PGID=""       # process-group id of run_sim.sh (has mujoco/UE/mc as children); empty in ATTACH mode
FWD_PID=""
RSP_PID=""
STOPPING=0

log() { echo "[run_all] $*"; }

# Stop only the TF stack we started (leaves the simulator alone).
stop_tf() {
  [ -n "$RSP_PID" ] && kill -INT "$RSP_PID" 2>/dev/null
  [ -n "$FWD_PID" ] && kill "$FWD_PID" 2>/dev/null
  pkill -f "scripts/rsp.launch.py" 2>/dev/null
  pkill -f "scripts/bin/mujoco_joint_bridge" 2>/dev/null
  pkill -f "scripts/joint_state_udp_bridge.py" 2>/dev/null
  pkill -f "scripts/base_footprint_publisher.py" 2>/dev/null
  pkill -f "scripts/sensor_tf_publisher.py" 2>/dev/null
}

# Full stop, triggered ONLY by an explicit Ctrl-C / TERM.
on_interrupt() {
  trap - INT TERM
  [ "$STOPPING" = "1" ] && return
  STOPPING=1
  echo
  log "Ctrl-C — stopping TF stack$([ "$ATTACH" = 1 ] && echo '' || echo ' + simulator') ..."
  stop_tf
  if [ "$ATTACH" != "1" ] && [ -n "$SIM_PGID" ]; then
    kill -TERM "-$SIM_PGID" 2>/dev/null
    sleep 2
    kill -KILL "-$SIM_PGID" 2>/dev/null
  fi
  log "done."
  exit 0
}
# NOTE: intentionally NOT trapping EXIT — a script error/timeout must never
# tear down a running simulator.
trap on_interrupt INT TERM

# 0. build the eCAL->UDP joint bridge if it isn't built yet
if [ ! -x scripts/bin/mujoco_joint_bridge ]; then
  log "building eCAL joint bridge ..."
  bash scripts/build_joint_bridge.sh || log "WARN: bridge build failed — real joints unavailable"
fi

# source ROS for ros2 topic list / launch
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

# 1. simulator (skipped in ATTACH mode)
if [ "$ATTACH" = "1" ]; then
  log "ATTACH mode: using the already-running simulator (not starting one)."
else
  log "starting simulator: robot=$ROBOT scene=$SCENE mujoco=$MUJOCO (log: bin/run_sim.log)"
  # run_sim.sh args: ROBOT SCENE OFFSCREEN PIXELSTREAM MUJOCORUNNING
  # MUJOCORUNNING=1 starts the robot_mujoco window that provides keyboard control.
  setsid bash scripts/run_sim.sh "$ROBOT" "$SCENE" 0 0 "$MUJOCO" > "$LOG_DIR/run_sim.log" 2>&1 &
  SIM_PGID=$!   # with setsid the child is its own group leader; pid == pgid
  [ "$MUJOCO" = "1" ] && log "keyboard control ON — focus the MuJoCo window, press U to stand, WASD to move."
fi

# 2. best-effort wait for the sim to publish odom. On timeout we DO NOT kill the
#    sim — the tf nodes below tolerate odom arriving later (e.g. after the robot
#    stands up), so we just warn and continue.
log "waiting up to ${SIM_WAIT_TIMEOUT}s for /odom/mujoco_odom (best-effort) ..."
deadline=$((SECONDS + SIM_WAIT_TIMEOUT))
sim_ready=0
while [ "$SECONDS" -lt "$deadline" ]; do
  if ros2 topic list 2>/dev/null | grep -q "/odom/mujoco_odom"; then
    sim_ready=1; break
  fi
  # if we launched the sim and it died during boot, report but do not kill anything
  if [ "$ATTACH" != "1" ] && [ -n "$SIM_PGID" ] && ! kill -0 "$SIM_PGID" 2>/dev/null; then
    log "ERROR: simulator exited during startup — see bin/run_sim.log. Not starting tf."
    exit 1
  fi
  sleep 1
done
if [ "$sim_ready" = "1" ]; then
  log "odom detected — simulator is up."
else
  log "WARN: /odom/mujoco_odom not seen in ${SIM_WAIT_TIMEOUT}s. Starting tf anyway"
  log "      (odom often appears only once the robot is activated — press U to stand)."
fi

# 3. robot_forward: odom -> base_link
if [ -f "$FORWARD_SETUP" ] && [ -x "$FORWARD_BIN" ]; then
  log "starting robot_forward (odom -> base_link, log: bin/robot_forward.log)"
  ( set +u; source "$FORWARD_SETUP"; exec "$FORWARD_BIN" ) > "$LOG_DIR/robot_forward.log" 2>&1 &
  FWD_PID=$!
else
  log "WARN: robot_forward not found at $FORWARD_BIN — no odom->base_link tf"
fi

# 4. tf stack (needs a URDF for legs/sensors/footplane)
if [ -n "$URDF" ]; then
  log "starting tf stack: rsp.launch.py urdf=$URDF footprint_mode=$FOOTPRINT_MODE (log: bin/rsp.log)"
  ros2 launch scripts/rsp.launch.py urdf:="$URDF" footprint_mode:="$FOOTPRINT_MODE" \
      > "$LOG_DIR/rsp.log" 2>&1 &
  RSP_PID=$!
else
  log "WARN: '$ROBOT' has no URDF (only xgb/xgw) — skipping legs/sensors/base_footprint tf."
fi

log "all components started. TF: odom -> base_link -> BASE_LINK -> {legs/feet, sensors} + base_footprint"
log "Press Ctrl-C to stop$([ "$ATTACH" = 1 ] && echo ' the tf stack' || echo ' everything')."

if [ "$ATTACH" = "1" ]; then
  # nothing of ours to wait on except the tf nodes; block until interrupted
  [ -n "$RSP_PID" ] && wait "$RSP_PID" || wait
else
  # block until the simulator exits; then tear down the (now-useless) tf stack
  wait "$SIM_PGID" 2>/dev/null || true
  if [ "$STOPPING" != "1" ]; then
    log "simulator exited — stopping tf stack."
    stop_tf
  fi
fi
