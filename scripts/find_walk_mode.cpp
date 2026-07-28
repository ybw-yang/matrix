// find_walk_mode: sweep SDKCmd (control_mode, motion_mode) pairs and measure
// whether the robot actually TRANSLATES, to discover the velocity-walk mode.
//
// Why this exists: control_mode=21 / motion_mode=100 turned out to be the
// RL_BalanceStand "twist" pose (in-place body roll/pitch/yaw), NOT locomotion.
// The translation-walk policy is RL_MIX (policy_mix_walk); we need the (cm, mm)
// pair that selects it. This tool drives a small forward vx for each candidate
// and reports horizontal displacement + mean world speed, read straight from
// eCAL "sdk_robotstate" (fields position[19], v_world[20], cur_ctrl_mode,
// cur_motion_mode) -- no ROS needed.
//
// SAFETY: this ACTIVELY COMMANDS the robot. Use in SIM only, with the robot
// already STANDING (keyboard U) and clear space ahead. Forward speed defaults
// to 0.25 m/s for ~2.5s per candidate, then it stops and re-issues stand.
//
// Usage:
//   find_walk_mode                       # OBSERVE only: print cur modes + pose for 6s
//   find_walk_mode sweep                 # sweep the built-in candidate list
//   find_walk_mode 21:0 2:0 11:0 20:0    # sweep an explicit cm:mm list
//   VX=0.3 DRIVE_S=3 find_walk_mode sweep
// Env: VX (fwd speed), DRIVE_S (drive secs), SETTLE_S (stop/stand secs).
// NOTE: SDKCmd has no enable_control_mode field (only vx/vy/vz, yaw/pitch/roll_rate,
//       control_mode, motion_mode), so nothing extra needs enabling.
//
// Build via scripts/build_joint_bridge.sh (added there), or manually:
//   g++ -std=c++17 -O2 scripts/find_walk_mode.cpp -I/usr/include \
//       -o scripts/bin/find_walk_mode \
//       -lecal_core -lecal_core_pb -lprotobuf -lrobot_sdk -pthread
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include "robot_sdk.pb.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

static std::mutex g_mtx;
static float g_pos[3] = {0, 0, 0};
static float g_vworld[3] = {0, 0, 0};
static bool g_have_state = false;
static std::atomic<int> g_cur_ctrl{-1}, g_cur_motion{-1};

static uint64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

static double getenv_d(const char* k, double def) {
  const char* v = std::getenv(k);
  return v ? std::atof(v) : def;
}

static void OnState(const char*, const robot_sdk::pb::SDKRobotState& s,
                    long long, long long, long long) {
  std::lock_guard<std::mutex> lk(g_mtx);
  for (int i = 0; i < 3 && i < s.position_size(); ++i) g_pos[i] = s.position(i);
  for (int i = 0; i < 3 && i < s.v_world_size(); ++i) g_vworld[i] = s.v_world(i);
  g_have_state = true;
  g_cur_ctrl = static_cast<int>(s.cur_ctrl_mode());
  g_cur_motion = static_cast<int>(s.cur_motion_mode());
}

static void snapshot(float pos[3], float vw[3], bool* have) {
  std::lock_guard<std::mutex> lk(g_mtx);
  for (int i = 0; i < 3; ++i) { pos[i] = g_pos[i]; vw[i] = g_vworld[i]; }
  *have = g_have_state;
}

// Send one SDKCmd on the "sdk_cmd" publisher.
static void send_cmd(eCAL::protobuf::CPublisher<robot_sdk::pb::SDKCmd>& pub,
                     float vx, float vy, float yaw, int cm, int mm) {
  robot_sdk::pb::SDKCmd cmd;
  cmd.set_vx(vx);
  cmd.set_vy(vy);
  cmd.set_yaw_rate(yaw);
  cmd.set_control_mode(static_cast<uint32_t>(cm));
  cmd.set_motion_mode(static_cast<uint32_t>(mm));
  cmd.set_time_stamp(now_ns());
  pub.Send(cmd);
}

// Drive (cm, mm) with forward vx for drive_s, measuring displacement + speed.
static void test_pair(eCAL::protobuf::CPublisher<robot_sdk::pb::SDKCmd>& pub,
                      int cm, int mm, double vx, double drive_s,
                      double settle_s) {
  const double rate = 50.0;
  const auto period = std::chrono::milliseconds((int)(1000.0 / rate));

  float p0[3], v[3]; bool have0 = false;
  snapshot(p0, v, &have0);

  std::printf("\n==== TEST control_mode=%d motion_mode=%d (vx=%.2f, %.1fs) ====\n",
              cm, mm, vx, drive_s);
  if (!have0) std::printf("  (no sdk_robotstate yet -- is the sim running?)\n");

  double vmax = 0.0, vsum = 0.0; int nsamp = 0;
  auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < drive_s) {
    send_cmd(pub, (float)vx, 0.f, 0.f, cm, mm);
    float pc[3], vc[3]; bool h = false;
    snapshot(pc, vc, &h);
    double sp = std::sqrt(vc[0] * vc[0] + vc[1] * vc[1]);
    vmax = std::max(vmax, sp); vsum += sp; ++nsamp;
    std::this_thread::sleep_for(period);
  }

  float p1[3], v1[3]; bool have1 = false;
  snapshot(p1, v1, &have1);
  double dx = p1[0] - p0[0], dy = p1[1] - p0[1];
  double disp = std::sqrt(dx * dx + dy * dy);
  double vmean = nsamp ? vsum / nsamp : 0.0;

  std::printf("  -> displacement=%.3f m  (dx=%.3f dy=%.3f)  mean|v|=%.3f max|v|=%.3f m/s\n",
              disp, dx, dy, vmean, vmax);
  std::printf("  -> robot reports cur_ctrl_mode=%d cur_motion_mode=%d\n",
              g_cur_ctrl.load(), g_cur_motion.load());
  std::printf("  -> VERDICT: %s\n",
              (disp > 0.10 || vmean > 0.05) ? "*** MOVED (translation) ***"
                                            : "no translation");

  // Stop, then re-issue stand (cm=1/mm=10) so the next candidate starts settled.
  auto ts = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - ts).count() < settle_s) {
    send_cmd(pub, 0.f, 0.f, 0.f, cm, mm);
    std::this_thread::sleep_for(period);
  }
  ts = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - ts).count() < settle_s) {
    send_cmd(pub, 0.f, 0.f, 0.f, 1, 10);   // stand
    std::this_thread::sleep_for(period);
  }
}

int main(int argc, char** argv) {
  const double vx      = getenv_d("VX", 0.25);
  const double drive_s = getenv_d("DRIVE_S", 2.5);
  const double settle_s = getenv_d("SETTLE_S", 1.5);

  eCAL::Initialize(argc, argv, "find_walk_mode");
  eCAL::protobuf::CSubscriber<robot_sdk::pb::SDKRobotState> sub("sdk_robotstate");
  sub.AddReceiveCallback(std::bind(OnState, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
  eCAL::protobuf::CPublisher<robot_sdk::pb::SDKCmd> pub("sdk_cmd");

  // Collect candidate pairs from argv (skip the "sweep" keyword).
  std::vector<std::pair<int,int>> pairs;
  bool sweep = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "sweep") { sweep = true; continue; }
    auto c = a.find(':');
    if (c != std::string::npos) {
      pairs.emplace_back(std::stoi(a.substr(0, c)), std::stoi(a.substr(c + 1)));
    }
  }
  if (pairs.empty() && sweep) {
    // Built-in candidates to CONFIRM the translation-walk control_mode.
    //
    // Decoded from mc_ctrl (FSM_State_StandUp jump table @0x4f3de0): the wire
    // control_mode is the FSM selector -> stand=1, WALK=18 (RL_MIX/policy_mix_walk),
    // balance-stand/RPY=21. motion_mode is NOT propagated for RL_MIX walk, so mm
    // barely matters once control_mode=18. We therefore fix a walk-ish mm and vary
    // control_mode to prove 18 MOVES and 21 does not, plus a couple of mm variants
    // under 18 to confirm mm is a no-op there.
    pairs = {
      {1, 10},    // stand baseline           -> expect NO translation
      {18, 1},    // RL_MIX / policy_mix_walk -> expect *** MOVED ***
      {18, 0},    // walk, mm=0               -> expect MOVED (mm ignored under RL_MIX)
      {18, 100},  // walk, mm=100             -> expect MOVED (mm ignored under RL_MIX)
      {21, 100},  // balance-stand / RPY      -> expect NO translation (control)
    };
  }

  std::printf("find_walk_mode: sub 'sdk_robotstate', pub 'sdk_cmd'. "
              "Waiting 1.5s for state...\n");
  std::fflush(stdout);
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  if (pairs.empty()) {
    // OBSERVE mode: just print pose + modes for 6s, send nothing.
    std::printf("OBSERVE (no candidates given). Printing pose + modes for 6s. "
                "Pass 'sweep' or cm:mm pairs to drive.\n");
    auto t0 = std::chrono::steady_clock::now();
    while (eCAL::Ok() &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < 6.0) {
      float p[3], v[3]; bool h = false; snapshot(p, v, &h);
      std::printf("  cur_ctrl=%d cur_motion=%d  pos=(%.2f,%.2f,%.2f) |v|=%.3f\n",
                  g_cur_ctrl.load(), g_cur_motion.load(), p[0], p[1], p[2],
                  std::sqrt(v[0]*v[0]+v[1]*v[1]));
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    eCAL::Finalize();
    return 0;
  }

  std::printf("\n*** SAFETY: sim only, robot STANDING, clear space ahead. ***\n");
  std::printf("Sweeping %zu candidate(s). Ctrl-C to abort.\n", pairs.size());
  std::fflush(stdout);

  for (auto& pr : pairs) {
    if (!eCAL::Ok()) break;
    test_pair(pub, pr.first, pr.second, vx, drive_s, settle_s);
    std::fflush(stdout);
  }

  std::printf("\nDone. Each pair whose VERDICT says MOVED is a locomotion mode.\n"
              "Compare mean|v| across the movers: higher speed = faster gait\n"
              "(mix_walk vs gait_walk). Use  CMD_MOTION_MODE=<mm>  to select it.\n");
  eCAL::Finalize();
  return 0;
}
