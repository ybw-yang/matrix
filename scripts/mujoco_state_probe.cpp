// Standalone eCAL probe: subscribe to the sim's `mujoco_state` topic
// (robot_sdk::pb::RobotState) and print the leg joint angles. Used to verify
// the data path and calibrate leg index order before wiring into ROS.
//
// Build:
//   g++ -std=c++17 scripts/mujoco_state_probe.cpp -I/usr/include \
//       -o /tmp/mujoco_state_probe \
//       -lecal_core -lecal_core_pb -lprotobuf -lrobot_sdk -pthread
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include "robot_sdk.pb.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <thread>

static std::atomic<long> g_count{0};

static void OnState(const char* /*topic*/, const robot_sdk::pb::RobotState& s,
                    long long /*t*/, long long /*c*/, long long /*id*/) {
  long n = ++g_count;
  if (n % 200 != 1) return;  // downsample: print ~ every 200th message
  std::printf("[msg %ld] sizes: q_abad=%d q_hip=%d q_knee=%d\n",
              n, s.q_abad_size(), s.q_hip_size(), s.q_knee_size());
  auto row = [](const char* nm, int sz, const std::function<float(int)>& g) {
    std::printf("  %-7s:", nm);
    for (int i = 0; i < sz; ++i) std::printf(" % .4f", g(i));
    std::printf("\n");
  };
  row("q_abad", s.q_abad_size(), [&](int i) { return s.q_abad(i); });
  row("q_hip",  s.q_hip_size(),  [&](int i) { return s.q_hip(i); });
  row("q_knee", s.q_knee_size(), [&](int i) { return s.q_knee(i); });
  std::fflush(stdout);
}

int main(int argc, char** argv) {
  const std::string topic = (argc > 1) ? argv[1] : "leg_data";
  eCAL::Initialize(argc, argv, "mujoco_state_probe");
  eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotState> sub(topic);
  sub.AddReceiveCallback(std::bind(OnState, std::placeholders::_1,
                                   std::placeholders::_2, std::placeholders::_3,
                                   std::placeholders::_4, std::placeholders::_5));
  std::printf("subscribed to eCAL topic '%s'; waiting for data ...\n", topic.c_str());
  std::fflush(stdout);
  while (eCAL::Ok()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  eCAL::Finalize();
  return 0;
}
