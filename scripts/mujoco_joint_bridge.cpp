// mujoco_joint_bridge: subscribe to the MC's `leg_data` eCAL topic
// (robot_sdk::pb::RobotState) and relay the 12 leg joint angles + velocities
// to a local UDP port, where joint_state_udp_bridge.py turns them into a ROS 2
// /joint_states message. eCAL (C++ only) is kept out of the ROS process.
//
// Packet layout (little-endian):
//   char[4]  magic   = "JNT1"
//   uint32   njoint  = 12
//   float32  pos[12] = q_abad[0..3], q_hip[0..3], q_knee[0..3]
//   float32  vel[12] = qd_abad[0..3], qd_hip[0..3], qd_knee[0..3]
//
// Build:
//   g++ -std=c++17 scripts/mujoco_joint_bridge.cpp -I/usr/include \
//       -o /tmp/mujoco_joint_bridge \
//       -lecal_core -lecal_core_pb -lprotobuf -lrobot_sdk -pthread
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include "robot_sdk.pb.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <thread>

static int   g_sock = -1;
static sockaddr_in g_dst{};

static void OnState(const char*, const robot_sdk::pb::RobotState& s,
                    long long, long long, long long) {
  if (s.q_abad_size() < 4 || s.q_hip_size() < 4 || s.q_knee_size() < 4) return;

  uint8_t buf[8 + 24 * sizeof(float)];
  std::memcpy(buf, "JNT1", 4);
  uint32_t nj = 12;
  std::memcpy(buf + 4, &nj, 4);
  float* pos = reinterpret_cast<float*>(buf + 8);
  float* vel = pos + 12;
  const bool have_vel =
      s.qd_abad_size() >= 4 && s.qd_hip_size() >= 4 && s.qd_knee_size() >= 4;
  for (int i = 0; i < 4; ++i) {
    pos[0 + i] = s.q_abad(i);
    pos[4 + i] = s.q_hip(i);
    pos[8 + i] = s.q_knee(i);
    vel[0 + i] = have_vel ? s.qd_abad(i) : 0.f;
    vel[4 + i] = have_vel ? s.qd_hip(i) : 0.f;
    vel[8 + i] = have_vel ? s.qd_knee(i) : 0.f;
  }
  sendto(g_sock, buf, sizeof(buf), 0,
         reinterpret_cast<sockaddr*>(&g_dst), sizeof(g_dst));
}

int main(int argc, char** argv) {
  const std::string topic = (argc > 1) ? argv[1] : "leg_data";
  const uint16_t    port  = (argc > 2) ? static_cast<uint16_t>(std::stoi(argv[2])) : 25998;

  g_sock = socket(AF_INET, SOCK_DGRAM, 0);
  g_dst.sin_family = AF_INET;
  g_dst.sin_port = htons(port);
  inet_pton(AF_INET, "127.0.0.1", &g_dst.sin_addr);

  eCAL::Initialize(argc, argv, "mujoco_joint_bridge");
  eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotState> sub(topic);
  sub.AddReceiveCallback(std::bind(OnState, std::placeholders::_1,
                                   std::placeholders::_2, std::placeholders::_3,
                                   std::placeholders::_4, std::placeholders::_5));
  std::printf("mujoco_joint_bridge: eCAL '%s' -> udp 127.0.0.1:%u\n",
              topic.c_str(), port);
  std::fflush(stdout);
  while (eCAL::Ok()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  eCAL::Finalize();
  if (g_sock >= 0) close(g_sock);
  return 0;
}
