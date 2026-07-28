// cmd_vel_ecal_bridge: ROS /cmd_vel (relayed over localhost UDP) -> eCAL SDKCmd.
//
// Pairs with cmd_vel_udp_pub.py, which forwards geometry_msgs/Twist as a UDP
// packet:  "VEL1" + float32 vx + float32 vy + float32 yaw_rate  (little-endian).
//
// It publishes robot_sdk::pb::SDKCmd on the eCAL topic "sdk_cmd" at a fixed rate,
// carrying vx/vy/yaw_rate plus the control_mode / motion_mode you pass in. It also
// subscribes to "sdk_robotstate" and prints the robot's CURRENT cur_ctrl_mode /
// cur_motion_mode whenever they change -- use that to calibrate the modes.
//
// SAFETY: without an explicit control_mode (>=0) it runs in OBSERVE mode and does
// NOT publish any command -- it only prints the current mode. Drive the robot into
// a walking state with the keyboard, read the mode here, then re-run with those
// values to enable velocity control. If no fresh /cmd_vel arrives for `timeout`
// seconds it publishes zero velocity (safety stop).
//
// XG SDK-PATH MODES. control_mode is the FSM selector: handleROSSDKCommand()
// writes the raw wire control_mode into RobotControlParameters (offset 0x98) and
// each FSM state's checkTransition() maps it (jump table @ mc_ctrl 0x4f3de0) to
// an FSM_StateName. Only {0,1,18,21,51} are recognized; other values are ignored.
//   passive            control_mode=0
//   stand              control_mode=1   motion_mode=10   (STAND_UP; hold to stand)
//   WALK (translate)   control_mode=18  motion_mode=1    <- RL_MIX / policy_mix_walk
//   balance-stand/RPY  control_mode=21  motion_mode=100  <- in-place body pose, NOT walk
//   joint_pd           control_mode=51
// 21=balance-stand is EMPIRICALLY confirmed (the "twist" that does in-place RPY).
// 18=walk is decoded from the jump table (high confidence; verify with
// scripts/bin/find_walk_mode). For WALK, control_mode=18 + vx/vy/yaw_rate is
// sufficient -- the SDK handler does NOT propagate motion_mode for RL_MIX, so
// motion_mode=1(Walk) is only belt-and-suspenders. enable_control_mode is NOT read
// on the SDK path.
// So to DRIVE via /cmd_vel: stand the robot first (keyboard U -> STAND_UP), then
// run:  cmd_vel_ecal_bridge 25999 18 1  and publish /cmd_vel. mc_ctrl only enters
// RL_MIX from a standing state, so a lying/passive robot will not move even in
// PUBLISH mode. If it won't leave balance-stand, send control_mode=1 first, then 18.
//
// Build via scripts/build_joint_bridge.sh. Manual:
//   g++ -std=c++17 scripts/cmd_vel_ecal_bridge.cpp -I/usr/include \
//       -o scripts/bin/cmd_vel_ecal_bridge \
//       -lecal_core -lecal_core_pb -lprotobuf -lrobot_sdk -pthread
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include "robot_sdk.pb.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

static std::mutex g_mtx;
static float g_vx = 0.f, g_vy = 0.f, g_yaw = 0.f;
static std::chrono::steady_clock::time_point g_last_rx;
static std::atomic<int> g_cur_ctrl{-1}, g_cur_motion{-1};

static uint64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

static void OnState(const char*, const robot_sdk::pb::SDKRobotState& s,
                    long long, long long, long long) {
  int c = static_cast<int>(s.cur_ctrl_mode());
  int m = static_cast<int>(s.cur_motion_mode());
  if (c != g_cur_ctrl.load() || m != g_cur_motion.load()) {
    g_cur_ctrl = c; g_cur_motion = m;
    std::printf("[mode] cur_ctrl_mode=%d  cur_motion_mode=%d\n", c, m);
    std::fflush(stdout);
  }
}

int main(int argc, char** argv) {
  const uint16_t udp_port = (argc > 1) ? static_cast<uint16_t>(std::stoi(argv[1])) : 25999;
  const int control_mode  = (argc > 2) ? std::stoi(argv[2]) : -1;   // <0 => observe only
  const int motion_mode   = (argc > 3) ? std::stoi(argv[3]) : 0;
  const double rate_hz    = (argc > 4) ? std::stod(argv[4]) : 50.0;
  const double timeout_s  = 0.5;   // stale command -> zero velocity
  const bool publish = (control_mode >= 0);

  eCAL::Initialize(argc, argv, "cmd_vel_ecal_bridge");
  eCAL::protobuf::CSubscriber<robot_sdk::pb::SDKRobotState> sub("sdk_robotstate");
  sub.AddReceiveCallback(std::bind(OnState, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  if (!publish) {
    std::printf("OBSERVE mode: printing cur_ctrl_mode / cur_motion_mode from "
                "'sdk_robotstate'. NOT publishing any command.\n"
                "Drive the robot (keyboard) into a walking state, note the modes,\n"
                "then re-run: cmd_vel_ecal_bridge %u <control_mode> <motion_mode>\n",
                udp_port);
    std::fflush(stdout);
    while (eCAL::Ok()) std::this_thread::sleep_for(std::chrono::milliseconds(200));
    eCAL::Finalize();
    return 0;
  }

  // UDP receiver for {vx,vy,yaw} from the ROS side.
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(udp_port);
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  int one = 1; setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  int fl = fcntl(sock, F_GETFL, 0); fcntl(sock, F_SETFL, fl | O_NONBLOCK);
  g_last_rx = std::chrono::steady_clock::now() - std::chrono::seconds(10);

  std::thread rx([&] {
    uint8_t buf[64];
    while (eCAL::Ok()) {
      ssize_t n = recv(sock, buf, sizeof(buf), 0);
      if (n >= (ssize_t)(4 + 12) && std::memcmp(buf, "VEL1", 4) == 0) {
        float v[3];
        std::memcpy(v, buf + 4, 12);
        std::lock_guard<std::mutex> lk(g_mtx);
        g_vx = v[0]; g_vy = v[1]; g_yaw = v[2];
        g_last_rx = std::chrono::steady_clock::now();
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  });

  eCAL::protobuf::CPublisher<robot_sdk::pb::SDKCmd> pub("sdk_cmd");
  std::printf("PUBLISH mode: /cmd_vel(udp:%u) -> eCAL 'sdk_cmd' "
              "control_mode=%d motion_mode=%d @ %.0fHz\n",
              udp_port, control_mode, motion_mode, rate_hz);
  std::fflush(stdout);

  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  while (eCAL::Ok()) {
    float vx, vy, yaw;
    bool fresh;
    {
      std::lock_guard<std::mutex> lk(g_mtx);
      auto age = std::chrono::steady_clock::now() - g_last_rx;
      fresh = std::chrono::duration<double>(age).count() < timeout_s;
      vx = fresh ? g_vx : 0.f;
      vy = fresh ? g_vy : 0.f;
      yaw = fresh ? g_yaw : 0.f;
    }
    robot_sdk::pb::SDKCmd cmd;
    cmd.set_vx(vx);
    cmd.set_vy(vy);
    cmd.set_yaw_rate(yaw);
    cmd.set_control_mode(static_cast<uint32_t>(control_mode));
    cmd.set_motion_mode(static_cast<uint32_t>(motion_mode));
    cmd.set_time_stamp(now_ns());
    pub.Send(cmd);
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::microseconds>(period));
  }

  rx.join();
  eCAL::Finalize();
  close(sock);
  return 0;
}
