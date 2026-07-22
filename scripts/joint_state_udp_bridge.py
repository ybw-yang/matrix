#!/usr/bin/env python3
"""Receive leg joint angles from mujoco_joint_bridge (UDP) and publish
sensor_msgs/JointState on /joint_states, so robot_state_publisher can broadcast
the real (not zero) leg tf for the four legs.

Pairs with scripts/mujoco_joint_bridge.cpp. Packet: "JNT1" + uint32 njoint +
12 float32 positions + 12 float32 velocities, each block ordered
[abad x4 legs, hip x4 legs, knee x4 legs].

The leg index order in the protobuf is mapped to URDF leg names via `leg_order`
(default FL,FR,RR,RL = the URDF joint order). If a leg looks wrong in RViz,
adjust leg_order; if a joint is mirrored, list it in `flip_joints`.
"""
import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

MAGIC = b"JNT1"
JOINT_TYPES = ["ABAD", "HIP", "KNEE"]  # matches pos block order abad/hip/knee


class JointStateUdpBridge(Node):
    def __init__(self):
        super().__init__("joint_state_udp_bridge")
        self.declare_parameter("port", 25998)
        self.declare_parameter("leg_order", ["FL", "FR", "RR", "RL"])
        # Sign convention: URDF legs are all kinematically identical. The
        # sim/controller reports hip+knee with the opposite sign to the URDF for
        # ALL legs, and treats the REAR legs as rotated 180deg about z, so the
        # rear abad is also inverted. Fix:
        #   - negate all 4 HIP and all 4 KNEE  (legs tuck inward, front & rear)
        #   - negate the 2 REAR ABAD           (rear feet splay outward, symmetric)
        # Front abad already matches, so it is left as-is.
        self.declare_parameter("flip_joints", [
            "FL_HIP_JOINT", "FL_KNEE_JOINT",
            "FR_HIP_JOINT", "FR_KNEE_JOINT",
            "RR_HIP_JOINT", "RR_KNEE_JOINT", "RR_ABAD_JOINT",
            "RL_HIP_JOINT", "RL_KNEE_JOINT", "RL_ABAD_JOINT",
        ])
        self.declare_parameter("joint_suffix", "_JOINT")

        port = int(self.get_parameter("port").value)
        self.legs = list(self.get_parameter("leg_order").value)
        self.flip = set(j for j in self.get_parameter("flip_joints").value if j)
        suffix = self.get_parameter("joint_suffix").value

        # Joint names in the same order as the packed 12 positions:
        # block 0 = abad for legs[0..3], block 1 = hip, block 2 = knee.
        self.names = [f"{leg}_{jt}{suffix}"
                      for jt in JOINT_TYPES for leg in self.legs]
        self.signs = [-1.0 if n in self.flip else 1.0 for n in self.names]

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))
        self.sock.setblocking(False)
        self.create_timer(0.002, self.poll)  # 500 Hz drain
        self.get_logger().info(
            f"joint_state_udp_bridge on udp 127.0.0.1:{port}; joints={self.names}")

    def poll(self):
        latest = None
        while True:  # drain to the newest packet
            try:
                latest = self.sock.recv(4096)
            except BlockingIOError:
                break
        if latest is None:
            return
        if len(latest) < 8 or latest[:4] != MAGIC:
            return
        (nj,) = struct.unpack_from("<I", latest, 4)
        need = 8 + nj * 2 * 4
        if nj != 12 or len(latest) < need:
            return
        pos = struct.unpack_from(f"<{nj}f", latest, 8)
        vel = struct.unpack_from(f"<{nj}f", latest, 8 + nj * 4)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [self.signs[i] * pos[i] for i in range(nj)]
        msg.velocity = [self.signs[i] * vel[i] for i in range(nj)]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointStateUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
