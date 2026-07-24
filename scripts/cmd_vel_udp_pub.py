#!/usr/bin/env python3
"""Forward ROS 2 /cmd_vel (geometry_msgs/Twist) to cmd_vel_ecal_bridge over UDP.

Packet: b"VEL1" + float32 vx + float32 vy + float32 yaw_rate (little-endian),
mapping linear.x->vx, linear.y->vy, angular.z->yaw_rate.

Publishes at a steady rate using the last received Twist so the command stays
alive; stops (zero) if no Twist arrives for `timeout` seconds.
"""
import socket
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelUdpPub(Node):
    def __init__(self):
        super().__init__("cmd_vel_udp_pub")
        self.declare_parameter("port", 25999)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("timeout", 0.5)
        port = int(self.get_parameter("port").value)
        rate = float(self.get_parameter("rate_hz").value)
        self.timeout = float(self.get_parameter("timeout").value)

        self.addr = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vx = self.vy = self.yaw = 0.0
        self.last = None

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f"/cmd_vel -> udp 127.0.0.1:{port} @ {rate:.0f}Hz "
            f"(linear.x->vx, linear.y->vy, angular.z->yaw_rate)")

    def on_cmd(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.yaw = msg.angular.z
        self.last = self.get_clock().now()

    def on_timer(self):
        vx, vy, yaw = self.vx, self.vy, self.yaw
        if self.last is None:
            return  # nothing received yet -> send nothing (bridge zeroes on stale)
        age = (self.get_clock().now() - self.last).nanoseconds * 1e-9
        if age > self.timeout:
            vx = vy = yaw = 0.0
        self.sock.sendto(b"VEL1" + struct.pack("<3f", vx, vy, yaw), self.addr)


def main():
    rclpy.init()
    node = CmdVelUdpPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
