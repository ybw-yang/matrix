#!/usr/bin/env python3
"""Publish odom -> base_link TF from /odom/mujoco_odom with the FULL orientation.

Replaces robot_forward's `odom -> base_link`, which flattens roll/pitch to zero
(only yaw survives), leaving base_link horizontal even when the body is tilted on
a slope or steps. /odom/mujoco_odom already carries the true body pose (verified:
its rpy matches /imu exactly, e.g. pitch=-20.6deg on a step), so we just relay its
pose straight into the transform.

Run this INSTEAD of robot_forward. The sim publishes odom on a best-effort QoS.

Usage:
  python3 scripts/odom_to_tf.py --ros-args \
      -p in_topic:=/odom/mujoco_odom -p odom_frame:=odom -p base_frame:=base_link
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.declare_parameter("in_topic", "/odom/mujoco_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        in_topic = self.get_parameter("in_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.br = TransformBroadcaster(self)
        # Sim odom is best-effort; match it or we get nothing.
        self.create_subscription(Odometry, in_topic, self.on_odom,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f"odom_to_tf: {in_topic}.pose -> TF {self.odom_frame} -> "
            f"{self.base_frame} (full 6-DoF, keeps roll/pitch)")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = TransformStamped()
        # Keep the sim's own stamp so tf lookups line up with other sim data.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = q  # full orientation, unlike robot_forward
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
