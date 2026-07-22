#!/usr/bin/env python3
"""Publish a base_footprint frame for MATRiX quadrupeds.

Two modes (parameter `mode`):

  projection  -- base_footprint is base_link projected onto the gravity-horizontal
                 plane of `odom`, keeping only yaw. Uses only the real
                 odom -> base_link transform (from robot_forward), so it is
                 correct on flat and sloped terrain regardless of joint data.
                 Published as odom -> base_footprint.

  footplane   -- base_footprint is defined by the plane through the four foot
                 links (support plane). Its +z is the plane normal, origin is
                 base_link projected onto the plane. Requires the leg tf chain,
                 i.e. a real /joint_states source; with zero joints the feet are
                 fixed to the body and the result tilts with the body.
                 Published as base_link -> base_footprint.

Run:
  ros2 run <pkg> base_footprint_publisher.py            # if installed in a pkg
  python3 scripts/base_footprint_publisher.py --ros-args -p mode:=projection
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_ros


def quat_from_matrix(r):
    """Rotation matrix (3x3) -> quaternion [x, y, z, w]."""
    t = np.trace(r)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class BaseFootprintPublisher(Node):
    def __init__(self):
        super().__init__("base_footprint_publisher")

        self.declare_parameter("mode", "projection")  # projection | footplane
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("footprint_frame", "base_footprint")
        self.declare_parameter("foot_frames", [
            "FL_FOOT_LINK", "FR_FOOT_LINK", "RR_FOOT_LINK", "RL_FOOT_LINK",
        ])
        # Heading (+x) points from the rear-feet midpoint to the front-feet midpoint.
        self.declare_parameter("front_feet", ["FL_FOOT_LINK", "FR_FOOT_LINK"])
        self.declare_parameter("rear_feet", ["RR_FOOT_LINK", "RL_FOOT_LINK"])
        self.declare_parameter("rate_hz", 50.0)

        self.mode = self.get_parameter("mode").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.footprint_frame = self.get_parameter("footprint_frame").value
        self.foot_frames = list(self.get_parameter("foot_frames").value)
        self.front_feet = list(self.get_parameter("front_feet").value)
        self.rear_feet = list(self.get_parameter("rear_feet").value)
        rate = float(self.get_parameter("rate_hz").value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f"base_footprint mode={self.mode} "
            f"({'odom->' if self.mode == 'projection' else 'base_link->'}{self.footprint_frame})"
        )

    def on_timer(self):
        if self.mode == "footplane":
            self.publish_footplane()
        else:
            self.publish_projection()

    # --- projection: gravity-horizontal ground frame under base_link ---------
    def publish_projection(self):
        try:
            tf = self.buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"waiting for {self.odom_frame}->{self.base_frame}: {e}",
                                    throttle_duration_sec=2.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        # yaw of base_link in odom
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.footprint_frame
        out.transform.translation.x = t.x
        out.transform.translation.y = t.y
        out.transform.translation.z = 0.0  # projected to ground plane
        out.transform.rotation.z = math.sin(yaw / 2.0)
        out.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster.sendTransform(out)

    # --- footplane: frame at the four-foot centroid ------------------------
    def publish_footplane(self):
        feet = {}
        for f in self.foot_frames:
            try:
                tf = self.buffer.lookup_transform(
                    self.base_frame, f, rclpy.time.Time(),
                    timeout=Duration(seconds=0.1))
            except tf2_ros.TransformException as e:
                self.get_logger().warn(
                    f"waiting for {self.base_frame}->{f}: {e} "
                    f"(need /joint_states for the leg chain)",
                    throttle_duration_sec=2.0)
                return
            tr = tf.transform.translation
            feet[f] = np.array([tr.x, tr.y, tr.z])

        p = np.array([feet[f] for f in self.foot_frames])  # 4x3 in base_link
        origin = p.mean(axis=0)                              # centroid of the 4 feet

        # z: support-plane normal (least-significant SVD direction), pointing up.
        _, _, vh = np.linalg.svd(p - origin)
        n = vh[2, :]
        if n[2] < 0.0:
            n = -n
        n = n / np.linalg.norm(n)

        # Heading (+x): rear-feet midpoint -> front-feet midpoint, projected
        # onto the support plane so x _|_ z exactly.
        front = np.mean([feet[f] for f in self.front_feet], axis=0)
        rear = np.mean([feet[f] for f in self.rear_feet], axis=0)
        fwd = front - rear
        x_axis = fwd - np.dot(fwd, n) * n
        if np.linalg.norm(x_axis) < 1e-6:
            # Degenerate (feet collinear front/back): fall back to base_link x.
            x_axis = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], n) * n
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(n, x_axis)
        rot = np.column_stack((x_axis, y_axis, n))
        q = quat_from_matrix(rot)

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.child_frame_id = self.footprint_frame
        out.transform.translation.x = float(origin[0])
        out.transform.translation.y = float(origin[1])
        out.transform.translation.z = float(origin[2])
        out.transform.rotation.x = float(q[0])
        out.transform.rotation.y = float(q[1])
        out.transform.rotation.z = float(q[2])
        out.transform.rotation.w = float(q[3])
        self.broadcaster.sendTransform(out)


def main():
    rclpy.init()
    node = BaseFootprintPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
