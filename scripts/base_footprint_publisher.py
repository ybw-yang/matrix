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
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
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
        # Foot contact is ESTIMATED from foot height (the sim exposes no contact
        # signal): a foot within `contact_threshold` metres of the lowest foot is
        # treated as in stance (touching ground). base_footprint (footplane mode)
        # is then computed from the contact feet only.
        self.declare_parameter("contact_threshold", 0.03)
        self.declare_parameter("publish_contacts", True)
        self.declare_parameter("contacts_topic", "/foot_contacts")
        # contact_mode:
        #   height   - foot within contact_threshold of the LOWEST foot. Simple,
        #              but only valid on flat ground (fails on slopes/steps where
        #              stance feet are legitimately at different heights).
        #   velocity - foot is stance if its speed in the odom frame is below
        #              contact_speed (a planted foot doesn't move). Terrain-
        #              independent: works on slopes and steps. Needs odom->foot tf.
        self.declare_parameter("contact_mode", "height")
        self.declare_parameter("contact_speed", 0.05)  # m/s, for velocity mode
        # Smoothing: gait motion makes the raw footplane jitter (contact set
        # switching, swing-foot motion, SVD normal noise). Low-pass the output
        # pose with time constant smooth_tau (s); larger = smoother but laggier.
        # 0 disables. `footprint_feet` chooses which feet define the centre:
        #   contact - only stance feet (support polygon centre; can jump)
        #   all     - all four feet (steadier "geometric" four-foot centre)
        self.declare_parameter("smooth_tau", 0.25)
        self.declare_parameter("footprint_feet", "contact")  # contact | all

        self.mode = self.get_parameter("mode").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.footprint_frame = self.get_parameter("footprint_frame").value
        self.foot_frames = list(self.get_parameter("foot_frames").value)
        self.front_feet = list(self.get_parameter("front_feet").value)
        self.rear_feet = list(self.get_parameter("rear_feet").value)
        self.contact_threshold = float(self.get_parameter("contact_threshold").value)
        self.contact_mode = self.get_parameter("contact_mode").value
        self.contact_speed = float(self.get_parameter("contact_speed").value)
        self.footprint_feet = self.get_parameter("footprint_feet").value
        rate = float(self.get_parameter("rate_hz").value)

        # EMA coefficient from time constant and the (fixed) timer period.
        tau = float(self.get_parameter("smooth_tau").value)
        dt = 1.0 / rate
        self.alpha = 1.0 if tau <= 0.0 else dt / (tau + dt)
        self._sm_pos = None
        self._sm_quat = None

        self._last_foot_odom = {}  # frame -> (np.array pos, Time) for velocity mode
        self._last_speed = {}      # frame -> last computed speed

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.contact_pub = None
        if bool(self.get_parameter("publish_contacts").value):
            self.contact_pub = self.create_publisher(
                Float32MultiArray, self.get_parameter("contacts_topic").value, 10)

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

    def detect_contacts(self, p_base):
        """Return a 4-bool contact array in foot_frames order."""
        if self.contact_mode == "velocity":
            c = self.contacts_by_velocity()
            if c is not None:
                return c
            self.get_logger().warn(
                "velocity contact mode needs odom->foot tf; falling back to height",
                throttle_duration_sec=5.0)
        zs = p_base[:, 2]
        return zs <= (zs.min() + self.contact_threshold)

    def contacts_by_velocity(self):
        """Stance = low speed in the odom frame (terrain-independent). None if
        odom->foot tf is unavailable (caller falls back to height)."""
        now = self.get_clock().now()
        speeds = []
        for f in self.foot_frames:
            try:
                tf = self.buffer.lookup_transform(
                    self.odom_frame, f, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            except tf2_ros.TransformException:
                return None
            t = tf.transform.translation
            pos = np.array([t.x, t.y, t.z])
            prev = self._last_foot_odom.get(f)
            self._last_foot_odom[f] = (pos, now)
            if prev is None:
                speeds.append(0.0)  # first sample: assume stance
                continue
            dt = (now - prev[1]).nanoseconds * 1e-9
            if dt <= 1e-4:
                speeds.append(self._last_speed.get(f, 0.0))
                continue
            v = float(np.linalg.norm(pos - prev[0]) / dt)
            self._last_speed[f] = v
            speeds.append(v)
        return np.array(speeds) < self.contact_speed

    def publish_contacts(self, contact):
        """Publish per-foot contact (1.0=stance, 0.0=swing) in foot_frames order."""
        if self.contact_pub is None:
            return
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = ",".join(self.foot_frames)  # e.g. FL,FR,RR,RL
        dim.size = len(self.foot_frames)
        dim.stride = len(self.foot_frames)
        msg.layout.dim = [dim]
        msg.data = [1.0 if c else 0.0 for c in contact]
        self.contact_pub.publish(msg)

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

        # --- contact estimate (height on flat ground / velocity for terrain) --
        contact = self.detect_contacts(p)
        contact_of = {f: bool(contact[i]) for i, f in enumerate(self.foot_frames)}
        self.publish_contacts(contact)

        # Which feet define the support plane: contact-only (support polygon) or
        # all four (steadier). Contacts are still published either way.
        use_contact = self.footprint_feet != "all"
        stance = p[contact] if use_contact else p

        # Support-plane normal (base_link frame). Need >=3 non-collinear points;
        # if fewer feet are selected, fall back to all four so it stays defined.
        plane_pts = stance if len(stance) >= 3 else p
        c = plane_pts.mean(axis=0)               # a point on the support plane
        _, _, vh = np.linalg.svd(plane_pts - c)
        n = vh[2, :]
        if n[2] < 0.0:
            n = -n
        n = n / np.linalg.norm(n)

        # Origin: base_footprint must lie on base_link's OWN normal (its z-axis) —
        # directly under the body — not at the laterally-offset foot centroid.
        # Intersect the base_link z-axis {(0,0,t)} with the support plane
        # (point c, normal n):  n.((0,0,t) - c) = 0  ->  t = (n.c)/n_z.
        if abs(n[2]) > 1e-6:
            origin = np.array([0.0, 0.0, float(n @ c) / float(n[2])])
        else:
            origin = np.array([0.0, 0.0, float(c[2])])

        # Heading (+x): rear-feet midpoint -> front-feet midpoint. Prefer selected
        # feet, but fall back to all if a whole end is airborne (heading undefined).
        keep = (lambda f: contact_of.get(f)) if use_contact else (lambda f: True)
        front_c = [feet[f] for f in self.front_feet if keep(f)] or \
                  [feet[f] for f in self.front_feet]
        rear_c = [feet[f] for f in self.rear_feet if keep(f)] or \
                 [feet[f] for f in self.rear_feet]
        front = np.mean(front_c, axis=0)
        rear = np.mean(rear_c, axis=0)
        fwd = front - rear
        x_axis = fwd - np.dot(fwd, n) * n
        if np.linalg.norm(x_axis) < 1e-6:
            # Degenerate (feet collinear front/back): fall back to base_link x.
            x_axis = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], n) * n
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(n, x_axis)
        rot = np.column_stack((x_axis, y_axis, n))
        q = quat_from_matrix(rot)

        # Low-pass the pose so gait motion doesn't jitter base_footprint.
        origin, q = self.smooth_pose(origin, q)

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

    def smooth_pose(self, pos, quat):
        """EMA on translation + normalized-lerp on rotation (alpha from smooth_tau)."""
        a = self.alpha
        quat = np.asarray(quat, dtype=float)
        if a >= 1.0:
            return pos, quat
        if self._sm_pos is None:
            self._sm_pos = np.array(pos, dtype=float)
            self._sm_quat = quat.copy()
            return self._sm_pos, self._sm_quat
        self._sm_pos = (1.0 - a) * self._sm_pos + a * np.asarray(pos, dtype=float)
        if np.dot(self._sm_quat, quat) < 0.0:  # nearest representation
            quat = -quat
        q = (1.0 - a) * self._sm_quat + a * quat
        self._sm_quat = q / np.linalg.norm(q)
        return self._sm_pos, self._sm_quat


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
