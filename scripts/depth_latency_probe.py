#!/usr/bin/env python3
"""Estimate the depth sensor's timestamp latency (delta) by geometric self-consistency.

Idea (see design discussion): a single plane can't fix a full 6-DoF pose (it's
symmetric under in-plane translation + rotation about its normal), so DON'T
back-solve the pose. Instead reduce the unknown to ONE scalar -- the stamp
latency delta -- and let the REAL odom trajectory supply pose(t-delta). Sweep
delta and pick the value that makes the observed geometry most self-consistent.

A static wall's normal, expressed in the fixed odom frame, must be identical
across frames. If the depth stamp trails its content by delta, looking up the
TF at the (too-late) stamp rotates each frame's wall by ~omega*delta, so the
per-frame normals FAN OUT. The delta that collapses that fan is the latency.

  cost(delta) = angular variance of { n_odom_k(delta) }         (primary)
  where  n_odom_k(delta) = R_odom_optical(t_k - delta) * n_opt_k

Efficiency: each frame's plane normal n_opt_k is fit ONCE in the optical frame
(delta-independent); sweeping delta only re-ROTATES that stored normal -- no
re-fitting per delta.

Corroborator: accumulate the window's clouds at pose(t-delta), RANSAC a single
plane, report inlier RMS thickness. Thin at the true delta, thick when the fan
is open. Neither metric needs the wall's true equation -- only self-consistency.

Gating: only frames taken while |omega| exceeds a threshold carry information
(signal ~ range*omega*delta). Static frames are dropped.

Usage:
  ros2 run ... OR:  python3 depth_latency_probe.py --ros-args \
      -p depth_topic:=/front_depth/image \
      -p info_topic:=/front_depth/camera_info \
      -p optical_frame:=front_optical -p odom_frame:=odom \
      -p base_frame:=base_link \
      -p delta_max_ms:=150.0 -p delta_step_ms:=2.0 \
      -p min_omega:=0.15 -p window_sec:=6.0

Then rotate the robot in front of a wall (a slow sweep + a fast sweep). The node
prints delta* every window_sec. IMPORTANT: run with the fixup's stamp_offset_ms
set to 0 to measure the RAW latency; a nonzero offset is already baked into the
stamp, so the probe would then report the RESIDUAL (near 0 if 50ms was right).
"""
import math
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

import tf2_ros


def _fit_plane_normal(pts, iters=60, thresh=0.02, min_inliers=80):
    """RANSAC a single plane through pts (N,3); return (unit_normal, inlier_rms)
    or (None, None). Sign is left arbitrary on purpose -- the delta cost below is
    built to be invariant to +/-n (see _normal_dispersion), so we must NOT try to
    canonicalize the sign: doing so flips unstably when the wall's normal sweeps
    near a hemisphere boundary and falsely flattens the cost curve."""
    n = pts.shape[0]
    if n < min_inliers:
        return None, None
    best_inliers = None
    best_normal = None
    rng = np.random.default_rng(0)  # deterministic (no Math.random-style flakiness)
    for _ in range(iters):
        idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = pts[idx]
        v = np.cross(p1 - p0, p2 - p0)
        nv = np.linalg.norm(v)
        if nv < 1e-9:
            continue
        v = v / nv
        d = pts @ v - (v @ p0)
        inl = np.abs(d) < thresh
        cnt = int(inl.sum())
        if best_inliers is None or cnt > best_inliers.sum():
            best_inliers = inl
            best_normal = v
    if best_inliers is None or best_inliers.sum() < min_inliers:
        return None, None
    # Refit on inliers via PCA (smallest-eigenvector = normal), robust final fit.
    inl_pts = pts[best_inliers]
    c = inl_pts.mean(axis=0)
    u, s, vt = np.linalg.svd(inl_pts - c, full_matrices=False)
    normal = vt[-1]
    normal = normal / (np.linalg.norm(normal) + 1e-12)
    resid = (inl_pts - c) @ normal
    rms = float(np.sqrt(np.mean(resid ** 2)))
    return normal, rms


def _normal_dispersion(normals):
    """Spread of unit normals, INVARIANT to +/-n sign. For coincident normals the
    scatter matrix S = mean(n n^T) has one eigenvalue ~1 and the rest ~0; as the
    normals fan out the 2nd-largest eigenvalue grows. Return that 2nd eigenvalue
    (0 = perfectly consistent). Unlike a mean-angle variance it needs no sign
    convention, so a wall sweeping a wide arc can't flip it into a false plateau."""
    N = np.asarray(normals)
    S = (N.T @ N) / N.shape[0]
    w = np.linalg.eigvalsh(S)          # ascending
    return float(w[-2])                # 2nd largest


class DepthLatencyProbe(Node):
    def __init__(self):
        super().__init__("depth_latency_probe")
        self.declare_parameter("depth_topic", "/front_depth/image")
        self.declare_parameter("info_topic", "/front_depth/camera_info")
        self.declare_parameter("optical_frame", "front_optical")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("delta_max_ms", 150.0)
        self.declare_parameter("delta_step_ms", 2.0)
        self.declare_parameter("min_omega", 0.15)      # rad/s gate
        self.declare_parameter("window_sec", 6.0)      # report cadence
        self.declare_parameter("skip_cell", 6)         # pixel downsample
        self.declare_parameter("d_min", 0.3)
        self.declare_parameter("d_max", 6.0)
        self.declare_parameter("ransac_thresh", 0.02)  # m
        self.declare_parameter("max_frames", 60)       # ring buffer cap
        # A yaw-latency signal lives ONLY in vertical walls: rotating about odom-z
        # doesn't move a z-aligned (floor/ceiling) normal, so those frames carry no
        # info AND corrupt the dispersion. Keep frames whose odom normal is near
        # horizontal (|n.z| < wall_tilt_max), then keep only the dominant wall
        # (reject normals more than wall_consistency_deg off the median direction).
        self.declare_parameter("wall_tilt_max", 0.5)         # reject floor/ceiling
        self.declare_parameter("wall_consistency_deg", 25.0)  # reject off-wall surfaces

        self.optical = self.get_parameter("optical_frame").value
        self.odom = self.get_parameter("odom_frame").value
        self.base = self.get_parameter("base_frame").value
        self.dmax_ms = float(self.get_parameter("delta_max_ms").value)
        self.dstep_ms = float(self.get_parameter("delta_step_ms").value)
        self.min_omega = float(self.get_parameter("min_omega").value)
        self.window = float(self.get_parameter("window_sec").value)
        self.skip = max(1, int(self.get_parameter("skip_cell").value))
        self.d_min = float(self.get_parameter("d_min").value)
        self.d_max = float(self.get_parameter("d_max").value)
        self.thresh = float(self.get_parameter("ransac_thresh").value)
        self.max_frames = int(self.get_parameter("max_frames").value)
        self.wall_tilt_max = float(self.get_parameter("wall_tilt_max").value)
        self.wall_consistency = math.radians(
            float(self.get_parameter("wall_consistency_deg").value))

        self.K = None                 # (fx,fy,cx,cy)
        # ring buffer of per-frame records: (stamp_Time, n_opt(3,), pts_opt(N,3))
        self.frames = deque(maxlen=self.max_frames)
        self.n_floor_reject = 0       # frames dropped as floor/ceiling (diagnostic)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.get_parameter("info_topic").value,
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("depth_topic").value,
                                 self.on_depth, qos_profile_sensor_data)
        self.create_timer(self.window, self.on_report)

        self.deltas_ms = np.arange(0.0, self.dmax_ms + 1e-6, self.dstep_ms)
        self.get_logger().info(
            f"depth_latency_probe: sweep delta 0..{self.dmax_ms}ms step {self.dstep_ms}ms, "
            f"gate |omega|>{self.min_omega} rad/s, report every {self.window}s. "
            f"ROTATE the robot in front of a wall. (set fixup stamp_offset_ms:=0 "
            f"to read RAW latency.)")

    def on_info(self, msg: CameraInfo):
        if self.K is None:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.get_logger().info(f"got intrinsics fx={self.K[0]:.1f} fy={self.K[1]:.1f} "
                                   f"cx={self.K[2]:.1f} cy={self.K[3]:.1f}")

    def _omega_at(self, stamp: Time):
        """Angular speed |omega| (rad/s) of base in odom around stamp, via two
        TF samples +/- dt. Returns None if TF unavailable."""
        dt = 0.05
        try:
            t0 = self.tf_buffer.lookup_transform(
                self.odom, self.base, stamp - Duration(seconds=dt))
            t1 = self.tf_buffer.lookup_transform(
                self.odom, self.base, stamp + Duration(seconds=dt))
        except Exception:
            return None
        q0 = t0.transform.rotation
        q1 = t1.transform.rotation
        r0 = Rotation.from_quat([q0.x, q0.y, q0.z, q0.w])
        r1 = Rotation.from_quat([q1.x, q1.y, q1.z, q1.w])
        rel = (r0.inv() * r1).magnitude()   # geodesic angle
        return rel / (2 * dt)

    def on_depth(self, msg: Image):
        if self.K is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        # gate on rotation speed first (cheap) -- skip static frames
        omega = self._omega_at(stamp)
        if omega is None or omega < self.min_omega:
            return
        pts = self._to_points(msg)
        if pts is None or pts.shape[0] < 100:
            return
        n_opt, rms = _fit_plane_normal(pts, thresh=self.thresh)
        if n_opt is None:
            return
        # Vertical-wall gate: rotate the normal into odom at the frame's OWN stamp
        # (delta=0 is fine as a coarse classifier -- a ~50ms error can't flip
        # floor<->wall) and drop near-horizontal surfaces (floor/ceiling), which
        # carry no yaw-latency signal and would corrupt the dispersion.
        R0 = self._R_odom_optical(stamp, 0.0)
        if R0 is None:
            return
        n_odom0 = R0.apply(n_opt)
        if abs(n_odom0[2]) > self.wall_tilt_max:
            self.n_floor_reject += 1
            return
        self.frames.append((stamp, n_opt, pts))

    def _to_points(self, msg: Image):
        fx, fy, cx, cy = self.K
        h, w = msg.height, msg.width
        buf = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if buf.size < h * w:
            return None
        depth = buf[:h * w].reshape(h, w)
        ys = np.arange(0, h, self.skip)
        xs = np.arange(0, w, self.skip)
        gx, gy = np.meshgrid(xs, ys)
        z = depth[gy, gx].astype(np.float64)
        m = np.isfinite(z) & (z > self.d_min) & (z < self.d_max)
        z = z[m]
        gx = gx[m].astype(np.float64)
        gy = gy[m].astype(np.float64)
        x = (gx - cx) * z / fx
        y = (gy - cy) * z / fy
        return np.stack([x, y, z], axis=1)

    def _R_odom_optical(self, stamp: Time, delta_s: float):
        """Rotation odom<-optical at (stamp - delta). None if TF missing."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom, self.optical, stamp - Duration(seconds=delta_s))
        except Exception:
            return None
        q = tf.transform.rotation
        return Rotation.from_quat([q.x, q.y, q.z, q.w])

    def _dominant_wall_frames(self, frames):
        """Keep only frames whose odom-frame normal (at delta=0) belongs to the
        SINGLE dominant wall. Frames from a different surface (a side wall, a
        residual bit of floor, furniture) have a normal pointing elsewhere; mixing
        them makes the dispersion floor high and delta-insensitive (the plateau we
        saw on the robot). Dominant direction = principal axis of the scatter
        matrix (sign-free); reject frames whose normal is > wall_consistency off it."""
        recs = []
        for (stamp, n_opt, pts) in frames:
            R0 = self._R_odom_optical(stamp, 0.0)
            if R0 is None:
                continue
            recs.append((stamp, n_opt, pts, R0.apply(n_opt)))
        if len(recs) < 6:
            return recs, 0.0
        N = np.array([r[3] for r in recs])
        S = (N.T @ N) / N.shape[0]
        w, V = np.linalg.eigh(S)
        axis = V[:, -1]                       # dominant normal direction (sign-free)
        # angle of each normal to the axis, folded to [0, 90] (|dot|)
        ang = np.arccos(np.clip(np.abs(N @ axis), 0.0, 1.0))
        keep = [recs[i][:3] for i in range(len(recs)) if ang[i] <= self.wall_consistency]
        rejected = len(recs) - len(keep)
        return keep, rejected

    def on_report(self):
        frames = list(self.frames)
        if len(frames) < 6:
            self.get_logger().warn(
                f"only {len(frames)} usable (rotating) frames buffered "
                f"(floor/ceiling rejected so far: {self.n_floor_reject}); "
                f"rotate more / longer in front of a SINGLE vertical wall.")
            return

        # Keep only the dominant vertical wall (drop mixed/off-wall surfaces).
        frames, n_offwall = self._dominant_wall_frames(frames)
        if len(frames) < 6:
            self.get_logger().warn(
                f"after wall-consistency gate only {len(frames)} frames on the "
                f"dominant wall ({n_offwall} off-wall dropped); aim the camera at "
                f"ONE flat wall while rotating.")
            return

        # Primary: normal angular variance vs delta.
        cost = np.full(self.deltas_ms.shape, np.nan)
        for di, dms in enumerate(self.deltas_ms):
            ds = dms / 1000.0
            normals = []
            for (stamp, n_opt, _pts) in frames:
                R = self._R_odom_optical(stamp, ds)
                if R is None:
                    continue
                normals.append(R.apply(n_opt))
            if len(normals) < 6:
                continue
            cost[di] = _normal_dispersion(np.asarray(normals))

        valid = ~np.isnan(cost)
        if valid.sum() < 4:
            self.get_logger().warn("not enough TF coverage across delta sweep; "
                                   "is odom TF live for the whole window?")
            return
        d_ms, curve = self.deltas_ms[valid], cost[valid]
        i_min = int(np.argmin(curve))
        delta_star = self._parabolic_min(d_ms, curve, i_min)

        # Corroborator: plane thickness at delta_star.
        thick = self._plane_thickness(frames, delta_star / 1000.0)

        # Naive reference: stamp age vs wall-clock now.
        ages = []
        for (stamp, _n, _p) in frames:
            ages.append((self.get_clock().now() - stamp).nanoseconds / 1e6)
        age_mean = float(np.mean(ages)) if ages else float("nan")

        # Well-depth quality: a trustworthy estimate has a clear interior minimum.
        # If the curve is monotone (min at either boundary) or barely dips, there's
        # no real signal -- warn instead of reporting a boundary-pinned delta*.
        cmin, cmax = float(curve.min()), float(curve.max())
        well = 1.0 - (cmin / cmax) if cmax > 0 else 0.0   # 0=flat, ->1=deep well
        boundary = (i_min == 0 or i_min == len(curve) - 1)

        # compact curve print (every ~10ms)
        show = [f"{d:.0f}:{c*1e4:.2f}" for d, c in zip(d_ms, curve)
                if abs(d % 10.0) < self.dstep_ms / 2]
        tag = "===== delta*"
        if boundary or well < 0.2:
            tag = ("XXXXX UNRELIABLE (curve monotone/flat -> no clear minimum; "
                   "rotate faster & keep ONE wall filling the view). delta*")
        self.get_logger().info(
            f"{tag} = {delta_star:.1f} ms  (frames={len(frames)}, "
            f"off-wall_dropped={n_offwall}, floor_dropped={self.n_floor_reject}, "
            f"well_depth={well:.2f}, "
            f"cost_min={curve[i_min]*1e4:.2f}e-4, "
            f"plane_thickness@delta*={thick*1000:.1f} mm, "
            f"naive_stamp_age~{age_mean:.1f} ms) =====\n"
            f"       dispersion curve (delta_ms:eig2*1e4): {'  '.join(show)}")

    @staticmethod
    def _parabolic_min(x, y, i):
        """Sub-step minimum via parabola through (i-1,i,i+1); clamps to grid."""
        if i <= 0 or i >= len(x) - 1:
            return float(x[i])
        y0, y1, y2 = y[i - 1], y[i], y[i + 1]
        denom = (y0 - 2 * y1 + y2)
        if abs(denom) < 1e-15:
            return float(x[i])
        off = 0.5 * (y0 - y2) / denom            # in index units
        off = max(-1.0, min(1.0, off))
        step = x[i + 1] - x[i]
        return float(x[i] + off * step)

    def _plane_thickness(self, frames, delta_s):
        """RMS thickness of one plane fit to all frames accumulated at pose(t-delta)."""
        acc = []
        for (stamp, _n, pts) in frames:
            R = self._R_odom_optical(stamp, delta_s)
            if R is None:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.odom, self.optical, stamp - Duration(seconds=delta_s))
            except Exception:
                continue
            t = tf.transform.translation
            acc.append(R.apply(pts) + np.array([t.x, t.y, t.z]))
        if not acc:
            return float("nan")
        allp = np.concatenate(acc, axis=0)
        if allp.shape[0] > 20000:                # cap cost
            sel = np.random.default_rng(1).choice(allp.shape[0], 20000, replace=False)
            allp = allp[sel]
        _n, rms = _fit_plane_normal(allp, thresh=self.thresh, min_inliers=200)
        return rms if rms is not None else float("nan")


def main():
    rclpy.init()
    node = DepthLatencyProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
