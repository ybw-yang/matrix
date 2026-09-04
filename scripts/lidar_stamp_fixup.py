#!/usr/bin/env python3
"""Republish /livox/lidar with its header stamp corrected to the capture instant.

WHY
  The sim's lidar stamp is LATER than the world state the cloud actually depicts.
  Measured 2026-09-03 with stamp_sync_probe.py's content-level check (recover the
  sensor's own yaw rate from the azimuth/range profile's frame-to-frame shift and
  cross-correlate it against /odom's twist), over a 94 s bag of in-place rotation
  with reversals:

      run of 463 scans   +42.7 ms   corr 0.798
      run of 151 scans   +52.7 ms   corr 0.526
      all pairs          +43.5 ms   corr 0.717
      (live runs earlier: +39.7, +46.8, +21.1 ms)

  Positive means the stamp is late, i.e. a cloud stamped T shows the world at
  T - 45 ms. Every window with usable motion agreed on the sign. So shift the
  stamp EARLIER by that amount and a downstream TF lookup lands on the pose the
  cloud was actually taken from.

  This is the same correction, and the same reasoning, as the depth image's
  stamp_offset_ms:=134.0 in rsp.launch.py.

  Note this is NOT the ~16 ms that `ros2 topic delay` or age(recv-stamp) shows:
  that is stamp->receive transport (of which only ~1.4 ms is real DDS cost for a
  0.52 MB cloud, calibrated by loopback). Capture->stamp is the part that moves
  the cloud in space, and it is the part corrected here.

ACCURACY
  The lidar is 10 Hz, so the content-level estimate is good to roughly +/-10 ms,
  not better. 45 ms is the centre of the measured cluster, not an exact figure.
  Re-measure with `stamp_sync_probe.py --bag <dir>` (record via
  record_sync_bag.sh) and set stamp_offset_ms:=0 while doing so, otherwise the
  probe reports the RESIDUAL after this correction -- which, if 45 ms was right,
  should come out near 0. That residual check is the way to validate a change.

WHAT THIS DOES NOT DO
  It does not synthesize per-point timestamps. The sim leaves that field all
  zero, but its clouds are close to instantaneous snapshots rather than 100 ms
  sweeps (fine-scale azimuth structure loses only ~20% at 14 deg of rotation per
  scan, where a true sweep would lose ~95%), so there is no intra-scan skew to
  correct here. On real Livox hardware there would be, and this node would not
  be enough.

Usage:
  python3 lidar_stamp_fixup.py --ros-args \
      -p in_topic:=/livox/lidar -p out_topic:=/livox/lidar_fixed \
      -p stamp_offset_ms:=45.0
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class LidarStampFixup(Node):
    def __init__(self):
        super().__init__("lidar_stamp_fixup")
        self.declare_parameter("in_topic", "/livox/lidar")
        self.declare_parameter("out_topic", "/livox/lidar_fixed")
        # Capture->stamp latency to remove, in ms. 0 = pass through unchanged
        # (use that when measuring the RAW latency with stamp_sync_probe.py).
        self.declare_parameter("stamp_offset_ms", 45.0)
        # ~3 s of buffer at 10 Hz, so republish jitter never drops a scan.
        self.declare_parameter("qos_depth", 30)
        # The sim publishes BEST_EFFORT. Republishing RELIABLE costs nothing here
        # (0.52 MB at 10 Hz = 5.2 MB/s, and a loopback of that size measured
        # 1.4 ms) and a RELIABLE publisher is still compatible with best-effort
        # subscribers, so a sensor_data-QoS LIO connects either way.
        self.declare_parameter("reliable_output", True)
        self.declare_parameter("log_period_s", 10.0)

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        if in_topic == out_topic:
            raise SystemExit(f"in_topic and out_topic are both {in_topic}: "
                             "republishing onto the input would feed back into itself")
        self.offset_ns = int(float(self.get_parameter("stamp_offset_ms").value) * 1e6)
        depth = int(self.get_parameter("qos_depth").value)

        # Subscribe BEST_EFFORT to match the source: a RELIABLE subscriber is
        # QoS-incompatible with a best-effort publisher and receives NOTHING.
        sub_qos = QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
        out_qos = QoSProfile(
            depth=depth,
            reliability=(ReliabilityPolicy.RELIABLE
                         if bool(self.get_parameter("reliable_output").value)
                         else ReliabilityPolicy.BEST_EFFORT),
            history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(PointCloud2, out_topic, out_qos)
        self.sub = self.create_subscription(PointCloud2, in_topic, self.on_cloud, sub_qos)

        self.n = 0
        self.n_last = 0
        self.t_last = self.get_clock().now()
        period = float(self.get_parameter("log_period_s").value)
        if period > 0:
            self.create_timer(period, self.on_log)
        self.get_logger().info(
            f"{in_topic} -> {out_topic}, shifting stamps earlier by "
            f"{self.offset_ns / 1e6:.1f} ms "
            f"({'reliable' if out_qos.reliability == ReliabilityPolicy.RELIABLE else 'best_effort'} out)")

    @staticmethod
    def _shift_stamp_earlier(stamp, offset_ns):
        """Move a builtin_interfaces/Time earlier by offset_ns, in place."""
        total = stamp.sec * 1_000_000_000 + stamp.nanosec - offset_ns
        if total < 0:
            total = 0
        stamp.sec = int(total // 1_000_000_000)
        stamp.nanosec = int(total % 1_000_000_000)

    def on_cloud(self, msg):
        if self.offset_ns:
            self._shift_stamp_earlier(msg.header.stamp, self.offset_ns)
        self.pub.publish(msg)
        self.n += 1

    def on_log(self):
        now = self.get_clock().now()
        dt = (now - self.t_last).nanoseconds * 1e-9
        if dt > 0:
            self.get_logger().info(
                f"republished {self.n} clouds ({(self.n - self.n_last) / dt:.2f} Hz)")
        self.n_last, self.t_last = self.n, now


def main():
    rclpy.init()
    node = LidarStampFixup()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch stops an ExecuteProcess with SIGINT/SIGTERM, which surfaces as
        # ExternalShutdownException; without catching it every shutdown prints a
        # traceback into the launch log.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
