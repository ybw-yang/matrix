#!/usr/bin/env python3
"""Fix the sim's depth image header AND synthesize the missing CameraInfo(s).

Two sim quirks handled here:

1. Depth image header: the MATRiX depth publisher fills `data` (640*480*4 bytes,
   32FC1) and `encoding` but leaves height/width/step at 0 (and frame_id="map"),
   which breaks RViz, cv_bridge and image_proc. We rewrite the header and
   republish a well-formed Image on `out_topic` (/front_depth/image).

2. CameraInfo: /image_raw/compressed/camera_info is registered but never
   publishes, so image_proc / depth->pointcloud have no intrinsics. The sim
   also never publishes a depth camera_info. We synthesize both from
   config/config.json's fov + resolution (pinhole model, no distortion):
       fx = (w/2) / tan(fov/2),  fy = fx (square pixels),  cx=w/2, cy=h/2
   - RGB CameraInfo  -> `rgb_info_topic`   (default the missing
     /image_raw/compressed/camera_info, 1920x1080) on a timer.
   - Depth CameraInfo-> `depth_info_topic` (default /front_depth/camera_info,
     640x480) stamped to match each fixed depth frame, so depth_image_proc can
     turn /front_depth/image into a cloud directly.

NOTE: fov is assumed HORIZONTAL (UE convention) with square pixels. If your
cloud looks stretched, flip `fov_axis` to vertical.

Usage (standalone):
  python3 scripts/depth_image_fixup.py --ros-args \
      -p width:=640 -p height:=480 -p frame_id:=front \
      -p fov:=90.0 -p rgb_width:=1920 -p rgb_height:=1080
"""
import math
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo

# bytes-per-pixel for the encodings the sim may emit
BPP = {"32FC1": 4, "16UC1": 2, "mono16": 2, "mono8": 1}

# Fallback sensor defaults, used only if config.json is missing/unreadable.
DEFAULTS = {
    "width": 640, "height": 480, "fov": 90.0,
    "rgb_width": 1920, "rgb_height": 1080, "rgb_fov": 90.0,
}


def _default_config_path():
    """config/config.json relative to this script (scripts/ -> ../config)."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, os.pardir, "config", "config.json"))


def read_sensor_config(path):
    """Pull depth + rgb resolution/fov from config.json, falling back to DEFAULTS
    for any missing file/key. depth_sensor drives the depth image + depth
    CameraInfo; camera drives the RGB CameraInfo. This keeps config.json the
    single source of truth so the depth header is rewritten with the sim's ACTUAL
    dims (a width/height mismatch corrupts the republished depth cloud)."""
    cfg = dict(DEFAULTS)
    try:
        with open(path) as f:
            sensors = json.load(f)["robot"]["sensors"]
    except (OSError, KeyError, ValueError):
        return cfg
    depth = sensors.get("depth_sensor", {})
    for src, dst in (("width", "width"), ("height", "height"), ("fov", "fov")):
        if src in depth:
            cfg[dst] = depth[src]
    cam = sensors.get("camera", {})
    for src, dst in (("width", "rgb_width"), ("height", "rgb_height"), ("fov", "rgb_fov")):
        if src in cam:
            cfg[dst] = cam[src]
    return cfg



def make_camera_info(width, height, fov_deg, frame_id, fov_axis="horizontal"):
    """Pinhole CameraInfo from a single fov (square pixels, no distortion)."""
    ci = CameraInfo()
    ci.width = int(width)
    ci.height = int(height)
    ci.header.frame_id = frame_id
    ci.distortion_model = "plumb_bob"
    ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]

    fov = math.radians(fov_deg)
    if fov_axis == "vertical":
        f = (height / 2.0) / math.tan(fov / 2.0)
    else:  # horizontal (default)
        f = (width / 2.0) / math.tan(fov / 2.0)
    fx = fy = f
    cx = width / 2.0
    cy = height / 2.0

    ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return ci


class DepthImageFixup(Node):
    def __init__(self):
        super().__init__("depth_image_fixup")
        # Sensor resolution / fov come from config.json (single source of truth)
        # so the depth header + synthesized CameraInfos match the sim. Each value
        # is still exposed as a parameter and can be overridden explicitly.
        self.declare_parameter("config_path", _default_config_path())
        cfg = read_sensor_config(self.get_parameter("config_path").value)

        # depth image fixup
        self.declare_parameter("width", int(cfg["width"]))
        self.declare_parameter("height", int(cfg["height"]))
        self.declare_parameter("frame_id", "front_optical")  # optical frame for depth data; "" = keep original
        self.declare_parameter("in_topic", "/image_raw/compressed/depth")
        self.declare_parameter("out_topic", "/front_depth/image")
        # Depth path QoS depth (queue length). The republish serializes ~3MB per
        # frame; a shallow queue (sensor_data default = 5) drops frames whenever
        # processing jitters. A deeper queue (~3s at 10Hz) absorbs the jitter.
        self.declare_parameter("qos_depth", 30)
        # Republish the depth image + depth CameraInfo with RELIABLE QoS. The sim
        # source is BEST_EFFORT (no retransmit); at 1080x720 (~3MB/frame, 30MB/s)
        # a best-effort subscriber drops ~35% even as the sole consumer. This node
        # subscribes best-effort (to match the source) and re-publishes reliable,
        # so any reliable downstream consumer gets every frame. A RELIABLE pub is
        # still compatible with best-effort subscribers (they just get best-effort
        # delivery). Set false to pass best-effort through unchanged.
        self.declare_parameter("reliable_output", True)
        # Latency compensation: subtract this many ms from the depth stamp so
        # downstream TF lookups land on the frame's true capture time, not its
        # (later) render->readback->publish time. 0 = off (unchanged behavior).
        self.declare_parameter("stamp_offset_ms", 0.0)
        # camera_info synthesis
        self.declare_parameter("publish_camera_info", True)
        self.declare_parameter("fov", float(cfg["fov"]))            # depth fov
        self.declare_parameter("fov_axis", "horizontal")  # horizontal | vertical
        self.declare_parameter("rgb_width", int(cfg["rgb_width"]))
        self.declare_parameter("rgb_height", int(cfg["rgb_height"]))
        self.declare_parameter("rgb_fov", float(cfg["rgb_fov"]))    # rgb camera fov
        self.declare_parameter("rgb_frame_id", "front")
        self.declare_parameter("rgb_info_topic", "/image_raw/compressed/camera_info")
        self.declare_parameter("depth_info_topic", "/front_depth/camera_info")
        self.declare_parameter("rgb_info_rate_hz", 10.0)

        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.frame = self.get_parameter("frame_id").value
        self.get_logger().info(f"w: {self.w}")
        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        self.stamp_offset_ns = int(float(self.get_parameter("stamp_offset_ms").value) * 1e6)

        # Best-effort, deep queue for SUBSCRIBING to the sim source (a reliable
        # sub would be QoS-incompatible with the best-effort source and receive
        # nothing). Deep queue absorbs delivery jitter.
        qos_depth = int(self.get_parameter("qos_depth").value)
        depth_qos = QoSProfile(depth=qos_depth,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST)
        # Output QoS: RELIABLE by default so downstream consumers get every frame
        # (best-effort would drop ~35% at this bitrate). Reverts to best-effort if
        # reliable_output is false.
        out_reliable = bool(self.get_parameter("reliable_output").value)
        out_qos = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE if out_reliable
            else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        # Callback groups: keep on_img on its own mutually-exclusive group so
        # frames stay strictly ordered, and the RGB CameraInfo timer on another,
        # so (under the MultiThreadedExecutor) the timer never blocks the depth
        # republish and vice-versa.
        self.img_cbg = MutuallyExclusiveCallbackGroup()
        self.timer_cbg = MutuallyExclusiveCallbackGroup()

        self.pub = self.create_publisher(Image, out_topic, out_qos)
        self.create_subscription(Image, in_topic, self.on_img, depth_qos,
                                 callback_group=self.img_cbg)
        self.warned = False

        self.pub_ci = bool(self.get_parameter("publish_camera_info").value)
        if self.pub_ci:
            fov = float(self.get_parameter("fov").value)
            axis = self.get_parameter("fov_axis").value
            depth_frame = self.frame or "front"
            rgb_frame = self.get_parameter("rgb_frame_id").value

            # Depth CameraInfo (stamped per depth frame in on_img). Same reliable
            # output QoS as the image so the pair stays deliverable together.
            self.depth_ci = make_camera_info(self.w, self.h, fov, depth_frame, axis)
            self.depth_ci_pub = self.create_publisher(
                CameraInfo, self.get_parameter("depth_info_topic").value,
                out_qos)

            # RGB CameraInfo (the missing sim topic) published on a timer.
            # Uses the camera's OWN fov (rgb_fov), which may differ from depth.
            rgb_w = int(self.get_parameter("rgb_width").value)
            rgb_h = int(self.get_parameter("rgb_height").value)
            rgb_fov = float(self.get_parameter("rgb_fov").value)
            self.rgb_ci = make_camera_info(rgb_w, rgb_h, rgb_fov, rgb_frame, axis)
            self.rgb_ci_pub = self.create_publisher(
                CameraInfo, self.get_parameter("rgb_info_topic").value,
                qos_profile_sensor_data)
            rate = float(self.get_parameter("rgb_info_rate_hz").value)
            self.create_timer(1.0 / rate, self.on_rgb_ci, callback_group=self.timer_cbg)
            self.get_logger().info(
                f"camera_info: rgb {rgb_w}x{rgb_h} fx={self.rgb_ci.k[0]:.1f} "
                f"(fov={rgb_fov}°) -> {self.get_parameter('rgb_info_topic').value}; "
                f"depth {self.w}x{self.h} fx={self.depth_ci.k[0]:.1f} "
                f"(fov={fov}°) -> {self.get_parameter('depth_info_topic').value} ({axis})")

        self.get_logger().info(
            f"depth fixup: {in_topic} -> {out_topic} "
            f"({self.w}x{self.h}, frame='{self.frame or 'keep'}', "
            f"stamp_offset={self.stamp_offset_ns / 1e6:.1f}ms)")

    def on_rgb_ci(self):
        self.rgb_ci.header.stamp = self.get_clock().now().to_msg()
        self.rgb_ci_pub.publish(self.rgb_ci)

    @staticmethod
    def _shift_stamp_earlier(stamp, offset_ns):
        """Move a builtin_interfaces/Time earlier by offset_ns, in place."""
        total = stamp.sec * 1_000_000_000 + stamp.nanosec - offset_ns
        if total < 0:
            total = 0
        stamp.sec = int(total // 1_000_000_000)
        stamp.nanosec = int(total % 1_000_000_000)

    def on_img(self, msg: Image):
        bpp = BPP.get(msg.encoding, 0)
        expected = self.w * self.h * bpp
        if bpp == 0:
            self.get_logger().warn(f"unknown encoding '{msg.encoding}', passing through",
                                   throttle_duration_sec=5.0)
        elif len(msg.data) != expected and not self.warned:
            self.warned = True
            self.get_logger().warn(
                f"data len {len(msg.data)} != {self.w}x{self.h}x{bpp}={expected}; "
                f"check width/height params against the sim resolution.")

        # Only overwrite the header dims if they are missing/zero.
        if msg.height == 0 or msg.width == 0 or msg.step == 0:
            msg.height = self.h
            msg.width = self.w
            msg.step = self.w * bpp if bpp else msg.step
        if self.frame:
            msg.header.frame_id = self.frame
        # Compensate the fixed render->publish latency (measured ~50ms via
        # `ros2 topic delay /front_depth/image`, vs ~0 for /odom). Shifting the
        # stamp earlier makes the planner's odom->base TF lookup resolve to the
        # pose at actual capture time, removing the motion/rotation ghost.
        if self.stamp_offset_ns:
            self._shift_stamp_earlier(msg.header.stamp, self.stamp_offset_ns)
        self.pub.publish(msg)

        # Depth CameraInfo stamped to match this frame (for depth_image_proc).
        if self.pub_ci:
            self.depth_ci.header.stamp = msg.header.stamp
            self.depth_ci_pub.publish(self.depth_ci)


def main():
    rclpy.init()
    node = DepthImageFixup()
    # Multi-threaded so the depth republish and the CameraInfo timer run on
    # separate threads (rmw serialization releases the GIL), preventing the timer
    # from stalling the depth path.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():  # avoid double-shutdown when a signal already tore it down
            rclpy.shutdown()


if __name__ == "__main__":
    main()
