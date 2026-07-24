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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

# bytes-per-pixel for the encodings the sim may emit
BPP = {"32FC1": 4, "16UC1": 2, "mono16": 2, "mono8": 1}


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
        # depth image fixup
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("frame_id", "front_optical")  # optical frame for depth data; "" = keep original
        self.declare_parameter("in_topic", "/image_raw/compressed/depth")
        self.declare_parameter("out_topic", "/front_depth/image")
        # camera_info synthesis
        self.declare_parameter("publish_camera_info", True)
        self.declare_parameter("fov", 90.0)
        self.declare_parameter("fov_axis", "horizontal")  # horizontal | vertical
        self.declare_parameter("rgb_width", 1920)
        self.declare_parameter("rgb_height", 1080)
        self.declare_parameter("rgb_frame_id", "front")
        self.declare_parameter("rgb_info_topic", "/image_raw/compressed/camera_info")
        self.declare_parameter("depth_info_topic", "/front_depth/camera_info")
        self.declare_parameter("rgb_info_rate_hz", 10.0)

        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.frame = self.get_parameter("frame_id").value
        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value

        self.pub = self.create_publisher(Image, out_topic, qos_profile_sensor_data)
        self.create_subscription(Image, in_topic, self.on_img, qos_profile_sensor_data)
        self.warned = False

        self.pub_ci = bool(self.get_parameter("publish_camera_info").value)
        if self.pub_ci:
            fov = float(self.get_parameter("fov").value)
            axis = self.get_parameter("fov_axis").value
            depth_frame = self.frame or "front"
            rgb_frame = self.get_parameter("rgb_frame_id").value

            # Depth CameraInfo (stamped per depth frame in on_img).
            self.depth_ci = make_camera_info(self.w, self.h, fov, depth_frame, axis)
            self.depth_ci_pub = self.create_publisher(
                CameraInfo, self.get_parameter("depth_info_topic").value,
                qos_profile_sensor_data)

            # RGB CameraInfo (the missing sim topic) published on a timer.
            rgb_w = int(self.get_parameter("rgb_width").value)
            rgb_h = int(self.get_parameter("rgb_height").value)
            self.rgb_ci = make_camera_info(rgb_w, rgb_h, fov, rgb_frame, axis)
            self.rgb_ci_pub = self.create_publisher(
                CameraInfo, self.get_parameter("rgb_info_topic").value,
                qos_profile_sensor_data)
            rate = float(self.get_parameter("rgb_info_rate_hz").value)
            self.create_timer(1.0 / rate, self.on_rgb_ci)
            self.get_logger().info(
                f"camera_info: rgb {rgb_w}x{rgb_h} fx={self.rgb_ci.k[0]:.1f} "
                f"-> {self.get_parameter('rgb_info_topic').value}; "
                f"depth {self.w}x{self.h} fx={self.depth_ci.k[0]:.1f} "
                f"-> {self.get_parameter('depth_info_topic').value} (fov={fov}° {axis})")

        self.get_logger().info(
            f"depth fixup: {in_topic} -> {out_topic} "
            f"({self.w}x{self.h}, frame='{self.frame or 'keep'}')")

    def on_rgb_ci(self):
        self.rgb_ci.header.stamp = self.get_clock().now().to_msg()
        self.rgb_ci_pub.publish(self.rgb_ci)

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
        self.pub.publish(msg)

        # Depth CameraInfo stamped to match this frame (for depth_image_proc).
        if self.pub_ci:
            self.depth_ci.header.stamp = msg.header.stamp
            self.depth_ci_pub.publish(self.depth_ci)


def main():
    rclpy.init()
    node = DepthImageFixup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
