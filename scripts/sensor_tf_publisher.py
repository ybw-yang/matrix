#!/usr/bin/env python3
"""Publish static TF from base_link to each sensor frame, read from config/config.json.

The sim/UE config uses CENTIMETERS with y pointing RIGHT (x fwd, y right, z up);
this converts to ROS metres with y pointing LEFT. Rotations are in DEGREES
(roll about x, pitch about y, yaw about z); the y flip negates roll and yaw.

Sensor frame_ids (from the live sim messages):
  camera /image_raw/compressed        -> front
  lidar  /livox/lidar                 -> lidar
  body imu /imu                       -> imu_link        (mounted at body origin)
  camera imu /front_camera/imu        -> camera_imu_link (co-located with camera)
  lidar  imu /front_lidar/imu         -> livox_imu_link  (co-located with lidar)
The depth cloud publishes frame_id "map" (a sim quirk) and is intentionally skipped.
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# config sensor key -> (output frame_id, also-emit co-located imu frame or None)
SENSOR_FRAMES = {
    "camera": ("front", "camera_imu_link"),
    "lidar": ("lidar", "livox_imu_link"),
}


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class SensorTfPublisher(Node):
    def __init__(self):
        super().__init__("sensor_tf_publisher")
        self.declare_parameter("config_path", "config/config.json")
        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("length_scale", 0.01)  # cm -> m
        self.declare_parameter("y_sign", -1.0)         # right -> left
        cfg_path = self.get_parameter("config_path").value
        self.parent = self.get_parameter("parent_frame").value
        self.scale = float(self.get_parameter("length_scale").value)
        self.ysign = float(self.get_parameter("y_sign").value)

        with open(cfg_path) as f:
            robot = json.load(f)["robot"]
        sensors = robot.get("sensors", {})

        tfs = []
        for key, (frame, imu_frame) in SENSOR_FRAMES.items():
            if key not in sensors:
                continue
            tfs.append(self.make(sensors[key], frame))
            if imu_frame:  # sensor's own IMU is co-located with the sensor
                tfs.append(self.make(sensors[key], imu_frame))
        # Body IMU sits at the body origin.
        tfs.append(self.make({}, "imu_link"))

        self.br = StaticTransformBroadcaster(self)
        self.br.sendTransform(tfs)
        self.get_logger().info(
            f"published {len(tfs)} static sensor tf under '{self.parent}': "
            + ", ".join(t.child_frame_id for t in tfs))

    def make(self, sensor, frame):
        pos = sensor.get("position", {})
        rot = sensor.get("rotation", {})
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent
        t.child_frame_id = frame
        t.transform.translation.x = float(pos.get("x", 0.0)) * self.scale
        t.transform.translation.y = float(pos.get("y", 0.0)) * self.scale * self.ysign
        t.transform.translation.z = float(pos.get("z", 0.0)) * self.scale
        flip = -1.0 if self.ysign < 0 else 1.0
        r = math.radians(float(rot.get("roll", 0.0))) * flip
        p = math.radians(float(rot.get("pitch", 0.0)))
        y = math.radians(float(rot.get("yaw", 0.0))) * flip
        q = rpy_to_quat(r, p, y)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t


def main():
    rclpy.init()
    node = SensorTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
