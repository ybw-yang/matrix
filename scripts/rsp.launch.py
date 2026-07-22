"""robot_state_publisher + tf glue for MATRiX URDF robots (xgb / xgw).

Nodes launched:
  * robot_state_publisher     -- URDF static/joint tf (root link BASE_LINK)
  * static_transform_publisher-- identity base_link -> BASE_LINK bridge, so
                                 robot_forward's `odom -> base_link` connects to
                                 the URDF root.
  * joint_state_publisher     -- publishes /joint_states so the revolute leg
                                 joints (12) are broadcast; WITHOUT this the legs
                                 (ABAD/HIP/KNEE/FOOT) are disconnected from
                                 BASE_LINK. NOTE: the sim does not expose real
                                 joint angles over ROS, so these are default (0)
                                 unless you feed a real /joint_states source.
  * base_footprint_publisher  -- see base_footprint_publisher.py.

Usage:
  ros2 launch scripts/rsp.launch.py \
      urdf:=src/robot_mujoco/zsibot_robots/xgb/xg_b.urdf \
      footprint_mode:=projection

Args:
  urdf              URDF path (xgb or xgw only; zgws ships none).
  base_frame        frame robot_forward broadcasts (default base_link).
  root_link         URDF root link (default BASE_LINK).
  publish_joints    run joint_state_publisher to connect the legs (default true).
  publish_footprint run base_footprint_publisher (default true).
  footprint_mode    projection | footplane (default projection).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf = LaunchConfiguration("urdf")
    base_frame = LaunchConfiguration("base_frame")
    root_link = LaunchConfiguration("root_link")
    footprint_mode = LaunchConfiguration("footprint_mode")
    real_joints = LaunchConfiguration("real_joints")
    robot_description = ParameterValue(Command(["cat ", urdf]), value_type=str)

    # Absolute paths to sibling scripts (this launch lives in scripts/).
    here = os.path.dirname(os.path.abspath(__file__))
    footprint_script = os.path.join(here, "base_footprint_publisher.py")
    joint_udp_script = os.path.join(here, "joint_state_udp_bridge.py")
    sensor_tf_script = os.path.join(here, "sensor_tf_publisher.py")
    ecal_bridge_bin = os.path.join(here, "bin", "mujoco_joint_bridge")
    config_json = os.path.join(here, os.pardir, "config", "config.json")

    return LaunchDescription([
        DeclareLaunchArgument("urdf",
            default_value="src/robot_mujoco/zsibot_robots/xgb/xg_b.urdf"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("root_link", default_value="BASE_LINK"),
        DeclareLaunchArgument("publish_footprint", default_value="true"),
        DeclareLaunchArgument("footprint_mode", default_value="footplane"),
        DeclareLaunchArgument("sensor_tf", default_value="true",
            description="publish base_link -> sensor frames from config/config.json"),
        DeclareLaunchArgument("real_joints", default_value="true",
            description="true: real leg angles from eCAL leg_data; "
                        "false: joint_state_publisher zeros."),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # Identity bridge: odom -> base_link (robot_forward) -> BASE_LINK (URDF).
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_link_bridge",
            output="screen",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", base_frame,
                "--child-frame-id", root_link,
            ],
        ),

        # --- REAL joints: eCAL leg_data -> UDP -> /joint_states ---------------
        ExecuteProcess(
            cmd=[ecal_bridge_bin, "leg_data", "25998"],
            output="screen",
            condition=IfCondition(real_joints),
        ),
        ExecuteProcess(
            cmd=["python3", joint_udp_script, "--ros-args", "-p", "port:=25998"],
            output="screen",
            condition=IfCondition(real_joints),
        ),

        # --- Fallback: zeros so the tree at least connects -------------------
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(real_joints),
        ),

        # base_footprint from ground projection (default) or four-foot plane.
        ExecuteProcess(
            cmd=[
                "python3", footprint_script, "--ros-args",
                "-p", ["mode:=", footprint_mode],
                "-p", ["base_frame:=", base_frame],
                "-p", "odom_frame:=odom",
                "-p", "footprint_frame:=base_footprint",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_footprint")),
        ),

        # Static tf base_link -> sensor frames (lidar / front / imu_link / ...).
        ExecuteProcess(
            cmd=[
                "python3", sensor_tf_script, "--ros-args",
                "-p", ["parent_frame:=", root_link],
                "-p", "config_path:=" + os.path.normpath(config_json),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("sensor_tf")),
        ),
    ])
