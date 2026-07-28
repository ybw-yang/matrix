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
    depth_fixup_script = os.path.join(here, "depth_image_fixup.py")
    cmd_vel_script = os.path.join(here, "cmd_vel_udp_pub.py")
    odom_tf_script = os.path.join(here, "odom_to_tf.py")
    ecal_bridge_bin = os.path.join(here, "bin", "mujoco_joint_bridge")
    cmd_vel_bridge_bin = os.path.join(here, "bin", "cmd_vel_ecal_bridge")
    config_json = os.path.normpath(os.path.join(here, os.pardir, "config", "config.json"))

    return LaunchDescription([
        DeclareLaunchArgument("urdf",
            default_value="src/robot_mujoco/zsibot_robots/xgb/xg_b.urdf"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("root_link", default_value="BASE_LINK"),
        DeclareLaunchArgument("odom_tf", default_value="true",
            description="publish odom->base_link from /odom/mujoco_odom with full "
                        "roll/pitch (replaces robot_forward, which flattens tilt)"),
        DeclareLaunchArgument("publish_footprint", default_value="true"),
        DeclareLaunchArgument("footprint_mode", default_value="footplane"),
        DeclareLaunchArgument("contact_mode", default_value="height",
            description="foot contact estimate: height (flat ground) | velocity (slopes/steps)"),
        DeclareLaunchArgument("footprint_smooth_tau", default_value="0.25",
            description="base_footprint low-pass time constant (s); 0 disables, larger=smoother"),
        DeclareLaunchArgument("footprint_feet", default_value="contact",
            description="feet defining base_footprint centre: contact | all"),
        DeclareLaunchArgument("sensor_tf", default_value="true",
            description="publish base_link -> sensor frames from config/config.json"),
        DeclareLaunchArgument("real_joints", default_value="true",
            description="true: real leg angles from eCAL leg_data; "
                        "false: joint_state_publisher zeros."),
        DeclareLaunchArgument("depth_fixup", default_value="true",
            description="fix the sim depth image header (0 dims) -> /front_depth/image; "
                        "resolution/fov read from config.json inside the node"),
        # cmd_vel velocity control WRITES commands to the robot -> opt-in (default off).
        DeclareLaunchArgument("cmd_vel", default_value="true",
            description="enable /cmd_vel -> eCAL sdk_cmd velocity bridge (sends commands!)"),
        # To WALK via /cmd_vel: stand the robot first (keyboard U), then launch
        # with cmd_control_mode:=18 cmd_motion_mode:=1 (RL_MIX / policy_mix_walk
        # translation walk). Mode map (SDK path): stand=1/10, WALK=18/1,
        # balance-stand/RPY=21/100 (in-place body pose, NOT walk).
        # Default stays OBSERVE(-1) so velocity control is armed only on opt-in.
        DeclareLaunchArgument("cmd_control_mode", default_value="-1",
            description="SDKCmd control_mode; <0 = OBSERVE only (prints mode, no "
                        "command). 18 = RL_MIX walk (translate); 21 = balance-stand/RPY"),
        DeclareLaunchArgument("cmd_motion_mode", default_value="0",
            description="SDKCmd motion_mode; 1=Walk (pairs with cmd_control_mode:=18). "
                        "Not propagated for RL_MIX walk, so belt-and-suspenders only"),
        DeclareLaunchArgument("cmd_udp_port", default_value="25999"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # odom -> base_link with FULL orientation (roll/pitch/yaw), replacing
        # robot_forward. /odom/mujoco_odom already carries the true tilted pose.
        ExecuteProcess(
            cmd=[
                "python3", odom_tf_script, "--ros-args",
                "-p", "in_topic:=/odom/mujoco_odom",
                "-p", "odom_frame:=odom",
                "-p", ["base_frame:=", base_frame],
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("odom_tf")),
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
                "-p", ["contact_mode:=", LaunchConfiguration("contact_mode")],
                "-p", ["smooth_tau:=", LaunchConfiguration("footprint_smooth_tau")],
                "-p", ["footprint_feet:=", LaunchConfiguration("footprint_feet")],
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

        # Fix the sim's depth image header (height/width/step=0) -> /front_depth/image.
        # Resolution + fov come from config.json (read inside the node), so the
        # rewritten header/CameraInfo match the sim; just point it at the config.
        ExecuteProcess(
            cmd=[
                "python3", depth_fixup_script, "--ros-args",
                "-p", "config_path:=" + config_json,
                "-p", "frame_id:=front_optical",
                "-p", "in_topic:=/image_raw/compressed/depth",
                "-p", "out_topic:=/front_depth/image",
                # Depth stamp trails the frame's true capture time by ~134ms
                # (GPU render->readback->eCAL->republish); /odom is ~0. Shift the
                # stamp earlier so the planner's TF lookup uses the true capture
                # pose -> no ghost. NOTE: 134ms is the capture->stamp latency
                # measured by depth_latency_probe.py (rotate facing one wall), NOT
                # the ~50ms stamp->receive transport that `ros2 topic delay` shows
                # -- the former is what causes the ghost. Re-measure with the probe
                # (run it while fixup uses stamp_offset_ms:=0 to read RAW latency).
                "-p", "stamp_offset_ms:=134.0",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("depth_fixup")),
        ),

        # --- OPT-IN velocity control: /cmd_vel -> eCAL sdk_cmd (SENDS commands) ---
        # cmd_control_mode < 0 keeps the C++ bridge in OBSERVE mode (no command),
        # so enabling cmd_vel without a calibrated mode is still safe.
        ExecuteProcess(
            cmd=[
                cmd_vel_bridge_bin,
                LaunchConfiguration("cmd_udp_port"),
                LaunchConfiguration("cmd_control_mode"),
                LaunchConfiguration("cmd_motion_mode"),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("cmd_vel")),
        ),
        ExecuteProcess(
            cmd=[
                "python3", cmd_vel_script, "--ros-args",
                "-p", ["port:=", LaunchConfiguration("cmd_udp_port")],
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("cmd_vel")),
        ),
    ])
