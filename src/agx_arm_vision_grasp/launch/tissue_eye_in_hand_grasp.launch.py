import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("agx_arm_vision_grasp")
    arm_ctrl_share = get_package_share_directory("agx_arm_ctrl")
    default_config = os.path.join(pkg_share, "config", "tissue_grasp.yaml")
    default_ik_config = os.path.join(arm_ctrl_share, "config", "arm_ik_pose.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML config for tissue detector and pick nodes.",
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Optional namespace shared by detector and pick nodes.",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="true",
        choices=["true", "false"],
        description="When true, publish perception/debug output but do not command the arm.",
    )
    auto_pick_arg = DeclareLaunchArgument(
        "auto_pick",
        default_value="false",
        choices=["true", "false"],
        description="Automatically run one pick sequence when a fresh target is detected.",
    )
    enable_ik_pose_arg = DeclareLaunchArgument(
        "enable_ik_pose",
        default_value="true",
        choices=["true", "false"],
        description="Start arm_ik_pose_node to convert PoseStamped commands into /control/move_j.",
    )
    ik_config_arg = DeclareLaunchArgument(
        "ik_config",
        default_value=default_ik_config,
        description="YAML config for arm_ik_pose_node.",
    )
    ik_python_executable_arg = DeclareLaunchArgument(
        "ik_python_executable",
        default_value="/home/shveiliang/miniconda3/envs/pika/bin/python",
        description="Python executable with pinocchio and casadi installed.",
    )
    enable_static_camera_tf_arg = DeclareLaunchArgument(
        "enable_static_camera_tf",
        default_value="true",
        choices=["true", "false"],
        description="Publish a fixed TCP -> RealSense root frame transform from launch arguments.",
    )
    camera_parent_frame_arg = DeclareLaunchArgument(
        "camera_parent_frame",
        default_value="tcp_link",
        description="Parent frame for the eye-in-hand camera transform.",
    )
    camera_child_frame_arg = DeclareLaunchArgument(
        "camera_child_frame",
        default_value="camera_link",
        description="RealSense root frame attached to the robot wrist.",
    )
    camera_x_arg = DeclareLaunchArgument(
        "camera_x", default_value="-0.07468078225321914"
    )
    camera_y_arg = DeclareLaunchArgument(
        "camera_y", default_value="-0.011606012607028928"
    )
    camera_z_arg = DeclareLaunchArgument(
        "camera_z", default_value="-0.09446864264218623"
    )
    camera_roll_arg = DeclareLaunchArgument(
        "camera_roll", default_value="-0.0321356983588658"
    )
    camera_pitch_arg = DeclareLaunchArgument(
        "camera_pitch", default_value="-1.2015408589509187"
    )
    camera_yaw_arg = DeclareLaunchArgument(
        "camera_yaw", default_value="0.009178550859444086"
    )

    detector_params = [
        LaunchConfiguration("config"),
    ]
    picker_params = [
        LaunchConfiguration("config"),
        {
            "dry_run": LaunchConfiguration("dry_run"),
            "auto_pick": LaunchConfiguration("auto_pick"),
        },
    ]

    detector = Node(
        package="agx_arm_vision_grasp",
        executable="tissue_detector",
        namespace=LaunchConfiguration("namespace"),
        name="tissue_detector",
        output="screen",
        parameters=detector_params,
    )

    picker = Node(
        package="agx_arm_vision_grasp",
        executable="tissue_pick",
        namespace=LaunchConfiguration("namespace"),
        name="tissue_pick",
        output="screen",
        parameters=picker_params,
    )

    ik_pose = ExecuteProcess(
        cmd=[
            LaunchConfiguration("ik_python_executable"),
            "-m",
            "agx_arm_ctrl.arm_ik_pose_node",
            "--ros-args",
            "-r",
            "__node:=arm_ik_pose_node",
            "--params-file",
            LaunchConfiguration("ik_config"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_ik_pose")),
    )

    static_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tissue_camera_static_tf",
        condition=IfCondition(LaunchConfiguration("enable_static_camera_tf")),
        arguments=[
            "--x",
            LaunchConfiguration("camera_x"),
            "--y",
            LaunchConfiguration("camera_y"),
            "--z",
            LaunchConfiguration("camera_z"),
            "--roll",
            LaunchConfiguration("camera_roll"),
            "--pitch",
            LaunchConfiguration("camera_pitch"),
            "--yaw",
            LaunchConfiguration("camera_yaw"),
            "--frame-id",
            LaunchConfiguration("camera_parent_frame"),
            "--child-frame-id",
            LaunchConfiguration("camera_child_frame"),
        ],
    )

    return LaunchDescription(
        [
            config_arg,
            namespace_arg,
            dry_run_arg,
            auto_pick_arg,
            enable_ik_pose_arg,
            ik_config_arg,
            ik_python_executable_arg,
            enable_static_camera_tf_arg,
            camera_parent_frame_arg,
            camera_child_frame_arg,
            camera_x_arg,
            camera_y_arg,
            camera_z_arg,
            camera_roll_arg,
            camera_pitch_arg,
            camera_yaw_arg,
            detector,
            picker,
            ik_pose,
            static_camera_tf,
        ]
    )
