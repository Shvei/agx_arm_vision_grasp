import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("agx_arm_ctrl")
    default_config = os.path.join(pkg_share, "config", "arm_ik_pose.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="YAML config for arm_ik_pose_node.",
    )
    python_executable_arg = DeclareLaunchArgument(
        "python_executable",
        default_value="/home/shveiliang/miniconda3/envs/pika/bin/python",
        description="Python executable with pinocchio and casadi installed.",
    )

    node = ExecuteProcess(
        cmd=[
            LaunchConfiguration("python_executable"),
            "-m",
            "agx_arm_ctrl.arm_ik_pose_node",
            "--ros-args",
            "-r",
            "__node:=arm_ik_pose_node",
            "--params-file",
            LaunchConfiguration("config"),
        ],
        output="screen",
    )

    return LaunchDescription([config_arg, python_executable_arg, node])
