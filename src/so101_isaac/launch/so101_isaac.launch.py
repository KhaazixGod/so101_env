"""Bring up the Isaac Sim table scene plus the joint-editing ROS 2 node.

    ros2 launch so101_isaac so101_isaac.launch.py
    ros2 launch so101_isaac so101_isaac.launch.py headless:=true table_height:=0.90

Isaac Sim is started as a plain process with its own interpreter
(``isaac_python``) because it ships its own Python environment and is not a
ROS 2 executable.  It inherits this launch's environment, which is what lets
its ROS 2 bridge find the sourced ROS 2 Jazzy libraries.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "so101_isaac"


def _bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_isaac(context, *_args, **_kwargs):
    """Assemble the Isaac Sim command line once the launch arguments are known."""
    cfg = {key: LaunchConfiguration(key).perform(context) for key in (
        "isaac_python", "robot_usd", "headless", "table_height", "table_length", "table_width",
        "robot_x", "robot_y", "robot_yaw", "state_topic", "command_topic",
        "stiffness", "damping", "physics_hz", "publish_tf", "publish_clock", "save_usd",
    )}

    scene_script = os.path.join(get_package_share_directory(PKG), "isaac", "so101_table_scene.py")

    cmd = [
        cfg["isaac_python"], scene_script,
        "--usd", cfg["robot_usd"],
        "--table-height", cfg["table_height"],
        "--table-size", cfg["table_length"], cfg["table_width"],
        "--robot-xy", cfg["robot_x"], cfg["robot_y"],
        "--robot-yaw", cfg["robot_yaw"],
        "--state-topic", cfg["state_topic"],
        "--command-topic", cfg["command_topic"],
        "--stiffness", cfg["stiffness"],
        "--damping", cfg["damping"],
        "--physics-hz", cfg["physics_hz"],
    ]
    if _bool(cfg["headless"]):
        cmd.append("--headless")
    if not _bool(cfg["publish_tf"]):
        cmd.append("--no-publish-tf")
    if not _bool(cfg["publish_clock"]):
        cmd.append("--no-publish-clock")
    if cfg["save_usd"]:
        cmd += ["--save-usd", cfg["save_usd"]]

    return [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            emulate_tty=True,
            additional_env={"PYTHONUNBUFFERED": "1"},
        )
    ]


def generate_launch_description() -> LaunchDescription:
    pkg_isaac = get_package_share_directory(PKG)
    pkg_description = get_package_share_directory("so101_description")

    declared = [
        DeclareLaunchArgument(
            "isaac_python",
            default_value=os.path.expanduser("~/env_isaac/bin/python3"),
            description="interpreter of the Isaac Sim installation",
        ),
        DeclareLaunchArgument(
            "robot_usd",
            default_value=os.path.join(pkg_description, "urdf", "so101", "so101.usda"),
        ),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("table_height", default_value="0.74", description="table top surface height, m"),
        DeclareLaunchArgument("table_length", default_value="1.20"),
        DeclareLaunchArgument("table_width", default_value="0.80"),
        DeclareLaunchArgument("robot_x", default_value="0.0", description="robot position on the table top, m"),
        DeclareLaunchArgument("robot_y", default_value="0.0"),
        DeclareLaunchArgument("robot_yaw", default_value="0.0", description="degrees"),
        DeclareLaunchArgument("state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("command_topic", default_value="/joint_command"),
        DeclareLaunchArgument("stiffness", default_value="60.0", description="angular drive stiffness, per degree"),
        DeclareLaunchArgument("damping", default_value="4.0", description="angular drive damping, per degree/s"),
        DeclareLaunchArgument("physics_hz", default_value="120.0"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("publish_clock", default_value="true"),
        DeclareLaunchArgument("save_usd", default_value="", description="optional path to dump the built scene"),
        DeclareLaunchArgument(
            "start_commander",
            default_value="true",
            description="also start the node that edits the joint state",
        ),
        DeclareLaunchArgument(
            "commander_delay",
            default_value="20.0",
            description="seconds to let Isaac Sim come up before starting the commander",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_isaac, "config", "joint_commander.yaml"),
        ),
    ]

    commander = Node(
        package=PKG,
        executable="joint_commander",
        name="joint_commander",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "state_topic": LaunchConfiguration("state_topic"),
                "command_topic": LaunchConfiguration("command_topic"),
            },
        ],
    )

    return LaunchDescription(
        declared
        + [
            OpaqueFunction(function=_launch_isaac),
            TimerAction(
                period=LaunchConfiguration("commander_delay"),
                actions=[commander],
                condition=IfCondition(LaunchConfiguration("start_commander")),
            ),
        ]
    )
