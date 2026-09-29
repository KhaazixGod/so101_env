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
        "table_edge", "robot_x", "robot_y", "robot_edge_margin", "robot_yaw",
        "state_topic", "command_topic",
        "stiffness", "damping", "max_force", "drive_config", "joint_drive",
        "conveyor", "conveyor_engine", "conveyor_width", "conveyor_length", "conveyor_thickness",
        "conveyor_gap", "conveyor_offset", "conveyor_yaw", "conveyor_speed",
        "conveyor_demo_boxes", "conveyor_demo_box_size", "conveyor_demo_box_mass", "conveyor_friction",
        "conveyor_test_cube", "conveyor_test_cube_size", "conveyor_test_cube_mass",
        "conveyor_test_cube_drop_height", "conveyor_test_cube_x", "conveyor_test_cube_y",
        "physics_hz", "publish_tf", "publish_clock", "save_usd",
    )}

    scene_script = os.path.join(get_package_share_directory(PKG), "isaac", "so101_table_scene.py")

    cmd = [
        cfg["isaac_python"], scene_script,
        "--usd", cfg["robot_usd"],
        "--table-height", cfg["table_height"],
        "--table-size", cfg["table_length"], cfg["table_width"],
        "--table-edge", cfg["table_edge"],
        "--robot-edge-margin", cfg["robot_edge_margin"],
        "--robot-yaw", cfg["robot_yaw"],
        "--state-topic", cfg["state_topic"],
        "--command-topic", cfg["command_topic"],
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

    # --robot-xy: leave both robot_x/robot_y empty (their default) to let the
    # scene script place the robot itself, --robot-edge-margin in from
    # --table-edge. Passing only one of the pair would be ambiguous, so both
    # are required together.
    if cfg["robot_x"] or cfg["robot_y"]:
        cmd += ["--robot-xy", cfg["robot_x"] or "0.0", cfg["robot_y"] or "0.0"]

    if not _bool(cfg["conveyor"]):
        cmd.append("--no-conveyor")
    else:
        cmd += [
            "--conveyor-engine", cfg["conveyor_engine"],
            "--conveyor-width", cfg["conveyor_width"],
            "--conveyor-length", cfg["conveyor_length"],
            "--conveyor-thickness", cfg["conveyor_thickness"],
            "--conveyor-gap", cfg["conveyor_gap"],
            "--conveyor-offset", cfg["conveyor_offset"],
            "--conveyor-yaw", cfg["conveyor_yaw"],
            "--conveyor-speed", cfg["conveyor_speed"],
            # warp engine only -- harmless to always pass, the scene script ignores
            # these under --conveyor-engine physx.
            "--conveyor-demo-boxes", cfg["conveyor_demo_boxes"],
            "--conveyor-demo-box-size", cfg["conveyor_demo_box_size"],
            "--conveyor-demo-box-mass", cfg["conveyor_demo_box_mass"],
            "--conveyor-friction", cfg["conveyor_friction"],
        ]
        if _bool(cfg["conveyor_test_cube"]):
            cmd += [
                "--conveyor-test-cube",
                "--conveyor-test-cube-size", cfg["conveyor_test_cube_size"],
                "--conveyor-test-cube-mass", cfg["conveyor_test_cube_mass"],
                "--conveyor-test-cube-drop-height", cfg["conveyor_test_cube_drop_height"],
            ]
            # Leave both conveyor_test_cube_x/y empty (the default) to drop it
            # at the belt's own centre -- same both-or-neither rule as robot_xy.
            if cfg["conveyor_test_cube_x"] or cfg["conveyor_test_cube_y"]:
                cmd += [
                    "--conveyor-test-cube-xy",
                    cfg["conveyor_test_cube_x"] or "0.0",
                    cfg["conveyor_test_cube_y"] or "0.0",
                ]

    # Drive gains: leave --stiffness/--damping/--max-force/--drive-config unset
    # (empty launch args) to let the scene script's own defaults -- and
    # config/joint_drives.yaml's `default:`/`joints:` sections -- take over.
    # Passing them here would always win over that file's `default:` section.
    if cfg["drive_config"]:
        cmd += ["--drive-config", cfg["drive_config"]]
    if cfg["stiffness"]:
        cmd += ["--stiffness", cfg["stiffness"]]
    if cfg["damping"]:
        cmd += ["--damping", cfg["damping"]]
    if cfg["max_force"]:
        cmd += ["--max-force", cfg["max_force"]]
    # Multiple joints: joint_drive:="joint_1:stiffness=90,damping=6;joint_5:stiffness=25"
    for spec in cfg["joint_drive"].split(";"):
        spec = spec.strip()
        if spec:
            cmd += ["--joint-drive", spec]

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
        DeclareLaunchArgument(
            "table_edge", default_value="front",
            description="front/back/left/right: which table edge the robot and conveyor default to "
                        "(front/back = the long edges at y=-W/2 / y=+W/2)",
        ),
        DeclareLaunchArgument(
            "robot_x", default_value="",
            description="robot position on the table top, m; leave robot_x AND robot_y empty "
                        "to auto-place the robot at table_edge, robot_edge_margin in from it",
        ),
        DeclareLaunchArgument("robot_y", default_value=""),
        DeclareLaunchArgument(
            "robot_edge_margin", default_value="0.08",
            description="how far in from table_edge the auto-placed robot sits, m",
        ),
        DeclareLaunchArgument("robot_yaw", default_value="0.0", description="degrees"),
        DeclareLaunchArgument("state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("command_topic", default_value="/joint_command"),
        DeclareLaunchArgument(
            "drive_config",
            default_value=os.path.join(pkg_isaac, "config", "joint_drives.yaml"),
            description="YAML with default/per-joint stiffness, damping, max_force; "
                        "empty string skips it and falls back to the scene script's hardcoded 60/4/None",
        ),
        DeclareLaunchArgument(
            "stiffness", default_value="",
            description="angular drive stiffness for every joint (per degree); overrides "
                        "drive_config's default: section; empty leaves drive_config in charge",
        ),
        DeclareLaunchArgument(
            "damping", default_value="",
            description="angular drive damping for every joint (per degree/s); same override rule as stiffness",
        ),
        DeclareLaunchArgument(
            "max_force", default_value="",
            description="drive maxForce for every joint, Nm; same override rule as stiffness",
        ),
        DeclareLaunchArgument(
            "joint_drive", default_value="",
            description="per-joint override(s), ';'-separated, e.g. "
                        "'joint_1:stiffness=90,damping=6;joint_5:stiffness=25' -- "
                        "highest precedence, wins over drive_config and the flags above",
        ),
        DeclareLaunchArgument(
            "conveyor", default_value="true",
            description="add a conveyor belt against table_edge",
        ),
        DeclareLaunchArgument(
            "conveyor_engine", default_value="physx", choices=["physx", "warp"],
            description="'physx' (default, works) is a kinematic body with PhysxSurfaceVelocityAPI. "
                        "'warp' is Isaac Sim's own conveyor_belt sample technique -- builds and "
                        "registers correctly but does not currently move anything (verified: "
                        "isaacsim.core.api.World(backend='warp') does not step physics in this "
                        "Isaac Sim release, on both installs on this machine). Use 'physx'.",
        ),
        DeclareLaunchArgument("conveyor_width", default_value="0.30", description="belt width, m"),
        DeclareLaunchArgument("conveyor_length", default_value="1.00", description="belt length, m"),
        DeclareLaunchArgument("conveyor_thickness", default_value="0.04"),
        DeclareLaunchArgument(
            "conveyor_gap", default_value="0.0",
            description="gap between the table edge and the belt's near end, m (0 = touching)",
        ),
        DeclareLaunchArgument(
            "conveyor_offset", default_value="0.0",
            description="shift the belt sideways along the edge, relative to the table centre, m",
        ),
        DeclareLaunchArgument(
            "conveyor_yaw", default_value="90.0",
            description="extra rotation on top of table_edge's own orientation, degrees. "
                        "0 feeds straight toward/away from the table; 90 (default) runs the "
                        "belt alongside the edge instead",
        ),
        DeclareLaunchArgument(
            "conveyor_speed", default_value="0.2",
            description="belt surface speed, m/s, along its direction of travel "
                        "(conveyor_yaw away from table_edge's toward-the-table direction)",
        ),
        DeclareLaunchArgument(
            "conveyor_demo_boxes", default_value="2",
            description="warp engine only: boxes registered up front so there is something to drag",
        ),
        DeclareLaunchArgument("conveyor_demo_box_size", default_value="0.05", description="warp engine only, m"),
        DeclareLaunchArgument("conveyor_demo_box_mass", default_value="0.05", description="warp engine only, kg"),
        DeclareLaunchArgument(
            "conveyor_friction", default_value="0.5",
            description="warp engine only: Coulomb friction coefficient between a demo box and the belt",
        ),
        DeclareLaunchArgument(
            "conveyor_test_cube", default_value="false",
            description="drop an ordinary dynamic cube onto the belt from above, to visually confirm "
                        "it's dragging things -- moves under conveyor_engine physx, sits still under warp",
        ),
        DeclareLaunchArgument("conveyor_test_cube_size", default_value="0.05", description="edge length, m"),
        DeclareLaunchArgument("conveyor_test_cube_mass", default_value="0.05", description="kg"),
        DeclareLaunchArgument(
            "conveyor_test_cube_drop_height", default_value="0.08",
            description="height above the belt's top surface to drop the cube from, m",
        ),
        DeclareLaunchArgument(
            "conveyor_test_cube_x", default_value="",
            description="where to drop the test cube, relative to the table centre; leave "
                        "conveyor_test_cube_x AND _y empty to drop it at the belt's own centre",
        ),
        DeclareLaunchArgument("conveyor_test_cube_y", default_value=""),
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
