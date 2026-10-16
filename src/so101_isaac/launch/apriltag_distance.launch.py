"""One-command startup: Orbbec Astra camera + AprilTag detection + distance report.

    ros2 launch so101_isaac apriltag_distance.launch.py

Brings up astra_camera's own astra.launch.xml, apriltag_custom_detector
(detects tags directly on the raw colour stream via the `apriltag` Python
bindings + cv2.solvePnP -- NOT apriltag_ros/image_proc; see that node's
docstring for why), apriltag_distance_monitor (watches /apriltag/detections
and /tf, reports each visible tag's straight-line distance from the camera on
/apriltag/distances), and, unless disabled, an OpenCV window
(apriltag_overlay_view) showing the feed with each detected tag's outline, ID
and distance drawn directly on top:

    ros2 topic echo /apriltag/distances      # numeric distance per tag
    ros2 launch so101_isaac apriltag_distance.launch.py show_image:=false   # headless

Tag family/size/detector tuning lives in config/apriltag.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "so101_isaac"


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory(PKG)
    astra_share = get_package_share_directory("astra_camera")

    declared = [
        DeclareLaunchArgument(
            "color_image_topic", default_value="/camera/color/image_raw",
            description="astra_camera's raw colour image topic to rectify and detect tags in",
        ),
        DeclareLaunchArgument(
            "color_camera_info_topic", default_value="/camera/color/camera_info",
            description="astra_camera's colour camera_info topic (calibration)",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(pkg_share, "config", "apriltag.yaml"),
            description="tag family/size/detector params, see that file",
        ),
        DeclareLaunchArgument(
            "camera_frame", default_value="camera_color_optical_frame",
            description="frame the reported tag distances are measured from",
        ),
        DeclareLaunchArgument(
            "show_image", default_value="true",
            description="open the OpenCV window showing the feed with tag outlines/distances",
        ),
        DeclareLaunchArgument(
            "color_width", default_value="1280",
            description="colour sensor width -- 640x480 gives a small 2cm tag too few "
                        "pixels per module at any real working distance; this Astra unit "
                        "supports up to 1280x960 (confirmed 2026-10-14), 1920x1080 is not "
                        "supported and silently disables the colour stream",
        ),
        DeclareLaunchArgument("color_height", default_value="960"),
        DeclareLaunchArgument("color_fps", default_value="30"),
    ]

    camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(astra_share, "launch", "astra.launch.xml")),
        launch_arguments={
            "color_width": LaunchConfiguration("color_width"),
            "color_height": LaunchConfiguration("color_height"),
            "color_fps": LaunchConfiguration("color_fps"),
        }.items(),
    )

    detector = Node(
        package=PKG,
        executable="apriltag_custom_detector",
        name="apriltag_custom_detector",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
        remappings=[
            ("/camera/color/image_raw", LaunchConfiguration("color_image_topic")),
            ("/camera/color/camera_info", LaunchConfiguration("color_camera_info_topic")),
        ],
    )

    distance_monitor = Node(
        package=PKG,
        executable="apriltag_distance_monitor",
        name="apriltag_distance_monitor",
        output="screen",
        parameters=[{"camera_frame": LaunchConfiguration("camera_frame")}],
    )

    image_view = Node(
        package=PKG,
        executable="apriltag_overlay_view",
        name="apriltag_overlay_view",
        output="screen",
        parameters=[{
            "camera_frame": LaunchConfiguration("camera_frame"),
            # no rectify node anymore -- apriltag_custom_detector works
            # straight off the raw colour stream.
            "image_topic": LaunchConfiguration("color_image_topic"),
        }],
        condition=IfCondition(LaunchConfiguration("show_image")),
    )

    return LaunchDescription(declared + [camera, detector, distance_monitor, image_view])
