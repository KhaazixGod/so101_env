"""AprilTag detection on the Orbbec Astra's colour stream.

    ros2 launch astra_camera astra.launch.xml   # in one terminal, first
    ros2 launch so101_isaac apriltag_astra.launch.py   # in another

Loads image_proc's RectifyNode and apriltag_ros's AprilTagNode into one
composable-node container (intra-process comms, matching apriltag_ros's own
stock camera_36h11.launch.yml) instead of two separate processes: rectify
takes /camera/color/image_raw + /camera/color/camera_info (astra_camera's
topics -- so astra.launch.xml must already be running) and republishes a
rectified image, which AprilTagNode consumes to detect tags and estimate
their pose. Detections come out on /apriltag/detections
(apriltag_msgs/AprilTagDetectionArray) and each tag's pose is also broadcast
on /tf as tag36h11:<id> relative to camera_color_optical_frame.

Tag family/size/detector tuning lives in config/apriltag.yaml, not here --
see that file before using this with tags other than 2 cm 36h11 ones.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

PKG = "so101_isaac"


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory(PKG)

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
    ]

    container = ComposableNodeContainer(
        name="apriltag_astra_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_color",
                namespace="camera",
                remappings=[
                    ("image", LaunchConfiguration("color_image_topic")),
                    ("camera_info", LaunchConfiguration("color_camera_info_topic")),
                    # RectifyNode's own output, relative to its "camera" namespace.
                    ("image_rect", "/camera/color/image_rect"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                namespace="apriltag",
                parameters=[LaunchConfiguration("config_file")],
                remappings=[
                    ("/apriltag/image_rect", "/camera/color/image_rect"),
                    ("/apriltag/camera_info", LaunchConfiguration("color_camera_info_topic")),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(declared + [container])
