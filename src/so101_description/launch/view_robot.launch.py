import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('so101_description')
    
    # 1. Đường dẫn file Xacro và RViz Config
    xacro_file = os.path.join(pkg_path, 'urdf', 'so101.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Node Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # 3. Node Joint State Publisher GUI (Thanh trượt điều khiển khớp)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 4. Node RViz2 (Tự động nạp file cấu hình)
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])