"""
Launch file for combined RealSense RGBD + safety shield visualization in RViz.
Shows camera feeds, pose overlays, motion prediction, and human/robot reach capsules.
Fixed frame is 'world' (required for safety shield markers).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    safety_shield_pkg_dir = get_package_share_directory('safety_shield_node')

    config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(safety_shield_pkg_dir, 'rviz', 'realsense_safety_shield.rviz'),
        description='Path to RViz config file'
    )

    pose_overlay_node = Node(
        package='realsense_rgbd_streamer',
        executable='pose_2d_overlay',
        name='pose_2d_overlay',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        pose_overlay_node,
        rviz_node,
    ])
