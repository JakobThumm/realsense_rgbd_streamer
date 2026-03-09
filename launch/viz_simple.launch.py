"""
Simple launch file for RealSense RGBD visualization in RViz.
Uses minimal configuration with camera_1_link as fixed frame.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('realsense_rgbd_streamer')

    # Declare config file argument
    config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'config', 'visualization_minimal.rviz'),
        description='Path to RViz config file'
    )

    # Pose 2D overlay node
    pose_overlay_node = Node(
        package='realsense_rgbd_streamer',
        executable='pose_2d_overlay',
        name='pose_2d_overlay',
        output='screen'
    )

    # RViz node
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
