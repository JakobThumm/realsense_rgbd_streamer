"""
Launch file for RealSense RGBD visualization in RViz.
Opens RViz with configured display for pointcloud, RGB image, and pose data.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('realsense_rgbd_streamer')

    # RViz config file
    rviz_config = os.path.join(pkg_dir, 'config', 'visualization.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rviz_node,
    ])
