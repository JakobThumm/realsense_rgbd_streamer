from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    depth_profile_arg = DeclareLaunchArgument(
        'depth_profile',
        default_value='848x480x30',
        description='RealSense depth module profile (WxHxFPS)'
    )

    color_profile_arg = DeclareLaunchArgument(
        'color_profile',
        default_value='848x480x30',
        description='RealSense RGB camera profile (WxHxFPS)'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
        }.items()
    )

    return LaunchDescription([
        depth_profile_arg,
        color_profile_arg,
        realsense_launch,
    ])
