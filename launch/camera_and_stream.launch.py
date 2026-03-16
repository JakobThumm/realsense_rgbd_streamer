from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('realsense_rgbd_streamer')

    # Camera arguments
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

    # Streamer arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='25.0',
        description='Publishing rate in Hz'
    )

    compress_rgb_arg = DeclareLaunchArgument(
        'compress_rgb',
        default_value='true',
        description='Compress RGB images (true/false)'
    )

    compress_depth_arg = DeclareLaunchArgument(
        'compress_depth',
        default_value='true',
        description='Compress depth images (true/false)'
    )

    rgb_quality_arg = DeclareLaunchArgument(
        'rgb_quality',
        default_value='90',
        description='JPEG quality for RGB compression (0-100)'
    )

    depth_compression_format_arg = DeclareLaunchArgument(
        'depth_compression_format',
        default_value='zstd',
        description='Depth compression format: zstd (default) or png'
    )

    depth_zstd_level_arg = DeclareLaunchArgument(
        'depth_zstd_level',
        default_value='1',
        description='Zstd compression level for depth (1-22)'
    )

    depth_png_compression_arg = DeclareLaunchArgument(
        'depth_png_compression',
        default_value='1',
        description='PNG compression level for depth (0-9, used when depth_compression_format=png)'
    )

    camera_namespace_arg = DeclareLaunchArgument(
        'streamer_camera_namespace',
        default_value='/camera/camera',
        description='RealSense camera namespace'
    )

    camera_pose_arg = DeclareLaunchArgument(
        'camera_pose',
        default_value='default',
        description='Camera pose preset from config/camera_poses.yaml'
    )

    stream_reliable_arg = DeclareLaunchArgument(
        'stream_reliable',
        default_value='true',
        description='QoS reliability for /rgbd_stream topics: true=Reliable, false=Best Effort'
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'realsense_camera.launch.py')
        ),
        launch_arguments={
            'depth_profile': LaunchConfiguration('depth_profile'),
            'color_profile': LaunchConfiguration('color_profile'),
            'camera_pose': LaunchConfiguration('camera_pose'),
        }.items()
    )

    streamer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rgbd_stream.launch.py')
        ),
        launch_arguments={
            'publish_rate': LaunchConfiguration('publish_rate'),
            'compress_rgb': LaunchConfiguration('compress_rgb'),
            'compress_depth': LaunchConfiguration('compress_depth'),
            'rgb_quality': LaunchConfiguration('rgb_quality'),
            'depth_compression_format': LaunchConfiguration('depth_compression_format'),
            'depth_zstd_level': LaunchConfiguration('depth_zstd_level'),
            'depth_png_compression': LaunchConfiguration('depth_png_compression'),
            'camera_namespace': LaunchConfiguration('streamer_camera_namespace'),
            'stream_reliable': LaunchConfiguration('stream_reliable'),
        }.items()
    )

    return LaunchDescription([
        depth_profile_arg,
        color_profile_arg,
        publish_rate_arg,
        compress_rgb_arg,
        compress_depth_arg,
        rgb_quality_arg,
        depth_compression_format_arg,
        depth_zstd_level_arg,
        depth_png_compression_arg,
        camera_namespace_arg,  # named 'streamer_camera_namespace' to avoid leaking into rs_launch.py
        camera_pose_arg,
        stream_reliable_arg,
        camera_launch,
        TimerAction(period=5.0, actions=[streamer_launch]),
    ])
