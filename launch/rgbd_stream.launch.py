from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
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
        'camera_namespace',
        default_value='/camera/camera',
        description='RealSense camera namespace'
    )

    stream_reliable_arg = DeclareLaunchArgument(
        'stream_reliable',
        default_value='true',
        description='QoS reliability for /rgbd_stream topics: true=Reliable, false=Best Effort'
    )

    set_publish_rate = SetParameter(
        name='publish_rate',
        value=LaunchConfiguration('publish_rate')
    )

    rgbd_publisher = Node(
        package='realsense_rgbd_streamer',
        executable='rgbd_publisher',
        name='rgbd_publisher',
        output='screen',
        parameters=[{
            'compress_rgb': LaunchConfiguration('compress_rgb'),
            'compress_depth': LaunchConfiguration('compress_depth'),
            'rgb_quality': LaunchConfiguration('rgb_quality'),
            'depth_compression_format': LaunchConfiguration('depth_compression_format'),
            'depth_zstd_level': LaunchConfiguration('depth_zstd_level'),
            'depth_png_compression': LaunchConfiguration('depth_png_compression'),
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'stream_reliable': LaunchConfiguration('stream_reliable'),
        }]
    )

    return LaunchDescription([
        publish_rate_arg,
        set_publish_rate,
        compress_rgb_arg,
        compress_depth_arg,
        rgb_quality_arg,
        depth_compression_format_arg,
        depth_zstd_level_arg,
        depth_png_compression_arg,
        camera_namespace_arg,
        stream_reliable_arg,
        rgbd_publisher,
    ])
