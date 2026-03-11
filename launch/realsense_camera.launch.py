from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('realsense_rgbd_streamer')

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

    camera_pose_arg = DeclareLaunchArgument(
        'camera_pose',
        default_value='default',
        description='Camera pose preset from config/camera_poses.yaml'
    )

    # Load camera pose from config file at launch time
    poses_file = os.path.join(pkg_dir, 'config', 'camera_poses.yaml')
    with open(poses_file, 'r') as f:
        all_poses = yaml.safe_load(f)['presets']

    def make_static_tf_node(context):
        preset = context.launch_configurations['camera_pose']
        if preset not in all_poses:
            raise ValueError(
                f"Unknown camera_pose preset '{preset}'. "
                f"Available: {list(all_poses.keys())}"
            )
        p = all_poses[preset]
        return [Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_camera_link',
            arguments=[
                str(p['x']), str(p['y']), str(p['z']),
                str(p['yaw']), str(p['pitch']), str(p['roll']),
                'world', 'camera_link',
            ],
        )]

    from launch.actions import OpaqueFunction
    static_tf = OpaqueFunction(function=make_static_tf_node)

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                         'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'depth_module.depth_profile': LaunchConfiguration('depth_profile'),
            'rgb_camera.color_profile': LaunchConfiguration('color_profile'),
            'camera_namespace': 'camera',
            'camera_name': 'camera',
        }.items()
    )

    return LaunchDescription([
        depth_profile_arg,
        color_profile_arg,
        camera_pose_arg,
        realsense_launch,
        static_tf,
    ])
