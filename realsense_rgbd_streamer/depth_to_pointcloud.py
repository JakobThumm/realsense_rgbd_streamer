#!/usr/bin/env python3
"""
Node to convert RealSense depth and color images to pointcloud for RViz visualization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
from cv_bridge import CvBridge


class DepthToPointCloud(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud')

        self.bridge = CvBridge()

        # Camera info and images
        self.depth_image = None
        self.color_image = None
        self.camera_info = None

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/realsense/camera_1/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.color_sub = self.create_subscription(
            Image,
            '/realsense/camera_1/color/image_raw',
            self.color_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/realsense/camera_1/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )

        # Publisher for pointcloud
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/realsense/camera_1/pointcloud',
            10
        )

        # Timer to publish pointcloud at regular intervals
        self.timer = self.create_timer(0.1, self.publish_pointcloud)  # 10 Hz

        self.get_logger().info('Depth to PointCloud node initialized')

    def depth_callback(self, msg):
        """Store latest depth image."""
        self.depth_image = msg

    def color_callback(self, msg):
        """Store latest color image."""
        self.color_image = msg

    def camera_info_callback(self, msg):
        """Store camera intrinsics."""
        self.camera_info = msg

    def publish_pointcloud(self):
        """Convert depth and color images to pointcloud and publish."""
        if self.depth_image is None or self.color_image is None or self.camera_info is None:
            return

        try:
            # Convert ROS images to numpy arrays
            depth_array = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding='passthrough')
            color_array = self.bridge.imgmsg_to_cv2(self.color_image, desired_encoding='rgb8')

            # Get camera intrinsics
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            # Convert depth to meters (RealSense depth is in mm)
            depth_array = depth_array.astype(np.float32) / 1000.0

            # Create point cloud
            points = []
            height, width = depth_array.shape

            # Subsample for performance (skip every N pixels)
            skip = 2

            for v in range(0, height, skip):
                for u in range(0, width, skip):
                    z = depth_array[v, u]

                    # Skip invalid depth values
                    if z <= 0 or z > 10.0:  # Filter out points beyond 10m
                        continue

                    # Project to 3D
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                    # Get RGB color
                    r = color_array[v, u, 0]
                    g = color_array[v, u, 1]
                    b = color_array[v, u, 2]

                    # Pack RGB into a single float32
                    rgb = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

                    points.append([x, y, z, rgb])

            if len(points) == 0:
                return

            # Create PointCloud2 message
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_1_color_optical_frame'

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]

            points_array = np.array(points, dtype=np.float32)

            cloud_msg = PointCloud2(
                header=header,
                height=1,
                width=len(points),
                is_dense=False,
                is_bigendian=False,
                fields=fields,
                point_step=16,
                row_step=16 * len(points),
                data=points_array.tobytes()
            )

            self.pointcloud_pub.publish(cloud_msg)

        except Exception as e:
            self.get_logger().error(f'Error converting to pointcloud: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
