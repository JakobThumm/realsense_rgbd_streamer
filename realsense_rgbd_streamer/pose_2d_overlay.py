#!/usr/bin/env python3
"""
Node to overlay 2D pose keypoints on RGB image for visualization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from uq_msgs.msg import Pose2D, Pose3D, MotionPrediction
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
import cv2
import numpy as np


class Pose2DOverlay(Node):
    def __init__(self):
        super().__init__('pose_2d_overlay')

        self.bridge = CvBridge()

        # Latest data
        self.latest_image = None
        self.latest_pose = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            Pose2D,
            '/uq/pose_2d',
            self.pose_callback,
            10
        )

        # Publisher for annotated image
        self.overlay_pub = self.create_publisher(
            Image,
            '/uq/pose_2d_overlay',
            10
        )

        # MotionPrediction -> MarkerArray visualizer
        self.motion_pred_sub = self.create_subscription(
            MotionPrediction,
            '/uq/motion_prediction',
            self.motion_prediction_callback,
            10
        )
        self.motion_pred_pub = self.create_publisher(
            MarkerArray,
            '/uq/motion_prediction_vis',
            10
        )

        # Pose3D -> MarkerArray converter
        self.pose3d_sub = self.create_subscription(
            Pose3D,
            '/uq/pose_3d',
            self.pose3d_callback,
            10
        )
        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            '/uq/pose_3d_vis',
            10
        )

        # Joint names for interpretability (13 joints)
        self.joint_names = [
            'Nose', 'LShoulder', 'RShoulder', 'LElbow', 'RElbow',
            'LWrist', 'RWrist', 'LHip', 'RHip', 'LKnee', 'RKnee',
            'LAnkle', 'RAnkle'
        ]

        # Define skeleton connections for 13-joint model
        self.skeleton = [
            (0, 1), (0, 2),       # Nose to shoulders
            (1, 3), (3, 5),       # Left arm
            (2, 4), (4, 6),       # Right arm
            (1, 2), (1, 7), (2, 8),  # Shoulders to hips
            (7, 8),               # Connect hips
            (7, 9), (9, 11),      # Left leg
            (8, 10), (10, 12)     # Right leg
        ]

        self.get_logger().info('Pose 2D Overlay node initialized (13-joint model)')
        self.get_logger().info('Publishing overlayed image to /uq/pose_2d_overlay')

    def motion_prediction_callback(self, msg: MotionPrediction):
        """Visualize predicted joint positions at time step 8 (index 7)."""
        if not msg.is_valid or msg.n_joints == 0:
            return

        t = 7  # 8th predicted time step (0-based)
        if t >= msg.prediction_horizon_length:
            return

        pts = msg.motion_predicted
        radii = msg.set_radius
        n = msg.n_joints
        markers = []
        for j in range(n):
            diameter = radii[t * n + j] / 1000.0 * 2
            m = Marker()
            m.header = msg.header
            m.ns = 'motion_prediction'
            m.id = j
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pts[(t * n + j) * 3] / 1000.0
            m.pose.position.y = pts[(t * n + j) * 3 + 1] / 1000.0
            m.pose.position.z = pts[(t * n + j) * 3 + 2] / 1000.0
            m.pose.orientation.w = 1.0
            m.scale.x = diameter
            m.scale.y = diameter
            m.scale.z = diameter
            m.color = ColorRGBA(r=0.0, g=0.6, b=1.0, a=0.6)
            markers.append(m)

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.motion_pred_pub.publish(marker_array)

    def pose3d_callback(self, msg: Pose3D):
        """Convert Pose3D to MarkerArray (joints + skeleton) and republish for RViz."""
        if not msg.human_detected or msg.n_joints == 0:
            return

        pts = msg.points_3d
        joints = []
        for i in range(msg.n_joints):
            p = Point()
            p.x = pts[i * 3] / 1000.0
            p.y = pts[i * 3 + 1] / 1000.0
            p.z = pts[i * 3 + 2] / 1000.0
            joints.append(p)

        # Joints marker (spheres)
        joint_marker = Marker()
        joint_marker.header = msg.header
        joint_marker.ns = 'joints'
        joint_marker.id = 0
        joint_marker.type = Marker.SPHERE_LIST
        joint_marker.action = Marker.ADD
        joint_marker.scale.x = 0.04
        joint_marker.scale.y = 0.04
        joint_marker.scale.z = 0.04
        joint_marker.color.r = 1.0
        joint_marker.color.g = 0.0
        joint_marker.color.b = 0.0
        joint_marker.color.a = 1.0
        joint_marker.points = joints

        # Bones marker (lines)
        bone_marker = Marker()
        bone_marker.header = msg.header
        bone_marker.ns = 'bones'
        bone_marker.id = 1
        bone_marker.type = Marker.LINE_LIST
        bone_marker.action = Marker.ADD
        bone_marker.scale.x = 0.02
        bone_marker.color.r = 0.0
        bone_marker.color.g = 1.0
        bone_marker.color.b = 0.0
        bone_marker.color.a = 1.0
        for idx1, idx2 in self.skeleton:
            if idx1 >= msg.n_joints or idx2 >= msg.n_joints:
                continue
            bone_marker.points.append(joints[idx1])
            bone_marker.points.append(joints[idx2])

        marker_array = MarkerArray()
        marker_array.markers = [joint_marker, bone_marker]
        self.marker_array_pub.publish(marker_array)

    def image_callback(self, msg):
        """Store latest RGB image and trigger overlay publish."""
        self.latest_image = msg
        self.publish_overlay()

    def pose_callback(self, msg):
        """Store latest 2D pose data."""
        self.latest_pose = msg

    def publish_overlay(self):
        """Draw 2D pose on image and publish."""
        if self.latest_image is None or self.latest_pose is None:
            return

        try:
            # Skip if no human detected
            if not self.latest_pose.human_detected:
                return

            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

            # Parse pose data from Pose2D message
            pose_data = np.array(self.latest_pose.keypoints_2d)

            # Reshape to (N, 2) where N is number of keypoints
            if len(pose_data) % 2 != 0:
                self.get_logger().warn(f'Invalid pose data length: {len(pose_data)}')
                return

            num_keypoints = len(pose_data) // 2
            keypoints = pose_data.reshape(num_keypoints, 2)

            # Verify we have the expected number of joints
            if num_keypoints != 13:
                self.get_logger().warn(f'Expected 13 joints, got {num_keypoints}')
                # Continue anyway, but may not draw all connections

            # Draw skeleton connections
            for connection in self.skeleton:
                idx1, idx2 = connection

                # Check if indices are valid
                if idx1 >= num_keypoints or idx2 >= num_keypoints:
                    continue

                pt1 = keypoints[idx1]
                pt2 = keypoints[idx2]

                # Skip if either point is invalid (e.g., 0,0 or negative)
                if (pt1[0] <= 0 or pt1[1] <= 0 or
                    pt2[0] <= 0 or pt2[1] <= 0):
                    continue

                # Convert to integer pixel coordinates
                pt1_int = (int(pt1[0]), int(pt1[1]))
                pt2_int = (int(pt2[0]), int(pt2[1]))

                # Draw line between keypoints
                cv2.line(cv_image, pt1_int, pt2_int, (0, 255, 0), 2)

            # Draw keypoints
            for i, kp in enumerate(keypoints):
                x, y = kp

                # Skip invalid keypoints
                if x <= 0 or y <= 0:
                    continue

                # Convert to integer coordinates
                center = (int(x), int(y))

                # Draw circle for each keypoint
                cv2.circle(cv_image, center, 4, (0, 0, 255), -1)

                # Optionally draw keypoint index
                # cv2.putText(cv_image, str(i), center,
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

            # Convert back to ROS message
            overlay_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            overlay_msg.header = self.latest_image.header

            # Publish
            self.overlay_pub.publish(overlay_msg)

        except Exception as e:
            self.get_logger().error(f'Error creating overlay: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = Pose2DOverlay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
