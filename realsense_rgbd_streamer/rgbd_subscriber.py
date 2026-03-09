#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os
import json
import zstandard


class RGBDSubscriber(Node):
    """
    Subscribes to RGBD stream and displays/saves images.
    Handles both compressed and uncompressed formats.

    In test_connection mode, acts as the pong side of a ping-pong latency
    test against an rgbd_publisher running in test_connection mode.
    """

    def __init__(self):
        super().__init__('rgbd_subscriber')

        # Declare parameters
        self.declare_parameter('compressed', True)
        self.declare_parameter('display', True)
        self.declare_parameter('save_path', '')
        self.declare_parameter('save_rate', 1.0)  # Save every N seconds
        self.declare_parameter('test_connection', False)

        # Get parameters
        self.compressed = self.get_parameter('compressed').value
        self.display = self.get_parameter('display').value
        self.save_path = self.get_parameter('save_path').value
        self.save_rate = self.get_parameter('save_rate').value
        self.test_connection = self.get_parameter('test_connection').value

        # CV Bridge
        self.bridge = CvBridge()

        if self.test_connection:
            self._init_test_mode()
        else:
            self._init_normal_mode()

    # -------------------------------------------------------------------------
    # Normal streaming mode
    # -------------------------------------------------------------------------

    def _init_normal_mode(self):
        """Initialize normal RGBD display/save mode."""
        # State
        self.rgb_image = None
        self.depth_image = None
        self.rgb_timestamp = None
        self.depth_timestamp = None
        self.last_save_time = 0
        self.frame_count = 0

        # Statistics
        self.rgb_received = 0
        self.depth_received = 0
        self.last_stats_time = time.time()
        self.decompression_times = []

        # Create save directory if needed
        if self.save_path and not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            self.get_logger().info(f'Created save directory: {self.save_path}')

        # Subscribe based on compression setting
        if self.compressed:
            self.rgb_sub = self.create_subscription(
                CompressedImage,
                'rgbd_stream/rgb/compressed',
                self.rgb_compressed_callback,
                10)

            self.depth_sub = self.create_subscription(
                CompressedImage,
                'rgbd_stream/depth/compressed',
                self.depth_compressed_callback,
                10)
        else:
            self.rgb_sub = self.create_subscription(
                Image,
                'rgbd_stream/rgb/raw',
                self.rgb_raw_callback,
                10)

            self.depth_sub = self.create_subscription(
                Image,
                'rgbd_stream/depth/raw',
                self.depth_raw_callback,
                10)

        # Timer for display and stats
        self.timer = self.create_timer(0.033, self.update_display)  # ~30 Hz
        self.stats_timer = self.create_timer(2.0, self.print_stats)

        self.get_logger().info('=== RGBD Subscriber Started ===')
        self.get_logger().info(f'Compressed mode: {self.compressed}')
        self.get_logger().info(f'Display: {self.display}')
        if self.save_path:
            self.get_logger().info(f'Save path: {self.save_path}')
            self.get_logger().info(f'Save rate: {self.save_rate} seconds')

    def rgb_compressed_callback(self, msg):
        """Callback for compressed RGB images."""
        decompress_start = time.time()

        try:
            # Decode JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error('Failed to decode RGB image')
                return

            self.rgb_image = cv_image
            self.rgb_timestamp = msg.header.stamp
            self.rgb_received += 1

            decompress_time = (time.time() - decompress_start) * 1000
            self.decompression_times.append(decompress_time)

        except Exception as e:
            self.get_logger().error(f'Error in RGB callback: {e}')

    def depth_compressed_callback(self, msg):
        """Callback for compressed depth images (Zstd or PNG)."""
        decompress_start = time.time()

        try:
            if msg.format.startswith('zstd_16UC1:'):
                # Zstd-compressed raw uint16 bytes
                dims = msg.format.split(':')[1]
                h, w = map(int, dims.split('x'))
                raw = zstandard.ZstdDecompressor().decompress(bytes(msg.data))
                cv_image = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
            else:
                # PNG fallback
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_ANYDEPTH)

            if cv_image is None:
                self.get_logger().error('Failed to decode depth image')
                return

            self.depth_image = cv_image
            self.depth_timestamp = msg.header.stamp
            self.depth_received += 1

            decompress_time = (time.time() - decompress_start) * 1000
            self.decompression_times.append(decompress_time)

        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {e}')

    def rgb_raw_callback(self, msg):
        """Callback for raw RGB images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_image = cv_image
            self.rgb_timestamp = msg.header.stamp
            self.rgb_received += 1
        except Exception as e:
            self.get_logger().error(f'Error in RGB callback: {e}')

    def depth_raw_callback(self, msg):
        """Callback for raw depth images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image = cv_image
            self.depth_timestamp = msg.header.stamp
            self.depth_received += 1
        except Exception as e:
            self.get_logger().error(f'Error in depth callback: {e}')

    def update_display(self):
        """Update display and save images."""
        if self.rgb_image is None or self.depth_image is None:
            return

        # Display images
        if self.display:
            # Display RGB
            cv2.imshow('RGB Stream', self.rgb_image)

            # Display depth (normalized for visualization)
            depth_normalized = cv2.normalize(self.depth_image, None, 0, 255,
                                            cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            cv2.imshow('Depth Stream', depth_colored)

            # Quit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Quit signal received')
                rclpy.shutdown()

        # Save images at specified rate
        if self.save_path:
            current_time = time.time()
            if current_time - self.last_save_time >= self.save_rate:
                self.save_images()
                self.last_save_time = current_time

    def save_images(self):
        """Save RGB and depth images to disk."""
        if self.rgb_image is None or self.depth_image is None:
            return

        try:
            # Create timestamped filenames
            timestamp_sec = self.rgb_timestamp.sec
            timestamp_nsec = self.rgb_timestamp.nanosec
            timestamp_str = f'{timestamp_sec}_{timestamp_nsec:09d}'

            # Save RGB
            rgb_filename = os.path.join(self.save_path,
                                       f'rgb_{timestamp_str}_{self.frame_count:06d}.jpg')
            cv2.imwrite(rgb_filename, self.rgb_image)

            # Save depth as 16-bit PNG
            depth_filename = os.path.join(self.save_path,
                                         f'depth_{timestamp_str}_{self.frame_count:06d}.png')
            cv2.imwrite(depth_filename, self.depth_image)

            self.frame_count += 1
            self.get_logger().info(f'Saved frame {self.frame_count}',
                                  throttle_duration_sec=2.0)

        except Exception as e:
            self.get_logger().error(f'Error saving images: {e}')

    def print_stats(self):
        """Print statistics."""
        current_time = time.time()
        elapsed = current_time - self.last_stats_time

        rgb_fps = self.rgb_received / elapsed if elapsed > 0 else 0
        depth_fps = self.depth_received / elapsed if elapsed > 0 else 0

        avg_decompress_time = 0.0
        if len(self.decompression_times) > 0:
            avg_decompress_time = sum(self.decompression_times) / len(self.decompression_times)

        status = f'Stats: RGB={rgb_fps:.1f} fps, Depth={depth_fps:.1f} fps'

        if self.compressed and avg_decompress_time > 0:
            status += f', Avg decompress={avg_decompress_time:.2f}ms'

        if self.save_path:
            status += f', Saved={self.frame_count} frames'

        self.get_logger().info(status)

        # Reset counters
        self.rgb_received = 0
        self.depth_received = 0
        self.last_stats_time = current_time
        self.decompression_times = []

    # -------------------------------------------------------------------------
    # Connection test mode
    # -------------------------------------------------------------------------

    def _init_test_mode(self):
        """Initialize connection latency test mode (pong side).

        Subscribes to connection_test/ping, decompresses each image, and
        immediately publishes a JSON pong on connection_test/pong containing
        the subscriber-side timestamps so the publisher can compute per-step
        latency.

        Pong JSON fields:
          seq            : ping sequence number (int)
          t_msg_received : time.time_ns() at start of ping callback
          t_decomp_done  : time.time_ns() after cv2.imdecode completes
          t_pong_sent    : time.time_ns() just before pong publish()
          size_kb        : compressed image size in kilobytes (float)
        """
        self.pings_received = 0

        # Subscribe to pings
        self.ping_sub = self.create_subscription(
            CompressedImage,
            'connection_test/ping',
            self.ping_callback,
            10)

        # Publish pongs
        self.pong_pub = self.create_publisher(
            String, 'connection_test/pong', 10)

        self.get_logger().info('=== RGBD Subscriber: Connection Test Mode ===')
        self.get_logger().info('Ping topic : connection_test/ping')
        self.get_logger().info('Pong topic : connection_test/pong')
        self.get_logger().info('Waiting for pings from publisher ...')

    def ping_callback(self, msg):
        """Receive a ping, decompress the image, and send a pong.

        Timing sequence (all via time.time_ns()):
          t_msg_received  — very first thing in this callback
          [np.frombuffer + cv2.imdecode]
          t_decomp_done   — right after decompression
          [build pong JSON]
          t_pong_sent     — just before pong publish()
        """
        # --- Record receive time as the very first operation ---
        t_msg_received = time.time_ns()

        # Parse sequence number from frame_id
        # Format set by publisher: "latency_test:{seq}:..."
        seq = -1
        frame_parts = msg.header.frame_id.split(':')
        if len(frame_parts) >= 2 and frame_parts[0] == 'latency_test':
            try:
                seq = int(frame_parts[1])
            except ValueError:
                pass

        if seq < 0:
            self.get_logger().error(
                f'Received ping with unexpected frame_id: "{msg.header.frame_id}"')
            return

        # --- Decompress JPEG ---
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        t_decomp_done = time.time_ns()

        if cv_image is None:
            self.get_logger().error(f'[Ping {seq}] Failed to decompress JPEG')
            return

        size_kb = len(msg.data) / 1024

        # --- Build and send pong ---
        pong_data = {
            'seq': seq,
            't_msg_received': t_msg_received,
            't_decomp_done': t_decomp_done,
            't_pong_sent': 0,        # filled in just below
            'size_kb': round(size_kb, 2),
        }

        t_pong_sent = time.time_ns()
        pong_data['t_pong_sent'] = t_pong_sent

        pong_msg = String()
        pong_msg.data = json.dumps(pong_data)
        self.pong_pub.publish(pong_msg)

        self.pings_received += 1
        decomp_ms = (t_decomp_done - t_msg_received) / 1e6
        overhead_ms = (t_pong_sent - t_decomp_done) / 1e6

        self.get_logger().info(
            f'[Ping {seq:3d}] Received & pong sent | '
            f'decomp={decomp_ms:.1f}ms  overhead={overhead_ms:.1f}ms  '
            f'size={size_kb:.1f} KB'
        )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RGBDSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
