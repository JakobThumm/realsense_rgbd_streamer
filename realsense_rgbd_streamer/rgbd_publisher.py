#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
import json
import zstandard


class RGBDPublisher(Node):
    """
    Publishes RGBD images from RealSense camera with optional compression.

    Subscribes to RealSense camera topics and republishes RGB + Depth at
    a controlled frequency with optional compression.

    In test_connection mode, runs a ping-pong latency test against an
    rgbd_subscriber running in test_connection mode instead of normal streaming.
    """

    def __init__(self):
        super().__init__('rgbd_publisher')

        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('compress_rgb', False)
        self.declare_parameter('compress_depth', False)
        self.declare_parameter('rgb_quality', 90)  # JPEG quality 0-100
        self.declare_parameter('depth_compression_format', 'zstd')  # 'zstd' or 'png'
        self.declare_parameter('depth_zstd_level', 3)   # Zstd level 1-22
        self.declare_parameter('depth_png_compression', 3)  # PNG compression 0-9
        self.declare_parameter('camera_namespace', '/camera/camera')
        self.declare_parameter('test_connection', False)
        self.declare_parameter('test_count', 20)  # number of ping-pong rounds

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.compress_rgb = self.get_parameter('compress_rgb').value
        self.compress_depth = self.get_parameter('compress_depth').value
        self.rgb_quality = self.get_parameter('rgb_quality').value
        self.depth_compression_format = self.get_parameter('depth_compression_format').value
        self.depth_zstd_level = self.get_parameter('depth_zstd_level').value
        self.depth_png_compression = self.get_parameter('depth_png_compression').value
        self._zstd_compressor = zstandard.ZstdCompressor(level=self.depth_zstd_level)
        camera_ns = self.get_parameter('camera_namespace').value
        self.test_connection = self.get_parameter('test_connection').value
        self.test_count = int(self.get_parameter('test_count').value)

        # CV Bridge
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        if self.test_connection:
            self._init_test_mode(camera_ns)
        else:
            self._init_normal_mode(camera_ns)

    # -------------------------------------------------------------------------
    # Normal streaming mode
    # -------------------------------------------------------------------------

    def _init_normal_mode(self, camera_ns):
        """Initialize normal RGBD streaming mode."""
        # Latest images storage
        self.rgb_image = None
        self.depth_image = None
        self.rgb_timestamp = None
        self.depth_timestamp = None

        # Statistics
        self.rgb_received_count = 0
        self.depth_received_count = 0
        self.published_count = 0
        self.last_rgb_time = None
        self.last_depth_time = None
        self.compression_times = []

        # Subscribers to RealSense camera
        self.rgb_sub = self.create_subscription(
            Image,
            f'{camera_ns}/color/image_raw',
            self.rgb_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            f'{camera_ns}/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)

        # Publishers
        if self.compress_rgb:
            self.rgb_pub = self.create_publisher(
                CompressedImage,
                'rgbd_stream/rgb/compressed',
                10)
        else:
            self.rgb_pub = self.create_publisher(
                Image,
                'rgbd_stream/rgb/raw',
                10)

        if self.compress_depth:
            self.depth_pub = self.create_publisher(
                CompressedImage,
                'rgbd_stream/depth/compressed',
                10)
        else:
            self.depth_pub = self.create_publisher(
                Image,
                'rgbd_stream/depth/raw',
                10)

        # Create timer for publishing at controlled rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)

        # Status timer
        self.status_timer = self.create_timer(2.0, self.print_status)

        self.get_logger().info('=== RGBD Publisher Started ===')
        self.get_logger().info(f'Publishing rate: {self.publish_rate} Hz')
        self.get_logger().info(f'RGB compression: {self.compress_rgb}' +
                              (f' (quality={self.rgb_quality})' if self.compress_rgb else ''))
        if self.compress_depth:
            if self.depth_compression_format == 'zstd':
                depth_info = f' (zstd level={self.depth_zstd_level})'
            else:
                depth_info = f' (png level={self.depth_png_compression})'
        else:
            depth_info = ''
        self.get_logger().info(f'Depth compression: {self.compress_depth}{depth_info}')
        self.get_logger().info(f'Subscribing to: {camera_ns}/color/image_raw')
        self.get_logger().info(f'Subscribing to: {camera_ns}/aligned_depth_to_color/image_raw')

    def rgb_callback(self, msg):
        """Callback for RGB images from RealSense."""
        with self.lock:
            self.rgb_image = msg
            self.rgb_timestamp = msg.header.stamp
            self.rgb_received_count += 1

            current_time = time.time()
            if self.last_rgb_time is not None:
                fps = 1.0 / (current_time - self.last_rgb_time)
                self.get_logger().debug(f'RGB FPS: {fps:.1f}')
            self.last_rgb_time = current_time

    def depth_callback(self, msg):
        """Callback for depth images from RealSense."""
        with self.lock:
            self.depth_image = msg
            self.depth_timestamp = msg.header.stamp
            self.depth_received_count += 1

            current_time = time.time()
            if self.last_depth_time is not None:
                fps = 1.0 / (current_time - self.last_depth_time)
                self.get_logger().debug(f'Depth FPS: {fps:.1f}')
            self.last_depth_time = current_time

    def compress_rgb_image(self, cv_image, timestamp):
        """Compress RGB image to JPEG."""
        compress_start = time.time()

        # Encode to JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.rgb_quality]
        success, encoded = cv2.imencode('.jpg', cv_image, encode_param)

        if not success:
            self.get_logger().error('Failed to compress RGB image')
            return None

        # Create CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()

        compress_time = (time.time() - compress_start) * 1000  # ms
        self.compression_times.append(compress_time)

        return msg

    def compress_depth_image(self, cv_image, timestamp):
        """Compress depth image using Zstd or PNG."""
        compress_start = time.time()

        msg = CompressedImage()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'camera_depth_optical_frame'

        if self.depth_compression_format == 'zstd':
            # Compress raw uint16 bytes with Zstd (lossless, fast)
            h, w = cv_image.shape[:2]
            compressed = self._zstd_compressor.compress(cv_image.tobytes())
            msg.format = f'zstd_16UC1:{h}x{w}'
            msg.data = list(compressed)
        else:
            # Fallback: PNG
            encode_param = [int(cv2.IMWRITE_PNG_COMPRESSION), self.depth_png_compression]
            success, encoded = cv2.imencode('.png', cv_image, encode_param)
            if not success:
                self.get_logger().error('Failed to compress depth image')
                return None
            msg.format = '16UC1; png'
            msg.data = encoded.tobytes()

        compress_time = (time.time() - compress_start) * 1000  # ms
        self.compression_times.append(compress_time)

        return msg

    def publish_callback(self):
        """Timer callback to publish images at controlled rate."""
        with self.lock:
            if self.rgb_image is None or self.depth_image is None:
                self.get_logger().warn('Waiting for both RGB and depth images...',
                                      throttle_duration_sec=2.0)
                return

            # Convert to OpenCV format
            try:
                rgb_cv = self.bridge.imgmsg_to_cv2(self.rgb_image, desired_encoding='bgr8')
                depth_cv = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding='passthrough')
            except Exception as e:
                self.get_logger().error(f'Error converting images: {e}')
                return

            # Publish RGB
            if self.compress_rgb:
                rgb_msg = self.compress_rgb_image(rgb_cv, self.rgb_timestamp)
                if rgb_msg is not None:
                    self.rgb_pub.publish(rgb_msg)
            else:
                self.rgb_pub.publish(self.rgb_image)

            # Publish Depth
            if self.compress_depth:
                depth_msg = self.compress_depth_image(depth_cv, self.depth_timestamp)
                if depth_msg is not None:
                    self.depth_pub.publish(depth_msg)
            else:
                self.depth_pub.publish(self.depth_image)

            self.published_count += 1

            self.get_logger().debug(f'Published frame {self.published_count}')

    def print_status(self):
        """Print status information."""
        with self.lock:
            avg_compression_time = 0.0
            if len(self.compression_times) > 0:
                avg_compression_time = sum(self.compression_times) / len(self.compression_times)
                self.compression_times = []  # Reset

            status = f'Status: RGB received={self.rgb_received_count}, ' \
                    f'Depth received={self.depth_received_count}, ' \
                    f'Published={self.published_count}'

            if avg_compression_time > 0:
                status += f', Avg compression time={avg_compression_time:.2f}ms'

            self.get_logger().info(status)

    # -------------------------------------------------------------------------
    # Connection test mode
    # -------------------------------------------------------------------------

    def _init_test_mode(self, camera_ns):
        """Initialize connection latency test mode.

        Sends CompressedImage pings to connection_test/ping and waits for
        JSON pongs on connection_test/pong. Each round measures the time for:

          Publisher side (local clocks, accurate):
            1. Bridge conversion  : ROS Image → OpenCV ndarray
            2. JPEG compression   : cv2.imencode
            3. Msg build + publish: CompressedImage construction + publish()

          Network (estimated, assumes symmetric):
            4. publisher → subscriber  (one-way)
            5. subscriber → publisher  (one-way)

          Subscriber side (local clocks, sent back in pong):
            6. JPEG decompression: np.frombuffer + cv2.imdecode
            7. Subscriber overhead: time between decompression done and pong sent

        The one-way network estimate is computed as:
            (full_RTT - subscriber_total) / 2
        where subscriber_total = t_pong_sent - t_msg_received.
        """
        self.test_seq = 0
        self.test_results = []
        self.pending_test = None   # dict with in-flight timestamps
        self.test_done = False

        # Publish pings (CompressedImage)
        self.ping_pub = self.create_publisher(
            CompressedImage, 'connection_test/ping', 10)

        # Receive pongs (JSON string)
        self.pong_sub = self.create_subscription(
            String, 'connection_test/pong', self.pong_callback, 10)

        # Subscribe to camera RGB only (depth not needed for latency test)
        self.rgb_sub = self.create_subscription(
            Image,
            f'{camera_ns}/color/image_raw',
            self.test_rgb_callback,
            10)

        self.get_logger().info('=== RGBD Publisher: Connection Test Mode ===')
        self.get_logger().info(f'Rounds       : {self.test_count}')
        self.get_logger().info(f'JPEG quality : {self.rgb_quality}')
        self.get_logger().info(f'Ping topic   : connection_test/ping')
        self.get_logger().info(f'Pong topic   : connection_test/pong')
        self.get_logger().info('Waiting for camera frames and subscriber ...')

    def test_rgb_callback(self, msg):
        """Test mode: send one ping per received camera frame.

        Only one ping is in-flight at a time; subsequent frames are skipped
        until the current pong is received.
        """
        with self.lock:
            if self.test_done or self.test_seq >= self.test_count:
                return
            if self.pending_test is not None:
                return  # still waiting for previous pong

            # --- Step 1: Bridge conversion (ROS Image → OpenCV) ---
            t_frame_received = time.time_ns()
            try:
                rgb_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Error converting image: {e}')
                return
            t_bridge_done = time.time_ns()

            # --- Step 2: JPEG compression ---
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.rgb_quality]
            success, encoded = cv2.imencode('.jpg', rgb_cv, encode_param)
            t_compressed = time.time_ns()

            if not success:
                self.get_logger().error('Failed to compress image for ping')
                return

            seq = self.test_seq

            # Build ping message.
            # Publisher-side timestamps are embedded in frame_id so the subscriber
            # can echo them back in the pong (not strictly required here, but useful
            # for debugging on the subscriber side).
            # Format: "latency_test:{seq}:{t_frame_received}:{t_bridge_done}:{t_compressed}:{t_sent}"
            # t_sent is recorded just before publish() below.
            ping_msg = CompressedImage()
            ping_msg.header.stamp = msg.header.stamp  # original camera capture time
            ping_msg.format = 'jpeg'
            ping_msg.data = encoded.tobytes()

            # --- Step 3: Msg build + publish ---
            t_sent = time.time_ns()
            ping_msg.header.frame_id = (
                f'latency_test:{seq}:{t_frame_received}:'
                f'{t_bridge_done}:{t_compressed}:{t_sent}'
            )

            self.pending_test = {
                'seq': seq,
                't_frame_received': t_frame_received,
                't_bridge_done': t_bridge_done,
                't_compressed': t_compressed,
                't_sent': t_sent,
            }

            self.ping_pub.publish(ping_msg)
            self.test_seq += 1

            bridge_ms = (t_bridge_done - t_frame_received) / 1e6
            compress_ms = (t_compressed - t_bridge_done) / 1e6
            size_kb = len(ping_msg.data) / 1024
            self.get_logger().info(
                f'[Ping {seq:3d}/{self.test_count - 1}] Sent | '
                f'bridge={bridge_ms:.1f}ms  compress={compress_ms:.1f}ms  '
                f'size={size_kb:.1f} KB'
            )

    def pong_callback(self, msg):
        """Receive pong and compute per-step latency breakdown."""
        t_pong_received = time.time_ns()

        with self.lock:
            if self.pending_test is None:
                return

            try:
                pong = json.loads(msg.data)
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Failed to parse pong JSON: {e}')
                return

            if pong['seq'] != self.pending_test['seq']:
                self.get_logger().warn(
                    f'Seq mismatch: got {pong["seq"]}, expected {self.pending_test["seq"]}')
                return

            p = self.pending_test
            self.pending_test = None

            # Subscriber-side timestamps (nanoseconds, subscriber clock)
            t_msg_received = pong['t_msg_received']
            t_decomp_done = pong['t_decomp_done']
            t_pong_sent = pong['t_pong_sent']

            # --- Compute step durations ---
            bridge_ms       = (p['t_bridge_done'] - p['t_frame_received']) / 1e6
            compress_ms     = (p['t_compressed']  - p['t_bridge_done'])    / 1e6
            msg_build_ms    = (p['t_sent']         - p['t_compressed'])    / 1e6
            full_rtt_ms     = (t_pong_received     - p['t_sent'])          / 1e6
            sub_total_ms    = (t_pong_sent         - t_msg_received)       / 1e6
            decomp_ms       = (t_decomp_done       - t_msg_received)       / 1e6
            sub_overhead_ms = (t_pong_sent         - t_decomp_done)        / 1e6
            # Combined network time (both directions together).
            # Not split into one-way estimates because the ping carries a large
            # compressed image while the pong is ~200 bytes of JSON, so the two
            # legs have inherently different transit times.
            net_combined_ms = max(0.0, full_rtt_ms - sub_total_ms)

            # Total = everything from frame received in callback to pong received
            total_ms = (t_pong_received - p['t_frame_received']) / 1e6

            result = {
                'seq': pong['seq'],
                'bridge_ms': bridge_ms,
                'compress_ms': compress_ms,
                'msg_build_ms': msg_build_ms,
                'net_combined_ms': net_combined_ms,
                'decomp_ms': decomp_ms,
                'sub_overhead_ms': sub_overhead_ms,
                'sub_total_ms': sub_total_ms,
                'full_rtt_ms': full_rtt_ms,
                'total_ms': total_ms,
                'size_kb': pong.get('size_kb', 0.0),
            }
            self.test_results.append(result)

            self.get_logger().info(
                f'[Pong {pong["seq"]:3d}/{self.test_count - 1}] '
                f'total={total_ms:.1f}ms  '
                f'bridge={bridge_ms:.1f}ms  '
                f'compress={compress_ms:.1f}ms  '
                f'net(both)={net_combined_ms:.1f}ms  '
                f'decomp={decomp_ms:.1f}ms'
            )

            if len(self.test_results) >= self.test_count:
                self.test_done = True
                self._print_test_summary()

    def _print_test_summary(self):
        """Print min/mean/max table for all measured steps."""
        r = self.test_results
        n = len(r)
        if n == 0:
            return

        def stats(key):
            vals = [x[key] for x in r]
            return min(vals), sum(vals) / n, max(vals)

        bridge_s    = stats('bridge_ms')
        compress_s  = stats('compress_ms')
        build_s     = stats('msg_build_ms')
        net_s       = stats('net_combined_ms')
        decomp_s    = stats('decomp_ms')
        sub_over_s  = stats('sub_overhead_ms')
        total_s     = stats('total_ms')

        bar = '=' * 70
        self.get_logger().info(bar)
        self.get_logger().info(f'CONNECTION TEST SUMMARY  ({n} frames, JPEG quality={self.rgb_quality})')
        self.get_logger().info(bar)
        self.get_logger().info(
            f'{"Step":<44} {"min":>6} {"mean":>7} {"max":>7}  ms')
        self.get_logger().info('-' * 70)
        self.get_logger().info(
            f'{"1. Bridge conversion  (pub, ROS→OpenCV)":<44} '
            f'{bridge_s[0]:6.1f} {bridge_s[1]:7.1f} {bridge_s[2]:7.1f}')
        self.get_logger().info(
            f'{"2. JPEG compression   (pub)":<44} '
            f'{compress_s[0]:6.1f} {compress_s[1]:7.1f} {compress_s[2]:7.1f}')
        self.get_logger().info(
            f'{"3. Msg build + publish call (pub)":<44} '
            f'{build_s[0]:6.1f} {build_s[1]:7.1f} {build_s[2]:7.1f}')
        self.get_logger().info(
            f'{"4. JPEG decompression (sub)":<44} '
            f'{decomp_s[0]:6.1f} {decomp_s[1]:7.1f} {decomp_s[2]:7.1f}')
        self.get_logger().info(
            f'{"5. Sub overhead (decomp done → pong sent)":<44} '
            f'{sub_over_s[0]:6.1f} {sub_over_s[1]:7.1f} {sub_over_s[2]:7.1f}')
        self.get_logger().info('-' * 70)
        self.get_logger().info(
            f'{"Network total (pub→sub + sub→pub)":<44} '
            f'{net_s[0]:6.1f} {net_s[1]:7.1f} {net_s[2]:7.1f}')
        self.get_logger().info('-' * 70)
        self.get_logger().info(
            f'{"Total  (frame received → pong received)":<44} '
            f'{total_s[0]:6.1f} {total_s[1]:7.1f} {total_s[2]:7.1f}')
        self.get_logger().info(bar)
        self.get_logger().info(
            'Note: "Network total" = RTT - subscriber_total (both directions combined).')
        self.get_logger().info(
            '      It is not split further: the image ping is ~100KB while the')
        self.get_logger().info(
            '      JSON pong is ~200B, so the two legs are not comparable.')
        self.get_logger().info(bar)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RGBDPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
