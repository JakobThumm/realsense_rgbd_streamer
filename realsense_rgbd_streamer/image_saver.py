#!/usr/bin/env python3

import os
import queue
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

DEFAULT_SAVE_PATH = os.path.join(os.getcwd(), 'data')


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.declare_parameter('save_path', DEFAULT_SAVE_PATH)
        self.declare_parameter(
            'color_topic',
            '/camera/camera/color/image_raw',
        )
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )

        save_path: str = str(
            self.get_parameter('save_path').value
        )
        color_topic: str = str(
            self.get_parameter('color_topic').value
        )
        depth_topic: str = str(
            self.get_parameter('depth_topic').value
        )

        self.color_dir = os.path.join(save_path, 'color')
        self.depth_dir = os.path.join(save_path, 'depth')
        os.makedirs(self.color_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.color_count = 0
        self.depth_count = 0

        # Background write queue: items are (path, img)
        self._write_queue: queue.Queue = queue.Queue()
        self._writer_thread = threading.Thread(
            target=self._writer_loop, daemon=True
        )
        self._writer_thread.start()

        # Large queue size so bag playback at full speed doesn't drop messages
        self.create_subscription(
            Image, color_topic, self.color_callback, 1000,
        )
        self.create_subscription(
            Image, depth_topic, self.depth_callback, 1000,
        )

        self.get_logger().info(
            f'Saving color PNGs to {self.color_dir}'
        )
        self.get_logger().info(
            f'Saving depth PNGs to {self.depth_dir}'
        )

    def color_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8',
        )
        ts = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        path = os.path.join(self.color_dir, f'{ts}.png')
        self._write_queue.put(('color', path, img))

    def depth_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough',
        )
        ts = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        path = os.path.join(self.depth_dir, f'{ts}.png')
        self._write_queue.put(('depth', path, img))

    def _writer_loop(self):
        while True:
            try:
                kind, path, img = self._write_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            cv2.imwrite(path, img)
            self._write_queue.task_done()
            if kind == 'color':
                self.color_count += 1
                self.get_logger().info(
                    f'Saved color frame {self.color_count}'
                    f' (queue: {self._write_queue.qsize()})',
                    throttle_duration_sec=2.0,
                )
            else:
                self.depth_count += 1
                self.get_logger().info(
                    f'Saved depth frame {self.depth_count}'
                    f' (queue: {self._write_queue.qsize()})',
                    throttle_duration_sec=2.0,
                )

    def flush(self):
        """Block until all queued writes are complete."""
        self.get_logger().info(
            f'Flushing {self._write_queue.qsize()} remaining frames...'
        )
        self._write_queue.join()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush()
        node.get_logger().info(
            f'Done. Saved {node.color_count} color '
            f'and {node.depth_count} depth images.'
        )
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
