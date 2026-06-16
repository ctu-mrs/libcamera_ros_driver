#!/usr/bin/env python3
"""Check timestamp synchronization between two camera image streams.

Pairs the two streams and prints t_left - t_right for each matched pair:
  - a tight, drift-free constant  => hardware-synced (the constant is a fixed
    phase/readout offset, fine to ignore or fold into calibration);
  - a value that wanders across several ms => sensors are free-running.

Usage:
  ros2 run libcamera_ros_driver check_stereo_sync.py \
      --left  /uav1/rpi_camera_front/image_raw \
      --right /uav1/rpi_camera_back/image_raw
"""

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters


class CheckStereoSync(Node):
    def __init__(self, left_topic, right_topic, slop):
        super().__init__('check_stereo_sync')
        left = message_filters.Subscriber(self, Image, left_topic)
        right = message_filters.Subscriber(self, Image, right_topic)
        # large slop so pairs form even when NOT synced -- we want to SEE the offset
        self.ts = message_filters.ApproximateTimeSynchronizer([left, right], queue_size=30, slop=slop)
        self.ts.registerCallback(self.cb)
        self.get_logger().info(f'comparing "{left_topic}" - "{right_topic}" (slop {slop*1e3:.0f} ms)')

    @staticmethod
    def _secs(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def cb(self, lmsg, rmsg):
        dt = self._secs(lmsg.header.stamp) - self._secs(rmsg.header.stamp)
        self.get_logger().info(f'dt = {dt*1e3:+.3f} ms')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--left', required=True, help='left/first image topic')
    parser.add_argument('--right', required=True, help='right/second image topic')
    parser.add_argument('--slop', type=float, default=0.05, help='max pairing time difference [s] (default 0.05)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = CheckStereoSync(args.left, args.right, args.slop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
