#!/usr/bin/env python3
"""Check timestamp synchronization between two camera image streams.

Pairs the two streams and prints t_left - t_right for each matched pair:
  - a tight, drift-free constant  => hardware-synced (the constant is a fixed
    phase/readout offset, fine to ignore or fold into calibration);
  - a value that wanders across several ms => sensors are free-running.

Usage:
  rosrun libcamera_ros_driver check_stereo_sync.py \
      --left  /uav1/camera_left/image_raw \
      --right /uav1/camera_right/image_raw
"""

import argparse

import rospy
from sensor_msgs.msg import Image
import message_filters


class CheckStereoSync(object):
    def __init__(self, left_topic, right_topic, slop):
        left = message_filters.Subscriber(left_topic, Image)
        right = message_filters.Subscriber(right_topic, Image)
        # large slop so pairs form even when NOT synced -- we want to SEE the offset
        self.ts = message_filters.ApproximateTimeSynchronizer([left, right], queue_size=30, slop=slop)
        self.ts.registerCallback(self.cb)
        rospy.loginfo('comparing "%s" - "%s" (slop %.0f ms)' % (left_topic, right_topic, slop * 1e3))

    def cb(self, lmsg, rmsg):
        dt = lmsg.header.stamp.to_sec() - rmsg.header.stamp.to_sec()
        rospy.loginfo('dt = %+.3f ms' % (dt * 1e3))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--left', required=True, help='left/first image topic')
    parser.add_argument('--right', required=True, help='right/second image topic')
    parser.add_argument('--slop', type=float, default=0.05, help='max pairing time difference [s] (default 0.05)')
    args, _ = parser.parse_known_args()

    rospy.init_node('check_stereo_sync', anonymous=True)
    CheckStereoSync(args.left, args.right, args.slop)
    rospy.spin()


if __name__ == '__main__':
    main()
