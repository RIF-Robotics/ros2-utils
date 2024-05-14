#!/usr/bin/env python3

import os
import time
import cv2
import math
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo



class CheckSyncTopics(Node):
    def __init__(self):
        super().__init__('check_sync_topics')

        self.tss = TimeSynchronizer([Subscriber(self, CameraInfo, '/camera_down_0/rgb/camera_info'),
                                     Subscriber(self, Image, "/camera_down_0/rgb/image_raw")],
                                    50)

        self.tss.registerCallback(self.synced_msgs_cb)


    def synced_msgs_cb(self, info_msg, image_msg):
        self.get_logger().warn(f'SYNCED')




def main(args=None):
    rclpy.init(args=args)
    check_sync_topics_node = CheckSyncTopics()

    try:
        rclpy.spin(check_sync_topics_node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
