#!/usr/bin/env python3

# This node filters out some messa

import os
import time
import cv2
import math
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage

class tfMsgFilter(Node):
    def __init__(self):
        super().__init__('tf_msg_filter')

        # Declare parameters defined in the launch file
        self.declare_parameter('filter_field', 'child_frame_id')
        # self.declare_parameter('filter_value', 'camera_link')

        self.filter_field = self.get_parameter('filter_field').get_parameter_value().string_value

        # TODO: turn these parameters into launch file parameters
        self.filter_value = ['scalpel_handle_track', \
                             'allis_track', \
                             'or_scissors_track', \
                             'dressing_forceps_track', \
                             'scalpel_handle_det', \
                             'allis_det', \
                             'or_scissors_det', \
                             'dressing_forceps_det']

        self.pub_filtered_tf = self.create_publisher(TFMessage, 'tf', 10)

        self.subscription = self.create_subscription(
            TFMessage, '/tf_original',
            self.tf_callback, 1)


    def tf_callback(self, msg):
        for t in msg.transforms:
            # debug
            print(t)


        # # TODO: construct the new tf message and publish
        # msg = TFMessage()
        # self.pub_filtered_tf.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    tf_msg_filter = tfMsgFilter()

    try:
        rclpy.spin(tf_msg_filter)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
