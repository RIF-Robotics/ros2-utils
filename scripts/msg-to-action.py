#!/usr/bin/env python3

import os
import time
import cv2
import math
import numpy as np

import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, ActionClient

from std_msgs.msg import String
from rif_pick_and_place_msgs.action import AssembleTray

class MsgToAction(Node):
    def __init__(self):
        super().__init__('msg_to_action')

        self._action_client = ActionClient(self, AssembleTray, 'assemble_tray')

        self.subscription = self.create_subscription(
            String, 'activate_assemble_action', self.callback, 1)


    def callback(self, msg):
        goal_msg = AssembleTray.Goal()
        goal_msg.order = 0

        if msg.data != "activate":
            self.get_logger().warn('Unexptected msg heard on trigger node: {}'.format(msg.data))
            return

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    msg_to_action = MsgToAction()

    try:
        rclpy.spin(msg_to_action)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
