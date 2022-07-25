#!/usr/bin/env python3

import os
import time
import cv2
import math
import numpy as np

import rclpy
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, ActionClient

from std_msgs.msg import String, Bool
from rif_pick_and_place_msgs.action import AssembleTray

class MsgToAction(Node):
    def __init__(self):
        super().__init__('msg_to_action')

        self._goal_handle = None
        self._action_client = ActionClient(self, AssembleTray, 'assemble_tray')

        self.subscription = self.create_subscription(
            String, 'activate_assemble_action', self.command_callback, 1)

        self.pub_cancel_place = self.create_publisher(Bool, 'cancel_place', 1)
        self.pub_cancel_pick_up = self.create_publisher(Bool, 'cancel_pick_up', 1)


    def command_callback(self, msg):
        self.get_logger().info('Assembly action message received: {}'.format(msg.data))

        if msg.data == "activate":
            self.trigger_assembly()
        elif msg.data == "cancel":
            self.cancel_assembly()
            cancel_msg = Bool()
            cancel_msg.data = True
            self.pub_cancel_place.publish(cancel_msg)
            self.pub_cancel_pick_up.publish(cancel_msg)
        else:
            self.get_logger().warn('Unexptected msg heard on trigger node: {}'.format(msg.data))


    def trigger_assembly(self):
        goal_msg = AssembleTray.Goal()
        goal_msg.order = 0
        self._action_client.wait_for_server()
        self._assembly_future = self._action_client.send_goal_async(goal_msg)
        self._assembly_future.add_done_callback(self.assembly_response_callback)


    def assembly_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle

        self._get_assembly_result_future = goal_handle.get_result_async()
        self._get_assembly_result_future.add_done_callback(self.get_assembly_result_callback)


    def get_assembly_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {}'.format(result))
        else:
            self.get_logger().info('Goal failed with status: {}'.format(status))


    def cancel_assembly(self):
        if self._goal_handle is None:
            self.get_logger().info('Cancel goal msg received. No action to cancel.')
            return

        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)


    def cancel_done(self, future):
        cancel_response = future.result()
        self.get_logger().info('Goal canceled: {}'.format(cancel_response))


def main(args=None):
    rclpy.init(args=args)
    msg_to_action = MsgToAction()

    try:
        rclpy.spin(msg_to_action)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
