#!/usr/bin/env python3

from typing import Optional

import rclpy

# Node, State and Publisher are aliases for LifecycleNode, LifecycleState and LifecyclePublisher
# respectively.
# In case of ambiguity, the more explicit names can be imported.

from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo

import std_msgs.msg


class BallLocatorNode(Node):

    def __init__(self):
        self.bridge = CvBridge()    
        super().__init__('tt_ball_locator')
        
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        self.sub = self.create_subscription(Image, 'head_front_camera/rgb/image_raw', self.callback, 10)
        self.sub2 = self.create_subscription(CameraInfo, '/head_front_camera/depth_registered/camera_info', self.camera_info_callback, 10)

        return super().on_activate(state)
    
    def callback(self, msg):
        self.get_logger().info('Keep it up brah!')

    def camera_info_callback(self, msg):
        # fx = msg.k[0]  # Focal length in x-direction
        # fy = msg.k[4]  # Focal length in y-direction

        # self.get_logger().info(f'fx and fy: [{fx}, {fy}]')
        self.get_logger().info(f'fx and fy: [{msg}]')

def main():
  rclpy.init()

  executor = rclpy.executors.SingleThreadedExecutor()
  lc_node = BallLocatorNode()
  executor.add_node(lc_node)
  try:
    executor.spin()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    lc_node.destroy_node()

if __name__ == '__main__':
  main()