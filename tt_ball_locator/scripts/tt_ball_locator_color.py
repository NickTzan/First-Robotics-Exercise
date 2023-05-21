#!/usr/bin/env python3

from typing import Optional

import rclpy

# Node, State and Publisher are aliases for LifecycleNode, LifecycleState and LifecyclePublisher
# respectively.
# In case of ambiguity, the more explicit names can be imported.

from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped

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
        # Initialize variables
        self.rgb_image = None
        self.depth_image = None
        self.depth_scale = 0.001
        # Subscribe to the RGB image topic
        self.rgb_sub = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.rgb_callback, 10)
        # Subscribe to the depth image topic
        self.depth_sub = self.create_subscription(Image, '/head_front_camera/depth_registered/image_raw', self.depth_callback, 10)
        return super().on_activate(state)
    
    def rgb_callback(self, msg):
        self.rgb_image = msg
        
    def depth_callback(self, msg):
        self.depth_image = msg
        if self.rgb_image is not None:
            # Convert the RGB image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.rgb_image, desired_encoding="passthrough")

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
 
            blur = cv2.GaussianBlur(gray, (5, 5), cv2.BORDER_DEFAULT)
            ret, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY_INV)

            contours, hierarchies = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            self.get_logger().info(f"Number of Contours: {len(contours)}")

            # Calculate the distance to the ball using depth information
            cv_depth_image = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding="passthrough")

            locations = []
            for i in contours:
                M = cv2.moments(i)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    depth = cv_depth_image[cy, cx] * 0.001
                    locations.append([cx, cy, depth])

                self.get_logger().info(f"x: {locations[0][0]} y: {locations[0][1]} depth:{locations[0][2]}")

            broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = self.get_clock().now().to_msg()
            static_transformStamped.header.frame_id = 'head_front_camera_rgb_optical_frame'
            static_transformStamped.child_frame_id = 'tt_ball_center_link'
            static_transformStamped.transform.translation.x = locations[0][0]
            static_transformStamped.transform.translation.y = locations[0][1]
            static_transformStamped.transform.translation.z = locations[0][2]
            static_transformStamped.transform.rotation.x = 1.0
            static_transformStamped.transform.rotation.y = 1.0
            static_transformStamped.transform.rotation.z = 1.0
            static_transformStamped.transform.rotation.w = 1.0


            broadcaster.sendTransform(static_transformStamped)


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