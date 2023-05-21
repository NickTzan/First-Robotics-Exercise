#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
import message_filters
from sensor_msgs.msg import CameraInfo



class BallLocatorNode(Node):

    def __init__(self):
        self.bridge = CvBridge()   
        super().__init__('tt_ball_locator')     

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.get_logger().info("on_activate() is called.")
        # Subscribe to the RGB image topic
        rgb_image = message_filters.Subscriber(self, Image, '/head_front_camera/rgb/image_raw')
        # Subscribe to the depth image topic
        depth_image = message_filters.Subscriber(self, Image, '/head_front_camera/depth_registered/image_raw')
        # Subscribe to the camera info topic
        camera_info = message_filters.Subscriber(self, CameraInfo, '/head_front_camera/rgb/camera_info')

        variables = message_filters.ApproximateTimeSynchronizer([rgb_image, depth_image, camera_info], 10, 0.1, allow_headerless=True)
        variables.registerCallback(self.callback)
        return super().on_activate(state)
  
      
    def callback(self, rgb_image, depth_image, camera_info):
        if rgb_image is not None:
            # Convert the RGB image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding='passthrough')
            # Convert the image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # Apply Gaussian blur to reduce noise
            blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
            # Perform circle detection using the HoughCircles method
            circles = cv2.HoughCircles(blurred_image, cv2.HOUGH_GRADIENT, dp=1, minDist=100, param1=100, param2=10, minRadius=1, maxRadius=10)
            # Calculate the distance to the ball using depth information
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

            if circles is not None:
                circles = np.uint16(np.around(circles))
                
                best = None
                for circle in circles[0,:]:
                   if best is None or circle[2]>best[2]:
                      best = circle
               
                # self.get_logger().info(f"x: {best[0]}, y: {best[1]}, z: {cv_depth_image[best[1], best[0]]}, width:{camera_info.width}, k[0]: {camera_info.k[0]}, k[4]: {camera_info.k[4]}")

                # draw the circle of the detected ball
                cv2.circle(cv_image, (best[0], best[1]), best[2], (0, 255, 0), 2)
                # show the image with the circles
                # cv2.imshow("Image window", cv_image)

                z = cv_depth_image[best[1], best[0]]
                x = (best[0] - ((camera_info.width)/2)) * z / camera_info.k[0]
                y = (best[1] - ((camera_info.height)/2)) * z / camera_info.k[4]
           
                transformer = TransformStamped()
                transformer.header.stamp = self.get_clock().now().to_msg()
                transformer.header.frame_id = 'head_front_camera_rgb_optical_frame'
                transformer.child_frame_id = 'tt_ball_center_link'
                transformer.transform.translation.x = x
                transformer.transform.translation.y = y
                transformer.transform.translation.z = float(z)
                transformer.transform.rotation.x = 0.0
                transformer.transform.rotation.y = 0.0
                transformer.transform.rotation.z = 0.0
                transformer.transform.rotation.w = 1.0

                self.broadcaster.sendTransform(transformer)

                # close the image window
                # cv2.waitKey(1)

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