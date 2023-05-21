#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.lifecycle import LifecycleNode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo

class BallLocatorNode(LifecycleNode):

    def __init__(self):
        super().__init__('tt_ball_locator')
        self.bridge = CvBridge()
        
    def on_configure(self):
        self.get_logger().info('Configured')

    def on_activate(self):
        self.sub = self.create_subscription(Image, 'head_front_camera/rgb/image_raw', self.callback, 10)
        self.sub2 = self.create_subscription(CameraInfo, '/head_front_camera/depth_registered/camera_info', self.camera_info_callback, 10)
        self.get_logger().info('Activated')

    def on_deactivate(self):
        self.get_logger().info('Deactivated')

    def on_cleanup(self):
        self.sub.destroy()
        self.get_logger().info('Cleaned up')

    def camera_info_callback(self, msg):
        fx = msg.K[0]  # Focal length in x-direction
        fy = msg.K[4]  # Focal length in y-direction

        print("Focal length (fx, fy):", fx, fy)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur to reduce noise
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        # Apply Canny edge detection
        edges = cv2.Canny(blurred_image, 50, 150)

        # Perform circle detection using the HoughCircles method
        circles = cv2.HoughCircles(
            edges,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=100,
            param1=50,
            param2=30,
            minRadius=10,
            maxRadius=100
        )


def main(args=None):
    rclpy.init(args=args)
    node = BallLocatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

