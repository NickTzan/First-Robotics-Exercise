#!/usr/bin/env python3

import rclpy
import tf2_ros
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.time import Time
import threading
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from math import pi, cos, sin, atan2
from std_srvs.srv import Empty


class TableExplorer(Node):

    def __init__(self):
        super().__init__('tt_table_explorer')     


    def on_configure(self, state: State) -> TransitionCallbackReturn:

        self.get_logger().info("on_configure() is called.")

        return TransitionCallbackReturn.SUCCESS
    

    def on_activate(self, state: State) -> TransitionCallbackReturn:

        table_explorer_thread = threading.Thread(target=self.run)
        table_explorer_thread.start()

        return super().on_activate(state)
    

    def target_to_pose(self, target, goal):

        # Offsets to correct the robot's position in each target
        correction = {1: (0.1, 0.8), 
                      2: (0.1, -0.45), 
                      3: (0.8, -0.15), 
                      4: (-0.8, -0.15)}
        
        if goal == 1:

            pose = PoseStamped()
            pose.header = target.header
            pose.pose.position.x = target.transform.translation.x + correction[goal][0]
            pose.pose.position.y = target.transform.translation.y + correction[goal][1]
            pose.pose.position.z = target.transform.translation.z
            
            angle = atan2(target.transform.translation.y - pose.pose.position.y, target.transform.translation.x - pose.pose.position.x)
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sin(angle/2)
            pose.pose.orientation.w = cos(angle/2)
            
        elif goal == 2:

            pose = PoseStamped()
            pose.header = target.header
            pose.pose.position.x = target.transform.translation.x + correction[goal][0]
            pose.pose.position.y = target.transform.translation.y + correction[goal][1]
            pose.pose.position.z = target.transform.translation.z
            
            angle = atan2(target.transform.translation.y - pose.pose.position.y, target.transform.translation.x - pose.pose.position.x)
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sin(angle/2)
            pose.pose.orientation.w = cos(angle/2)
            
        elif goal == 3:

            pose = PoseStamped()
            pose.header = target.header
            pose.pose.position.x = target.transform.translation.x + correction[goal][0]
            pose.pose.position.y = target.transform.translation.y + correction[goal][1]
            pose.pose.position.z = target.transform.translation.z
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sin(2*pi/4)
            pose.pose.orientation.w = cos(2*pi/4)
            
        else:

            pose = PoseStamped()
            pose.header = target.header
            pose.pose.position.x = target.transform.translation.x + correction[goal][0]
            pose.pose.position.y = target.transform.translation.y + correction[goal][1]
            pose.pose.position.z = target.transform.translation.z
            
            angle = atan2(target.transform.translation.y - pose.pose.position.y, target.transform.translation.x - pose.pose.position.x)
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sin(angle/2)
            pose.pose.orientation.w = cos(angle/2)
    
        return pose

    
    def run(self):

        self.request = Empty.Request()      
        self.client = self.create_client(Empty, '/tt_umpire/assignment2/i_feel_confident')
        
        # Wait until the service is available
        while True:
            if self.client.wait_for_service(timeout_sec=1.0):
                break
            else:
                rclpy.spin_once(self)
                
        # Create a buffer and a listener
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        
        # Create a navigator and wait until it activates
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # Wait until the transform is available
        while (not self.buffer.can_transform("map", "tt_table_boundary_1_link", Time())):
            rclpy.spin_once(self, timeout_sec=1.0)
        
        target_names = ["tt_table_boundary_1_link", "tt_table_boundary_2_link", "tt_table_boundary_3_link", "tt_table_boundary_4_link"]
        target_poses = []

        for i, target_name in enumerate(target_names):
            target = self.buffer.lookup_transform("map", target_name, Time())
            pose = self.target_to_pose(target, i+1)
            target_poses.append(pose)

        # Move to each target pose, send a request and wait 1 second
        for pose in target_poses:
            self.navigate_to(pose)
            self.send_request()
            time.sleep(1)
        

    def navigate_to(self, pose):
        # Navigate to the target pose and wait until the task is complete
        self.navigator.goToPose(pose)
        while not self.navigator.isTaskComplete():
            time.sleep(1)


    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()



def main():
  rclpy.init()

  executor = rclpy.executors.SingleThreadedExecutor()
  lc_node = TableExplorer()
  executor.add_node(lc_node)
  try:
    executor.spin()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    lc_node.destroy_node()

if __name__ == '__main__':
  main()
