#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='tt_ball_locator',
            executable='tt_ball_locator.py',
            name='tt_ball_locator'
        )
    ])
