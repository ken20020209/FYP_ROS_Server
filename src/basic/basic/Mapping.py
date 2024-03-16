
#common lib
import os
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import numpy
from typing import List

#ros2 lib
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Bool,String


from .lib.robot_navigator import BasicNavigator

class Mapping(Node):
    def __init__(self,name='Mapping'):
        super().__init__('name')


def main(args=None):
    rclpy.init(args=args)
    mapping = Mapping()
    rclpy.spin(mapping)
    mapping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()