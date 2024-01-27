

#common lib
import os
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import numpy

#ros2 lib
import rclpy
from rclpy.node import Node

class Camera(Node):
    def __init__(self,name='Camera'):
        super().__init__(name)
        raise NotImplementedError
    