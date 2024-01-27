

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
from sensor_msgs.msg import Image, CompressedImage


class Camera(Node):

    #dummy processer
    class DummyProcesser:
        def __init__(self):
            pass
        def process(self,img):
            return img
    #save procesor on setting
    setting:dict = {}
    processor:any = DummyProcesser()

    def __init__(self,name='Camera'):
        super().__init__(name)
        self.subCamera = self.create_subscription(Image, 'camera/raw', self.subCammera, 10)
        self.pubCamera = self.create_publisher(Image, 'camera/processed', 10)
        self.bridge = CvBridge()
        self.img = None
        self.img_effected = None
    
    def subCammera(self,msg):
        # get image from dog camera and process it
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img_effected = self.processor.process(self.img)
        msg_effected = self.bridge.cv2_to_imgmsg(self.img_effected, "bgr8")
        self.pubCamera.publish(msg_effected)
        
    def process(self,):
        raise NotImplementedError
    