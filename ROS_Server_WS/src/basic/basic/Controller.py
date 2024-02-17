
#common lib
import os
import signal
import subprocess
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import numpy
from typing import List

#ros2 lib
import rclpy
from rclpy.node import Node

from message.srv import SwitchService

from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Bool,String


class Controller(Node):
    def __init__(self,name='Controller'):
        super().__init__('name')

        self.camera = False
        self.navigation = False
        self.slam = False

        self.camera_sp = None
        self.navigation_sp = None
        self.slam_sp = None

        self.camera_switch_service = self.create_service(SwitchService,'controller/camera_switch',self.camera_switch)
        self.navigation_switch_service = self.create_service(SwitchService,'controller/navigation_switch',self.navigation)
        self.slam_switch_service= self.create_service(SwitchService,'controller/slam_switch',self.slam_switch)

        self.camera_pub = self.create_publisher(Bool,'controller/camera',10)
        self.navigation_pub = self.create_publisher(Bool,'controller/navigation',10)
        self.slam_pub = self.create_publisher(Bool,'controller/slam',10)

        self.timer= self.create_timer(1,self.timer_callback)

    def timer_callback(self):
        camera_msg = Bool()
        camera_msg.data = self.camera
        self.camera_pub.publish(camera_msg)

        navigation_msg = Bool()
        navigation_msg.data = self.navigation
        self.navigation_pub.publish(navigation_msg)

        slam_msg = Bool()
        slam_msg.data = self.slam
        self.slam_pub.publish(slam_msg)

    def camera_switch(self,request,response):
        response.result = "fail to switch the camera"
        if request.switch == self.camera:
            response.result = "the camera is already in the state"
            return response
        if request.switch == True:
            self.camera = True
            response.result = "the camera started"
            self.camera_sp = subprocess.Popen(["ros2","launch","basic","Camera.launch.py"],env=os.environ.copy())
        else:
            self.camera = False
            response.result = "the camera closed"
            self.camera_sp.send_signal(signal.SIGINT)
            self.camera_sp = None
            
        return response
    def navigation_switch(self,request,response):
        response.result = "fail to switch the navigation"
        if self.slam == True:
            response.result = "cant start the navigation while the slam on"
            return response
        if request.switch == self.navigation:
            response.result = "the navigation is already in the state"
            return response
        if request.switch == True:
            self.navigation = True
            response.result = "the navigation started"
            self.navigation_sp = subprocess.Popen(["ros2","launch","basic","Navigation.launch.py"],env=os.environ.copy())
        else:
            self.navigation = False
            response.result = "the navigation closed"
            self.navigation_sp.send_signal(signal.SIGINT)
            self.navigation_sp = None
        return response
    def slam_switch(self,request,response):
        response.result = "fail to switch the slam"
        if(self.navigation==True):
            response.result = "cant start the slam while the navigation on"
            return response
        if request.switch == self.slam:
            response.result = "the slam is already in the state"
            return response
        if request.switch == True:
            self.slam = True
            response.result = "the slam started"
            self.slam_sp = subprocess.Popen(["ros2","launch","basic","Slam.launch.py"],env=os.environ.copy())
        else:
            self.slam = False
            response.result = "the slam closed"
            self.slam_sp.send_signal(signal.SIGINT)
            self.slam_sp = None
        return response

        