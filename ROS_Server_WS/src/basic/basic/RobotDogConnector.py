
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
from service.srv import RegisterDog

class RobotDogConnector(Node):
    dogList:dict = {}
    def __init__(self,name='RobotDogConnector'):
        super().__init__(name)
        self.registerService = self.create_service(RegisterDog,'/dog/reg',self.registerDog)
        # raise NotImplementedError
    
    def registerDog(self,request, response):
        # raise NotImplementedError
        response.id='123'
        return response
    def unregisterDog(self,request, response):
        raise NotImplementedError
    def getDogList(self,request, response):
        raise NotImplementedError

def main(args=None):
    rclpy.init(args=args)
    robotDogConnector = RobotDogConnector()
    rclpy.spin(robotDogConnector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()