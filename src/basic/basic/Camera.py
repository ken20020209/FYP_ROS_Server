

#common lib
import os
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import requests
import numpy as np
import base64
import json

#ros2 lib
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32


class Camera(Node):

    #save procesor on setting
    effect:dict = {0:"none",1:"detect",2:"classify",3:"pose",4:"segment"}
    cursetting=0

    def __init__(self,name='Camera'):
        super().__init__(name)
        # self.subCamera = self.create_subscription(Image, 'camera/raw', self.subCammera, 10)
        self.subCamera = self.create_subscription(CompressedImage, 'camera/raw', self.subCammera, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pubCamera = self.create_publisher(CompressedImage, 'camera/processed',  QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pubCameraRaw = self.create_publisher(Image, 'camera/processed/raw',  QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subCameraSetting = self.create_subscription(Int32, 'camera/setting', self.subSetting, 10)
        self.pubCameraCurrentSetting = self.create_publisher(Int32, 'camera/setting/current',  10)

        self.bridge = CvBridge()
        self.img = None
        self.img_effected = None

        self.timer = self.create_timer(1, self.timer_callback)

        self.declare_parameter('url','http://localhost:80/effect')
        self.url = self.get_parameter('url').value

        self.url= os.getenv('CAMERA_URL',self.url)

        print("Camera init success")

    def timer_callback(self):
        self.pubCameraCurrentSetting.publish(Int32(data=self.cursetting))
    def subSetting(self,msg):
        if(msg.data>4 or msg.data<0):
            return
        # get setting from setting camera and process it
        self.cursetting = msg.data


    def subCammera(self,msg):
        # get image from dog camera and process it
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.img_effected = self.process(self.img)
        msg_effected = self.bridge.cv2_to_compressed_imgmsg(self.img_effected)
        self.pubCamera.publish(msg_effected)

        # for rviz2 display
        msg_msg_effected= self.bridge.cv2_to_imgmsg(self.img_effected)
        self.pubCameraRaw.publish(msg_msg_effected)



    def process(self,frame):
        def cv2_to_base64(image):
            _, buffer = cv.imencode('.jpg', image)
            image_bytes = buffer.tobytes()
            base64_bytes = base64.b64encode(image_bytes)
            base64_string = base64_bytes.decode('utf-8')
            return base64_string

        def base64_to_cv2(base64_string):
            base64_bytes = base64_string.encode('utf-8')
            image_bytes = base64.b64decode(base64_bytes)
            buffer = np.frombuffer(image_bytes, dtype=np.uint8)
            image = cv.imdecode(buffer, cv.IMREAD_COLOR)
            return image
        if(self.cursetting==0 or self.cursetting>4):
            return frame
        headers = {
        'Content-Type': 'application/json'
        }
        frame = cv.resize(frame, (640, 640))

        image=cv2_to_base64(frame)
        # print image size
        # print(len(image)/1024,"KB")
        payload = json.dumps({
            "image": image,
            "effect": self.effect[self.cursetting]
        })
        #todo missing error handling
        response = requests.request("POST", self.url, headers=headers, data=payload)
        response = json.loads(response.text)
        image = base64_to_cv2(response["image"])
        return image
    
    
def main():
    rclpy.init()
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
