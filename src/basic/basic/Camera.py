

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
import datetime

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

        self.robot_id=int(os.getenv('ROBOT_ID',0))
        
        default_record_storage_path = os.path.join(os.path.expanduser('~'),'data/record')
        self.record_storage_path = os.getenv('RECORD_STORAGE_PATH',default_record_storage_path)
        self.record_storage_path=self.record_storage_path+f"/{self.robot_id}"

        if(os.path.exists(self.record_storage_path)==False):
            os.makedirs(self.record_storage_path)
            self.get_logger().info(f"Create Record Storage Path: {self.record_storage_path}")

        self.apiDatabseUrl=os.getenv('API_DATABASE_URL','http://localhost:5000')
        self.apiDatabseUrlRecord = f"{self.apiDatabseUrl}/api/record"

        self.token=os.getenv('API_DATABASE_TOKEN',None)
        if(self.token is None):
            self.token = self.getToken()

        self.recordHeight = 480
        self.recordWidth = 640

        self.recordFrameCount=0
        # frame*second 
        self.recordFrameRequired=25*30
       
        self.recordWrite = None

        self.waitUploadRecodeList=[]
        


        # self.update_writer_timer = self.create_timer(10, self.update_writer_callback)



        print("Camera init success")


    def timer_callback(self):
        self.pubCameraCurrentSetting.publish(Int32(data=self.cursetting))
    def update_writer_callback(self):
        if(self.recordWrite is None):
            self.recordWrite = self.getNewRecordWriter()
            return

        self.recordWrite.release()
        self.waitUploadRecodeList.append(self.recordName)

        self.recordWrite = self.getNewRecordWriter()

        # upload record to database
        for record in self.waitUploadRecodeList:
            if(self.uploadRecod(record)):
                self.waitUploadRecodeList.remove(record)
            else:
                break

    def getNewRecordWriter(self):
        # datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+
        self.recordName = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")+f"robot_id_{self.robot_id}"+"record.mp4"
        return cv.VideoWriter(os.path.join(self.record_storage_path,self.recordName),cv.VideoWriter_fourcc(*'avc1'), 25, (self.recordWidth, self.recordHeight))

    def subSetting(self,msg):
        if(msg.data>4 or msg.data<0):
            return
        # get setting from setting camera and process it
        self.cursetting = msg.data


    def subCammera(self,msg):
        # get image from dog camera and process it
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.img = cv.resize(self.img, (self.recordWidth, self.recordHeight))
        # save record to local storage
        self.writeRecord(self.img)
        # process image
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
        # frame = cv.resize(frame, (640, 640))

        image=cv2_to_base64(frame)
        # print image size
        # print(len(image)/1024,"KB")
        payload = json.dumps({
            "image": image,
            "effect": self.effect[self.cursetting]
        })
        #todo missing error handling
        try:
            response = requests.request("POST", self.url, headers=headers, data=payload)
        except:
            return frame

        response = json.loads(response.text)
        image = base64_to_cv2(response["image"])
        return image
    def writeRecord(self,frame):
        
        self.recordWidth = frame.shape[1]
        self.recordHeight = frame.shape[0]
      
        if(self.recordFrameCount==0):
            self.update_writer_callback()
        self.recordFrameCount+=1
        self.recordFrameCount%=self.recordFrameRequired
            

        # save record to local storage
        # print(frame.shape)
        self.recordWrite.write(frame)



    def getToken(self):
        payload = json.dumps({
            "username": os.getenv('API_DATABASE_USERNAME','admin'),
            "password": os.getenv('API_DATABASE_PASSWORD','12345678')
            })
        headers = {
        'Content-Type': 'application/json'
        }
        try:
            response = requests.request("POST", self.apiDatabseUrl+'/api/auth/login', headers=headers, data=payload)
        except:
            return None
        data=response.json().get('data')
        if(data is None):
            return None
        token = data.get('token')
        if(token):
            return "Bearer "+token
        else:
            return None
    def uploadRecod(self,name):

        record_path = os.path.join(self.record_storage_path,name)
        token = self.token
        if(token is None):
            return False
        headers = {
            'Authorization': token
        }
        payload = {
            'robot_id': self.robot_id,
            'name': name
        }
        files = [
            ('record', open(record_path, 'rb'))
        ]
        try:
            response = requests.request("POST", self.apiDatabseUrlRecord, headers=headers, data=payload,files=files)
        except:
            return False
        if(response.status_code==200):
            return True
        else:
            return False
            

def main():
    rclpy.init()
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
