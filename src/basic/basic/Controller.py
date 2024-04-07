
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
import requests

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
        self.env= os.environ.copy()
        self.env["DISPLAY"] = ":"+self.env["ROS_DOMAIN_ID"]

        #start Xvnc and novnc
        offset = int(self.env["ROS_DOMAIN_ID"])
        Xvnc_port = 5900 + offset
        novnc_port = 6080 + offset
        
        #remember to install the Xvnc and novnc
        self._logger.info("start Xvnc and novnc")
        self.Xvnc_sp = subprocess.Popen(["Xvnc",f":{offset}","-geometry","1920x1080","-depth","24","-SecurityTypes","none"],env=self.env)
        self.novnc_sp = subprocess.Popen(["/usr/share/novnc/utils/launch.sh","--listen",str(novnc_port),"--vnc","localhost:"+str(Xvnc_port)],env=self.env)



        self.camera_sp = None
        self.navigation_sp = None
        self.slam_sp = None
        self.navigation_switch_service = self.create_service(SwitchService,'controller/navigation_switch',self.navigation_switch)
        self.slam_switch_service= self.create_service(SwitchService,'controller/slam_switch',self.slam_switch)

        self.navigation_pub = self.create_publisher(Bool,'controller/navigation',10)
        self.slam_pub = self.create_publisher(Bool,'controller/slam',10)

        self.timer= self.create_timer(1,self.timer_callback)

        default_map_storage_path = os.path.join(os.path.expanduser('~'),'data/temp/map')
        self.map_storage_path = os.getenv('MAP_STORAGE_PATH',default_map_storage_path)
        if not os.path.exists(self.map_storage_path):
            os.makedirs(self.map_storage_path)
            self.get_logger().info(f"Create Map Storage Path: {self.map_storage_path}")
        self.apiDatabseUrl=os.getenv('API_DATABASE_URL','http://localhost:5000')
        self.apiDatabseUrlMap = f"{self.apiDatabseUrl}/data/map"

    def timer_callback(self):

        navigation_msg = Bool()
        navigation_msg.data = self.navigation
        self.navigation_pub.publish(navigation_msg)

        slam_msg = Bool()
        slam_msg.data = self.slam
        self.slam_pub.publish(slam_msg)

    def navigation_switch(self,request,response):
        response.result = "fail to switch the navigation"
        if self.slam == True:
            response.result = "cant start the navigation while the slam on"
            return response
        if request.switch_service == self.navigation:
            response.result = "the navigation is already in the state"
            return response
        if request.switch_service == True:
            if(self.getMap(request.map) == False):
                response.result = "failed to get the map"
                return response

            self.navigation = True
            response.result = "the navigation started"

            mapPath=os.path.join(self.map_storage_path,request.map)
            mapParam=f'map:={mapPath}'
            if(self.get_namespace != ""):
                self.navigation_sp = subprocess.Popen(["ros2","launch","basic","Navigation_slam_toolbox.launch.py",f"namespace:={self.get_namespace()}",mapParam],env=self.env)
            else:
                self.navigation_sp = subprocess.Popen(["ros2","launch","basic","Navigation_slam_toolbox.launch.py",mapParam],env=self.env)
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
        if request.switch_service == self.slam:
            response.result = "the slam is already in the state"
            return response
        if request.switch_service == True:
            self.slam = True
            response.result = "the slam started"
            if(self.get_namespace != ""):
                self.slam_sp = subprocess.Popen(["ros2","launch","basic","Mapping_slam_toolbox.launch.py",f"namespace:={self.get_namespace()}"],env=self.env)
            else:
                self.slam_sp = subprocess.Popen(["ros2","launch","basic","Mapping_slam_toolbox.launch.py"],env=self.env)
        else:
            self.slam = False
            response.result = "the slam closed"
            self.slam_sp.send_signal(signal.SIGINT)
            self.slam_sp = None
        return response
    def getMap(self,name):
        def download_file(url, folder_name):
            local_filename = url.split("/")[-1]
            path = os.path.join(folder_name, local_filename)

            with requests.get(url, stream=True) as r:
                if(r.status_code != 200):
                    raise Exception(f"Failed to download file: {url}")
                with open(path, "wb") as f:
                    for chunk in r.iter_content(chunk_size=1024 * 8):
                        if chunk:
                            f.write(chunk)
                            f.flush()
                            os.fsync(f.fileno())

            return local_filename
        # download map from database
        url=self.apiDatabseUrlMap+"/"+name
        data_url=url+".data"
        posegraph_url=url+".posegraph"
        try:
            download_file(data_url,self.map_storage_path)
            download_file(posegraph_url,self.map_storage_path)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to download map: {e}")
            return False
        
    
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()