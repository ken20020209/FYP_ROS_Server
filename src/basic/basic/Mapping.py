
#common lib
import os
import glob
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

from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Bool,String

from message.srv import SaveMap
from slam_toolbox.srv import SerializePoseGraph,SaveMap as slamSaveMap

from .lib.robot_navigator import BasicNavigator

class Mapping(Node):
    def __init__(self,name='Mapping'):
        super().__init__('name')

        self.saveMapService = self.create_service(SaveMap,'/save_map',self.save_map_callback)

        default_map_storage_path = os.path.join(os.path.expanduser('~'),'data/map')
        self.map_storage_path = os.getenv('MAP_STORAGE_PATH',default_map_storage_path)

        self.apiDatabseUrl=os.getenv('API_DATABASE_URL','http://localhost:5000')
        self.apiDatabseUrl = f"{self.apiDatabseUrl}/api/map"

        if not os.path.exists(self.map_storage_path):
            os.makedirs(self.map_storage_path)
            self.get_logger().info(f"Create Map Storage Path: {self.map_storage_path}")


    def save_map_callback(self,request,response):
        
        self.saveSlamMapClient = self.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map',)
        self.get_logger().info(f"Save Map Request: {request.name}")

        map_path = os.path.join(self.map_storage_path,request.name)

        isExist = glob.glob(f"{map_path}.*")
        if(len(isExist)>0):
            # delete file
            for f in isExist:
                os.remove(f)

        req=SerializePoseGraph.Request()
        req.filename = map_path
        future = self.saveSlamMapClient.call_async(req)
        # rclpy.spin_until_future_complete(self,future)

        # if future.result() is not None:
        #     self.get_logger().info(f"Map Saved: {map_path}")
        #     response.status = True
        # else:
        #     self.get_logger().error(f"Failed to Save Map: {map_path}")
        #     response.status = False
        time.sleep(1)

        isExist = glob.glob(f"{map_path}.*")
        if len(isExist) > 0:
            response.status = True
        else:
            response.status = False
            response.result = f"Failed to Save Map slam"
            return response
        
        
        # call slam_toolbox/save_map service
        self.saveMapClient= self.create_client(slamSaveMap,'/slam_toolbox/save_map')

        req = slamSaveMap.Request()
        req.name.data = map_path
        future = self.saveMapClient.call_async(req)

        time.sleep(2)
        # print(f"Map Saved: {map_path}",future.result())

        isExist=os.path.exists(f"{map_path}.pgm")
        
        if(isExist):
            response.status = True
        else:
            response.status = False
            response.result = f"Failed to Save Map png"
            return  response
        
        # transform pgm to png
        pgm_path = f"{map_path}.pgm"
        png_path = f"{map_path}.png"
        img = cv.imread(pgm_path, cv.IMREAD_UNCHANGED)
        cv.imwrite(png_path, img)

        # upload to api-database server
        
        url = self.apiDatabseUrl

        map_png_path=f"{map_path}.png"
        map_posegraph_path=f"{map_path}.posegraph"
        map_posegraphData_path=f"{map_path}.data"


        payload = {'robot_id': '0',
        'name': request.name}
        files=[
        ('png',(map_png_path,open(map_png_path,'rb'),'image/png')),
        ('posegraph',(map_posegraph_path,open(map_posegraph_path,'rb'),'image/png')),
        ('posegraphData',(map_posegraphData_path,open(map_posegraphData_path,'rb'),'application/octet-stream'))
        ]
        headers = {}
        try:
            responseApi = requests.request("POST", url, headers=headers, data=payload, files=files)
        except Exception as e:
            self.get_logger().error(f"Failed to Upload Map to API-Database: {e}")
            response.status = False
            response.result = f"Failed to Upload Map to API-Database"
            return response

        if(responseApi.status_code==400):
            response.status = False
            response.result = responseApi.json().get('msg')
            return response

        response.result=f'Map Saved: {request.name}'

        return response
        


def main(args=None):
    rclpy.init(args=args)
    mapping = Mapping()
    rclpy.spin(mapping)
    mapping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()