
#common lib
import os
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import numpy
from typing import Dict
from multiprocessing import Process
import subprocess
import signal

#ros2 lib
import rclpy
from rclpy.node import Node
from service.srv import RegisterDog,GetDogList,UnregisterDog
from service.msg import DogStatus

def startController(port,rosDomainId):
    os.environ['ROS_DOMAIN_ID'] = str(rosDomainId)
    os.system(f"ros2 launch basic RobotDogController.launch.py port:={port}")

class RobotDogConnector(Node):
    #save the dog that registered
    dogList:dict = {}
    #save the port and rosDomainId will arrange each dog
    ports:list=[i for i in range(9091,9120)]
    rosDomainIds:list = [i for i in range(17,50)]

    def __init__(self,name='RobotDogConnector'):
        super().__init__(name)
        #register the service
        self.registerService = self.create_service(RegisterDog,'dog/reg',self.registerDog)
        self.getDogListService = self.create_service(GetDogList,'dog/list',self.getDogList)
        self.unregisterDogService = self.create_service(UnregisterDog,'dog/unreg',self.unregisterDog)

        #set timer to check the dog status
        self.create_timer(5,self.checkDogStatus)

    def checkDogStatus(self):
        #check the dog status
        for key in list(self.dogList.keys()):
            item=self.dogList[key]
            # self._logger.error(str(item["life"]))
            if(item["life"]<0):
                self._logger.error(f"dog {key} is not response")
                self.unregisterDog(UnregisterDog.Request(dog_id=key),UnregisterDog.Response())
            else:
                item["life"] -=5
    
    def registerDog(self,request:RegisterDog.Request, response:RegisterDog.Response):
        if(request.dog_id in self.dogList or request.dog_id==""):
            self._logger.error("regsterDog: dog_id is already registered or empty")
            response.id = -1
            return response
        #get the port and rosDomainId
        port=self.ports.pop(0)
        rosDomainId=self.rosDomainIds.pop(0)
        #add the port and rosDomainId to the list
        self.dogList[request.dog_id] = {"port":port,"rosDomainId":rosDomainId}
        response.id = rosDomainId
        
        self._logger.info(f"register dog {request.dog_id} with port {port} and rosDomainId {rosDomainId}")

        #start the controller and rosbrige with port and rosDomainId with multiprocessing 
        #it can't kill node with terminate
        # p = Process(target=startController,args=(port,rosDomainId))
        # p.start()
        # self.dogList[request.dog_id]["process"] = p

        #start the controller and rosbrige with port and rosDomainId with subprocess
        sp_env=os.environ.copy()
        sp_env['ROS_DOMAIN_ID'] = str(rosDomainId)
        sp = subprocess.Popen(["ros2","launch","basic","RobotDogController.launch.py",f"port:={port}"],env=sp_env)
        self.dogList[request.dog_id]["process"] = sp
        #---------------------------------------------------

        #create the dog/status subscriber
        self.dogList[request.dog_id]["life"] = 10
        self.dogList[request.dog_id]["battery"] = 100
        def statusCallback(msg):
            # self._logger.error(f"get status from {request.dog_id}")
            self.dogList[request.dog_id]["life"] +=5
            self.dogList[request.dog_id]["battery"] = msg.battery
            if(msg.status==-1):
                self.dogList[request.dog_id]["life"] = -1
        self.dogList[request.dog_id]["getDogStatus"] = self.create_subscription(DogStatus,f'{request.dog_id}/dog/status',statusCallback,10)
        

    
        return response
    def unregisterDog(self,request:UnregisterDog.Request, response:UnregisterDog.Response):
        #check if the dog_id is registered
        if(request.dog_id not in self.dogList):
            self._logger.error("unregisterDog: dog_id is not registered")
            response.result = False
            return response
        #get the port and rosDomainId
        port = self.dogList[request.dog_id]["port"]
        rosDomainId = self.dogList[request.dog_id]["rosDomainId"]

        # unregister the dog with dog_id kill the rosbridge 
        #it can't kill node with terminate
        # p:Process=self.dogList[request.dog_id]["process"]
        # p.terminate()

        #unregister the dog with dog_id kill the rosbridge with subprocess
        sp:subprocess.Popen=self.dogList[request.dog_id]["process"]
        sp.send_signal(signal.SIGINT)
        #---------------------------------------------------

        #remove the dog/status subscriber
        self.destroy_subscription(self.dogList[request.dog_id]["getDogStatus"])
        

        #add the port and rosDomainId back to the list
        self.ports.append(port)
        self.rosDomainIds.append(rosDomainId)
        # self.dogList.pop(request.dog_id)
        del self.dogList[request.dog_id]

        #response
        response.result = True
        self._logger.info(f"unregister dog {request.dog_id} with port {port} and rosDomainId {rosDomainId}")
        return response
        

        

    def getDogList(self,request:GetDogList.Request, response: GetDogList.Response):

        self._logger.info("get dog list")
        #get the dog_id and port
        for key,item in self.dogList.items():
            response.dog_ids.append(key)
            response.ports.append(item["port"])
            response.batterys.append(item["battery"])
        return response
    

def main(args=None):
    rclpy.init(args=args)
    robotDogConnector = RobotDogConnector()
    print('init robotDogConnector node')
    rclpy.spin(robotDogConnector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()