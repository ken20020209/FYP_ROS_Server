
#common lib
import json
import os
import sys
import time
import cv2 as cv
import requests
from cv_bridge import CvBridge
import numpy
from typing import Dict
from multiprocessing import Process
import subprocess
import signal

#ros2 lib
import rclpy
from rclpy.node import Node
from message.srv import RegisterDog,GetDogList,UnregisterDog
from message.msg import DogStatus,DogList
from std_msgs.msg import Int32

def startController(port,rosDomainId):
    os.environ['ROS_DOMAIN_ID'] = str(rosDomainId)
    os.system(f"ros2 launch basic Controller.launch.py port:={port}")

class RobotDogConnector(Node):
    #save the dog that registered
    dogList:dict = {}
    # api_database_robotList:dict = None
    #save the port and rosDomainId will arrange each dog
    ports:list=[i for i in range(9091,9120)]
    rosDomainIds:list = [i for i in range(17,36)]

    def __init__(self,name='RobotDogConnector'):
        super().__init__(name)

        #declare the parameter
        self.declare_parameter('discoverServer','127.0.0.1')

        #register the service
        self.registerService = self.create_service(RegisterDog,'dog/reg',self.registerDog)
        self.getDogListService = self.create_service(GetDogList,'dog/list',self.getDogList)
        self.pubDogListTopic = self.create_publisher(DogList,'dog/list',10)
        self.unregisterDogService = self.create_service(UnregisterDog,'dog/unreg',self.unregisterDog)

        self.statuspub = self.create_publisher(Int32,'server/status',10)

        self.status=1
        #set timer to check the dog status
        self.create_timer(1,self.checkDogStatus)
        self.create_timer(2,self.pubDogList_timer)

        #fetch the robot list from the api database
        self.apiDatabseUrl=os.getenv('API_DATABASE_URL','http://localhost:5000')
        # if self.fetchFormApiDatabase():
        #     self._logger.info("fetch robot list from api database success")
        # else:
        #     self._logger.error("fetch robot list from api database fail")
        self._logger.info("init robotDogConnector node")

    def fetchFormApiDatabase(self,name)->int:
        def getToken():
            payload = json.dumps({
                "username": os.getenv('API_DATABASE_USERNAME','admin'),
                "password": os.getenv('API_DATABASE_PASSWORD','12345678')
                })
            headers = {
            'Content-Type': 'application/json'
            }
            try:
                response = requests.request("POST", self.apiDatabseUrl+'/api/auth/login', headers=headers, data=payload)
                data=response.json().get('data')
                if(data is None):
                    return None
                token = data.get('token')
                if(token):
                    return "Bearer "+token
                else:
                    return None
            except Exception as e:
                self._logger.error(str(e))
                # self._logger.error("fetch token from api database fail")
                return None

        self.token=getToken()
        if(self.token is None):
            self.get_logger().error("fetch token from api database fail")
            return 0
        os.environ['API_DATABASE_TOKEN'] = self.token
        def getRobot():
            headers = {
                'Authorization': self.token,
            }
            try:
                response = requests.request("GET", self.apiDatabseUrl+f'/api/robot?name={name}', headers=headers)
            except:
                return None
            if(response.status_code!=200):
                return None
            data=response.json().get('data')
            if(data is None):
                return None
            return data
        
        robot=getRobot()
        if(robot is None ):
            self.get_logger().error(f"fetch robot {name} from api database fail")
            return 0
        # self._logger.info(str(robot))
        self.get_logger().info(f"fetch robot {name} from api database success")
        self.get_logger().info(f"robot id is {robot.get('id')}")

        return robot.get('id')

        

    def pubDogList_timer(self):
        msg = DogList()
        for key,item in self.dogList.items():
            msg.dog_ids.append(key)
            msg.ports.append(item["port"])
            msg.batterys.append(item["battery"])
            msg.domain_ids.append(item["rosDomainId"])
            msg.types.append(item["type"])
        self.pubDogListTopic.publish(msg)
        
    def checkDogStatus(self):
        #check the dog status
        for key in list(self.dogList.keys()):
            item=self.dogList[key]
            # self._logger.error(str(item["life"]))
            if(item["life"]<0):
                self._logger.error(f"dog {key} is not response")
                self.unregisterDog(UnregisterDog.Request(dog_id=key),UnregisterDog.Response())
            else:
                item["life"] -=1
        
        #publish the server status
        msg = Int32()
        msg.data = self.status
        self.statuspub.publish(msg)

    
    def registerDog(self,request:RegisterDog.Request, response:RegisterDog.Response):
        if(request.dog_id[0]=="/"):
            request.dog_id=request.dog_id[1:]
        
        # if(self.api_database_robotList and request.dog_id not in self.api_database_robotList):
        #     self._logger.error(f"register dog fail: {request.dog_id} is not in the api database")
        #     response.id = -1
        #     return response
        if(request.dog_id in self.dogList or request.dog_id==""):
            self._logger.error("regsterDog: dog_id is already registered or empty")
            # response.id = -1

            #new vesrion that if same dog_id is repeated, give back the port and rosDomainId
            #BUG if the dog_id is repeated. The one controller server will have two physical dog
            self._logger.info(f"give back the port and rosDomainId")
            response.id = self.dogList[request.dog_id]["rosDomainId"]

            
            return response
        #get the port and rosDomainId
        port=self.ports.pop(0)
        rosDomainId=self.rosDomainIds.pop(0)
        #add the port and rosDomainId to the list
        self.dogList[request.dog_id] = {"port":port,"rosDomainId":rosDomainId,"type":request.type}
        response.id = rosDomainId
        
        self._logger.info(f"register dog {request.dog_id} with port {port} and rosDomainId {rosDomainId} type {request.type}")

        #start the controller and rosbrige with port and rosDomainId with multiprocessing 
        #it can't kill node with terminate
        # p = Process(target=startController,args=(port,rosDomainId))
        # p.start()
        # self.dogList[request.dog_id]["process"] = p

        #start the controller and rosbrige with port and rosDomainId with subprocess
        sp_env=os.environ.copy()

        #set up the environment variable

        # get the robot id from the api database
        sp_env['ROBOT_ID'] = str(self.fetchFormApiDatabase(request.dog_id))

        sp_env['ROBOT_NAME'] = str(request.dog_id)
        
        sp_env['ROS_DOMAIN_ID'] = str(rosDomainId)
        if(self.get_parameter('discoverServer').get_parameter_value().string_value!="127.0.0.1"):
            # sp_env['DISCOVERY_SERVER_PORT'] = f"{11811+rosDomainId}"
            sp_env['FASTRTPS_DEFAULT_PROFILES_FILE']=sp_env['FASTRTPS_DEFAULT_PROFILES_FILE'][:-4]+f"{rosDomainId}.xml"
            self.dogList[request.dog_id]["fastdds_discovery"] = Process(target=lambda: os.system(f"fastdds discovery -i 0 -p {11811+rosDomainId}"))
            self.dogList[request.dog_id]["fastdds_discovery"].start()
        # sp = subprocess.Popen(["ros2","launch","basic","Controller.launch.py",f"port:={port}",f"namespace:={request.dog_id}"],env=sp_env)
        sp = subprocess.Popen(["ros2","launch","basic","Controller.launch.py",f"port:={port}"],env=sp_env)
        self.dogList[request.dog_id]["process"] = sp
        #---------------------------------------------------

        #create the dog/status subscriber
        self.dogList[request.dog_id]["life"] = 10
        self.dogList[request.dog_id]["battery"] = 100
        def statusCallback(msg):
            # self._logger.error(f"get status from {request.dog_id}")
            self.dogList[request.dog_id]["life"] =10
            self.dogList[request.dog_id]["battery"] = msg.battery
            if(msg.status==-1):
                self.dogList[request.dog_id]["life"] = -1
        self.dogList[request.dog_id]["getDogStatus"] = self.create_subscription(DogStatus,f'{request.dog_id}/dog/status',statusCallback,10)
        
        self._logger.info(f"register dog success")

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
        if(self.get_parameter('discoverServer').get_parameter_value().string_value!="127.0.0.1"):
            self.dogList[request.dog_id]["fastdds_discovery"].terminate()
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
            response.domain_ids.append(item["rosDomainId"])
            response.types.append(item["type"])
        return response
    def destroy_node(self) -> numpy.bool:
        self.status=-1
        self.checkDogStatus()
        return super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    robotDogConnector = RobotDogConnector()
    print('init robotDogConnector node')
    rclpy.spin(robotDogConnector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
