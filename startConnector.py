import os
from time import sleep
from threading import Thread
from multiprocessing import Process,Manager,Value
import requests

import subprocess
import socket
import json


def gethostIP():
    try:
        url = os.getenv('ROS_PROXY_IP_URL','http://localhost:9088/api/ros/ip')
        payload = {}
        headers = {}
        response = requests.request("GET", url, headers=headers, data=payload)
        if(response.status_code!=200):
            return ''
        data=response.json()
        ip=data.get('ip')
        if(ip is None):
            return ''
        return ip
    except Exception as e:
        print(str(e))
        return ''
def putHostIP(ip) -> bool:
    try:
        import requests
        url = os.getenv('ROS_PROXY_URL','http://localhost:9089') +'/api/ros/ip'
        payload = json.dumps({
            "ip":ip
        })
        headers = {
        'Content-Type': 'application/json'
        }
        response = requests.request("POST", url, headers=headers, data=payload)
        if(response.status_code!=200):
            return False
        return True
    except Exception as e:
        print(str(e))
        return False

# if(os.path.exists(f"{os.path.dirname(os.path.abspath(__file__))}/install/setup.bash")):
#     with open(f"{os.path.dirname(os.path.abspath(__file__))}/env.json") as f:
#         env=json.load(f)
#         for k,v in env.items():
#             k=k.upper()
#             os.environ[k]=v

discovery_server =os.getenv("discover_server" ,"discovery.ken20020209.com")

count=0

def get_ip_address(domain_name):
    try:
        return socket.gethostbyname(domain_name)
    except:
        return "127.0.0.1"
    
discovery_server_ip=get_ip_address(discovery_server)

while count<10 and discovery_server_ip!="127.0.0.1":
    print(f"get host ip from {discovery_server_ip}")
    ip=gethostIP()
    if(ip==""):
        sleep(5)
        continue
    discovery_server_ip=ip
    count+=1
    break
if(count>=10):
    exit(1)
count=0
while count<10 and discovery_server_ip!="127.0.0.1":
    print(f"put host ip {discovery_server_ip}")
    if(putHostIP(discovery_server_ip)):
        sleep(1)
        break
    count+=1
if(count>=10):
    exit(1)

print(f"{discovery_server}:{get_ip_address(discovery_server)}")
cmd=f"/bin/bash -c"
cmd+=f" '"
cmd+=f"source {os.path.dirname(os.path.abspath(__file__))}/install/setup.bash"
if(discovery_server_ip!="127.0.0.1"):
    print(f"start with discovery server {discovery_server_ip}")
    Process(target=lambda: os.system(f"fastdds discovery -i 0 -p {11811} > /dev/null")).start()


    # for i in range(17,36):
    #     Process(target=lambda: os.system(f"fastdds discovery -i 0 -p {11811+i} > /dev/null")).start()

    # cmd+=f' && fastdds discovery -i 0'
    cmd+=f' && export FASTRTPS_DEFAULT_PROFILES_FILE={os.path.dirname(os.path.abspath(__file__))}/fastdds/super_client_configuration_file.xml' 
    # cmd+=f" && export ROS_SUPER_CLIENT=TRUE"
    # cmd+=f' && export DISCOVERY_SERVER_PORT=11811'
    # cmd+=f' && export ROS_DISCOVERY_SERVER={discovery_server_ip}:11811'
cmd+=f"&& ros2 launch basic Connector.launch.py discoverServer:={discovery_server_ip}"
cmd+=f"'"
os.system(cmd)