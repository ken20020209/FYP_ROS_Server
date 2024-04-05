import os
from time import sleep
from threading import Thread
from multiprocessing import Process,Manager,Value

import subprocess
import socket
import json


# if(os.path.exists(f"{os.path.dirname(os.path.abspath(__file__))}/install/setup.bash")):
#     with open(f"{os.path.dirname(os.path.abspath(__file__))}/env.json") as f:
#         env=json.load(f)
#         for k,v in env.items():
#             k=k.upper()
#             os.environ[k]=v

discovery_server =os.getenv("discover_server" ,"discoveryserver.ddns.net")


def get_ip_address(domain_name):
    try:
        return socket.gethostbyname(domain_name)
    except:
        return "127.0.0.1"
    
discovery_server_ip=get_ip_address(discovery_server)
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