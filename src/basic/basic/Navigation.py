
#common lib
import os
import sys
import time
import cv2 as cv
from cv_bridge import CvBridge
import numpy
from typing import List

#ros2 lib
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped,Pose
from std_msgs.msg import Bool,String


from message.srv import MoveToPoint,MoveToPoints,PatrolPoints,StopNavigation,FollowObject

from .lib.robot_navigator import BasicNavigator

class Navigation(Node):
    def __init__(self,name='Navigation'):
        super().__init__('name')

        self.isTaskComplete=True
        self.feedback = "idle"

        self.navigator = BasicNavigator()
        self.navigator.lifecycleStartup()

        #create topic publisher
        self.isTaskComplete_publisher = self.create_publisher(Bool,'nav2/is_task_complete',10)
        self.feedback_publisher = self.create_publisher(String,'nav2/feedback',10)

        # create service
        self.moveToPoint_client = self.create_service(MoveToPoint,'nav2/move_to_point',self.moveToPoint)
        self.moveToPoints_client = self.create_service(MoveToPoints,'nav2/move_to_points',self.moveToPoints)
        self.patrolPoints_client = self.create_service(PatrolPoints,'nav2/patrol_points',self.patrolPoints)
        self.stopNavigation_client = self.create_service(StopNavigation,'nav2/stop_navigation',self.stopNavigation)
        self.followObject_client = self.create_service(FollowObject,'nav2/follow_object',self.followObject)



        self.create_timer(1,self.update)
    
    def update(self):
        # print(self.navigator.getFeedback)
        self.isTaskComplete = self.navigator.isTaskComplete()
        if(self.isTaskComplete):
            self.feedback="idle"

        self.isTaskComplete_publisher.publish(Bool(data=self.isTaskComplete))
        self.feedback_publisher.publish(String(data=self.feedback))

        #debug message
        # print(self.navigator.isTaskComplete())
        pass

    def _createPoseStamped(self,pose:Pose):
        goal_pose = PoseStamped()
        goal_pose.pose=pose
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        return goal_pose

    def moveToPoint(self,request:MoveToPoint.Request,response:MoveToPoint.Response):
        self.navigator.cancelTask()
        goal_pose = self._createPoseStamped(request.pose)
        response.result='failed'
        if self.navigator.goToPose(goal_pose):
            response.result='success'
            self.feedback="moving to point"
        return response
    
    def moveToPoints(self,request:MoveToPoints.Request,response:MoveToPoints.Response):
        self.navigator.cancelTask()
        goal_poses = []
        for pose in request.poses:
            goal_poses.append(self._createPoseStamped(pose))
        
        response.result='failed'
        if self.navigator.followWaypoints(goal_poses):
            response.result='success'
            self.feedback="moving to points"
        return response
    
    def patrolPoints(self,request:PatrolPoints.Request,response:PatrolPoints.Response):
        self.navigator.cancelTask()
        goal_poses = []
        for pose in request.poses:
            goal_poses.append(self._createPoseStamped(pose))
        response.result='failed'
        if self.navigator.patrolWaypoints(goal_poses):
            response.result='success'
            self.feedback="patrolling points"
        return response
    
    def stopNavigation(self,request:PatrolPoints.Request,response:PatrolPoints.Response):
        self.navigator.patrol_callback=False
        self.navigator.cancelTask()
        response.result='success'
        return response
    
    # TODO: Follow Object Service 
    # no idea how to implement this
    def followObject(self,request:FollowObject.Request,response:FollowObject.Response):
        raise NotImplementedError

def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()