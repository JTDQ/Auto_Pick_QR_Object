#!/usr/bin/env python
# -*- coding: UTF-8 -*-
""" 靠近目标点 """

import rospy
from math import pow, atan2, sqrt
from tf.transformations import *

import smach
import smach_ros
from smach_ros import SimpleActionState
from smach_ros import ServiceState

import threading
import time

# Navigation
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Vector3

# Manipulator
from geometry_msgs.msg import Pose
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import SetKinematicsPose

# AR Markers
from ar_track_alvar_msgs.msg import AlvarMarker
from ar_track_alvar_msgs.msg import AlvarMarkers

class getCloserToGoal(smach.State):
    def __init__(self,goodPosition):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.namespace = rospy.get_param("~robot_name")
        self.marker_pose_sub = rospy.Subscriber(self.namespace + '/ar_pose_marker', AlvarMarkers, self.arMarkerMsgCallback)
        self.odom_sub = rospy.Subscriber(self.namespace + '/odom', Odometry, self.odomMsgCallback)
        self.cmd_vel_pub = rospy.Publisher(self.namespace + '/cmd_vel', Twist, queue_size=10)
        self.tb3_odom = Odometry()
        self.cmd_vel = Twist()
        self.priv_dist = 0.0
        self.priv_heading = 0.0
        self.ar_marker_pose = False
        self.goodPosition=goodPosition

    def arMarkerMsgCallback(self, ar_marker_pose_msg):
        if len(ar_marker_pose_msg.markers) == 0:
            self.ar_marker_pose = False
        else:            
            self.ar_marker_pose = AlvarMarker()
            self.ar_marker_pose = ar_marker_pose_msg.markers[0]

    def odomMsgCallback(self, odom_msg):       
        self.tb3_odom = odom_msg

    def getDistanceFromRobot(self, goal):
        return goal.pose.pose.position.x
              
    def getAngleBtwRobotAndMarker(self, goal):
        return math.atan2(goal.pose.pose.position.y, goal.pose.pose.position.x)

    def execute(self, userdata):
        while 1:
            if self.ar_marker_pose == False:
                rospy.loginfo('Failed to get pose of the marker')

                self.cmd_vel.linear.x  = -0.04
                self.cmd_vel.angular.z = 0.0
            
                self.cmd_vel_pub.publish(self.cmd_vel)
                continue

            dist    = self.getDistanceFromRobot(self.ar_marker_pose)   # meter
            heading = self.getAngleBtwRobotAndMarker(self.ar_marker_pose)       # radian
            
            objective_function = (1.0 * abs(dist)) + (10.0 * abs(heading))        
            rospy.loginfo(("dist and angle",dist,heading,objective_function))
            # rospy.logwarn('dist: %f, heading: %f, obj_func_result: %f', dist, heading, objective_function)

            # dist tolerance: 0.170 meter, heading tolerance: +-0.09 rad (+-5.0 deg)
            if objective_function >= self.goodPosition:            
                self.cmd_vel.linear.x  = (0.2 * dist) + (0.02 * (dist - self.priv_dist))
                self.cmd_vel.linear.y  = 0.0 
                self.cmd_vel.linear.z  = 0.0

                self.cmd_vel.angular.x = 0.0  
                self.cmd_vel.angular.y = 0.0 
                self.cmd_vel.angular.z = (1.0 * heading) + (0.01 * (heading - self.priv_heading))
            
                self.cmd_vel_pub.publish(self.cmd_vel)
            else:
                self.cmd_vel.linear.x  = 0.0
                self.cmd_vel.angular.z = 0.0
            
                self.cmd_vel_pub.publish(self.cmd_vel)  

                return 'succeeded'

            self.priv_dist = dist
            self.priv_heading = heading



