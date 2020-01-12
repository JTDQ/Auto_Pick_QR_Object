#!/usr/bin/env python
# -*- coding: UTF-8 -*-
""" 根据二维码位置，计算出目标距离 """

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

class getPoseOfTheObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],
                             output_keys=['output_object_pose'])

        self.namespace = rospy.get_param("~robot_name")
        self.marker_pose_sub = rospy.Subscriber(self.namespace + '/ar_pose_marker', AlvarMarkers, self.arMarkerMsgCallback)
        self.ball_pose=rospy.Subscriber("/ball_pose",Vector3,self.ball_pose_callBack)
        self.goal_height = rospy.get_param("~goal_height")
        self.ar_marker_pose = False
        self.canot_find_times = 0
        self.ball_pos=None

    def arMarkerMsgCallback(self, ar_marker_pose_msg):
        if len(ar_marker_pose_msg.markers) == 0:
            if self.ar_marker_pose != False and self.canot_find_times < 3:
                self.canot_find_times += 1
            else:
                self.ar_marker_pose = False
                rospy.loginfo("CANNOT FIND AR POSE")
        else:
            self.ar_marker_pose = AlvarMarker()
            self.ar_marker_pose = ar_marker_pose_msg.markers[0]
            # rospy.loginfo("FIND AR POSE")
    def ball_pose_callBack(self,data):
        self.ball_pos=data
    def execute(self, userdata):
        if self.ar_marker_pose == False:
            rospy.logwarn('Failed to get pose of the marker')
            return 'aborted'
        else:
            object_pose = Pose()
            object_pose.position.x =self.ar_marker_pose.pose.pose.position.x+ 0.0
            object_pose.position.y  =self.ar_marker_pose.pose.pose.position.y +0.0
            object_pose.position.z = self.goal_height
            rospy.loginfo(object_pose.position)
            dist = math.sqrt((object_pose.position.x * object_pose.position.x) +
                        (object_pose.position.y * object_pose.position.y))

            if object_pose.position.y > 0:
                yaw = math.acos(object_pose.position.x / dist)
            else:
                yaw = (-1) * math.acos(object_pose.position.x / dist)

            roll = 0.0
            pitch = 0.0

            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)

            object_pose.orientation.w = cy * cr * cp + sy * sr * sp
            object_pose.orientation.x = cy * sr * cp - sy * cr * sp
            object_pose.orientation.y = cy * cr * sp + sy * sr * cp
            object_pose.orientation.z = sy * cr * cp - cy * sr * sp

            userdata.output_object_pose = object_pose
            rospy.loginfo('Succeeded to get pose of the object')
            return 'succeeded'
