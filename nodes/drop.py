#!/usr/bin/env python
# -*- coding: UTF-8 -*-

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

from getObjectPose import getPoseOfTheObject
from getCloserGoal import getCloserToGoal
# 流程总述：
# 1、准备阶段：（机械臂归零，获取目标位置）
#   1.1、首先 init_pose->open_gripper->get_object_pose->
#   1.2、为了避免重复位置导致机械臂重启，这里设置了另一组初始位置。和1.1的对应 SET_Abort_INIT_POSITION->ABort_OPEN_GRIPPER->ABort_GET_POSE_OF_THE_OBJECT
# 2、伸手去抓阶段：ALIGN_ARM_WITH_OBJECT->CLOSE_TO_OBJECT->GRIP_OBJECT
# 3、抓住收回阶段：drop_UP_OBJECT -> SET_HOLDING_POSITION

def drop(drop_center):
    with drop_center:
        def joint_position_request_cb(userdata, request):
            joint = JointPosition()
            joint.position = userdata.input_position
            joint.max_velocity_scaling_factor = 1.0
            joint.max_accelerations_scaling_factor = 1.0
            request.planning_group = userdata.input_planning_group
            request.joint_position = joint
            return request

        def joint_position_response_cb(userdata, response):
            if response.is_planned == False:
                return 'aborted'
            else:
                rospy.sleep(3.)
                return 'succeeded'

        def eef_pose_request_cb(userdata, request):
            eef = KinematicsPose()
            eef.pose = userdata.input_pose
            rospy.loginfo('eef.position.x : %f', eef.pose.position.x)
            rospy.loginfo('eef.position.y : %f', eef.pose.position.y)
            rospy.loginfo('eef.position.z : %f', eef.pose.position.z)
            eef.max_velocity_scaling_factor = 1.0
            eef.max_accelerations_scaling_factor = 1.0
            eef.tolerance = userdata.input_tolerance

            request.planning_group = userdata.input_planning_group
            request.kinematics_pose = eef
            return request

        def align_arm_with_object_response_cb(userdata, response):
            if response.is_planned == False:
                drop_center.userdata.align_arm_with_object_tolerance += 0.005
                rospy.logwarn('Set more tolerance[%f]', drop_center.userdata.align_arm_with_object_tolerance)
                return 'aborted'
            else:
                OFFSET_FOR_STRETCH = 0.030
                drop_center.userdata.object_pose.position.x += OFFSET_FOR_STRETCH
                rospy.sleep(3.)
                return 'succeeded'

        def close_to_object_response_cb(userdata, response):
            if response.is_planned == False:
                drop_center.userdata.close_to_object_tolerance += 0.005
                rospy.logwarn('Set more tolerance[%f]', drop_center.userdata.close_to_object_tolerance)
                return 'aborted'
            else:
                OFFSET_FOR_OBJECT_HEIGHT = 0.020
                drop_center.userdata.object_pose.position.z += OFFSET_FOR_OBJECT_HEIGHT
                rospy.sleep(3.)
                return 'succeeded'

        def drop_up_object_response_cb(userdata, response):
            if response.is_planned == False:
                drop_center.userdata.drop_up_object_tolerance += 0.005
                rospy.logwarn('Set more tolerance[%f]', drop_center.userdata.drop_up_object_tolerance)
                return 'aborted'
            else:
                rospy.sleep(3.)
                return 'succeeded'

        def gripper_request_cb(userdata, request):
            joint = JointPosition()
            joint.position = userdata.input_gripper
            joint.max_velocity_scaling_factor = 1.0
            joint.max_accelerations_scaling_factor = 1.0

            request.planning_group = userdata.input_planning_group
            request.joint_position = joint
            return request

        def gripper_response_cb(userdata, response):
            rospy.sleep(1.)
            return 'succeeded'

        namespace=rospy.get_param("~robot_name")
        planning_group=rospy.get_param("~planning_group")
# 0、 这里定义了机械臂的几个关键位置节点
        drop_center.userdata.planning_group = planning_group

        drop_center.userdata.init_position = [0.0, -0.65, 1.20, -0.54]
        drop_center.userdata.abort_position = [0.0, -0.55, 1.10, -0.44]
        drop_center.userdata.holding_position = [0.0, 0, -0.8, 0.20]
        drop_center.userdata.drop_position = [1.57, 0.0, 0.00, 0.4]
        drop_center.userdata.release_position = [1.57, 0.0, -0.80, 0.4]
        drop_center.userdata.end_init_position = [0.0, -0.8, 0.5, -0.54]
        drop_center.userdata.object_pose = Pose()

        drop_center.userdata.close_gripper = [-0.005]
        drop_center.userdata.open_gripper = [0.005]


        drop_center.userdata.drop_up_object_tolerance = 0.01
        drop_center.userdata.close_to_object_tolerance = 0.01
        drop_center.userdata.align_arm_with_object_tolerance = 0.01


        smach.StateMachine.add('SET_DROP_POSITION',
                                ServiceState(planning_group + '/moveit/set_joint_position',
                                                SetJointPosition,
                                                request_cb=joint_position_request_cb,
                                                response_cb=joint_position_response_cb,
                                                input_keys=['input_planning_group',
                                                            'input_position']),
                                transitions={'succeeded':'OPEN_GRIPPER_DROP'},
                                remapping={'input_planning_group':'planning_group',
                                        'input_position':'drop_position'})
        smach.StateMachine.add('OPEN_GRIPPER_DROP',
                                ServiceState(namespace + '/gripper',
                                                SetJointPosition,
                                                request_cb=gripper_request_cb,
                                                response_cb=gripper_response_cb,
                                                input_keys=['input_planning_group',
                                                            'input_gripper']),
                                transitions={'succeeded':'SET_RELEASE_POSITION'},
                                remapping={'input_planning_group':'planning_group',
                                        'input_gripper':'open_gripper'})   
        smach.StateMachine.add('SET_RELEASE_POSITION',
                                ServiceState(planning_group + '/moveit/set_joint_position',
                                                SetJointPosition,
                                                request_cb=joint_position_request_cb,
                                                response_cb=joint_position_response_cb,
                                                input_keys=['input_planning_group',
                                                            'input_position']),
                                transitions={'succeeded':'SET_ENDINIT_POSITION'},
                                remapping={'input_planning_group':'planning_group',
                                        'input_position':'release_position'}) 

        smach.StateMachine.add('SET_ENDINIT_POSITION',
                                ServiceState(planning_group + '/moveit/set_joint_position',
                                                SetJointPosition,
                                                request_cb=joint_position_request_cb,
                                                response_cb=joint_position_response_cb,
                                                input_keys=['input_planning_group',
                                                            'input_position']),
                                transitions={'succeeded':'succeeded'},
                                remapping={'input_planning_group':'planning_group',
                                        'input_position':'end_init_position'})  




        return drop_center                                      

