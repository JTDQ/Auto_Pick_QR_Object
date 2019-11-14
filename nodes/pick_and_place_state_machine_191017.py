#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# 2019年05月05日13:51:19 测试移动过去后抓取
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
        # self.marker_pose_sub = rospy.Subscriber(
        #     'ar_pose_marker', AlvarMarkers, self.arMarkerMsgCallback)
        self.ball_pose=rospy.Subscriber("/ball_pose",Vector3,self.ball_pose_callBack)
        self.OFFSET_FOR_GOAL_HEIGHT = 0.130
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
            # object_pose.position = self.ar_marker_pose.pose.pose.position
 
            # object_pose.position.x =self.ar_marker_pose.pose.pose.position.z+ 0.0
            # object_pose.position.y  =-self.ar_marker_pose.pose.pose.position.x +0.0
            # object_pose.position.z =-self.ar_marker_pose.pose.pose.position.y + self.OFFSET_FOR_GOAL_HEIGHT
            object_pose.position.x =self.ar_marker_pose.pose.pose.position.x+ 0.0
            object_pose.position.y  =self.ar_marker_pose.pose.pose.position.y +0.0
            object_pose.position.z =0.250 #self.ar_marker_pose.pose.pose.position.z + self.OFFSET_FOR_GOAL_HEIGHT
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


class getCloserToGoal(smach.State):
    def __init__(self):
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
            # rospy.loginfo('ar_marker_pose.x : %f', self.ar_marker_pose.pose.pose.position.x)
            # rospy.loginfo('ar_marker_pose.y : %f', self.ar_marker_pose.pose.pose.position.y)
            # rospy.loginfo('ar_marker_pose.z : %f', self.ar_marker_pose.pose.pose.position.z)
            # rospy.loginfo('ar_marker_pose.yaw : %f', math.degrees(self.getAngleBtwRobotAndMarker(self.ar_marker_pose)))

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
            if objective_function >= 0.210:            
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

# class getCloserToGoal(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded', 'failed'])

#         self.namespace = rospy.get_param("~robot_name")
#         self.marker_pose_sub = rospy.Subscriber(self.namespace + '/ar_pose_marker', AlvarMarkers, self.arMarkerMsgCallback)
#         self.odom_sub = rospy.Subscriber(self.namespace + '/odom', Odometry, self.odomMsgCallback)
#         self.cmd_vel_pub = rospy.Publisher(self.namespace + '/cmd_vel', Twist, queue_size=10)
#         self.tb3_odom = Odometry()

#         self.cmd_vel = Twist()

#         self.priv_dist = 0.0
#         self.priv_heading = 0.0
#         self.ar_marker_pose =False

#     def arMarkerMsgCallback(self, ar_marker_pose_msg):
#         if len(ar_marker_pose_msg.markers) == 0:
#             self.ar_marker_pose = False
#         else:            
#             self.ar_marker_pose = AlvarMarker()
#             self.ar_marker_pose = ar_marker_pose_msg.markers[0]
#             # rospy.loginfo('get a ar')

#     def odomMsgCallback(self, odom_msg):       
#         self.tb3_odom = odom_msg

#     def getDistanceFromRobot(self, goal):
#         return sqrt(goal.pose.pose.position.x**2+goal.pose.pose.position.y**2)
              
#     def getAngleBtwRobotAndMarker(self, goal):
#         return math.atan2(goal.pose.pose.position.y, goal.pose.pose.position.x)

#     def execute(self, userdata):
#         while 1:
#             # rospy.loginfo('ar_marker_pose.x : %f', self.ar_marker_pose.pose.pose.position.x)
#             # rospy.loginfo('ar_marker_pose.y : %f', self.ar_marker_pose.pose.pose.position.y)
#             # rospy.loginfo('ar_marker_pose.z : %f', self.ar_marker_pose.pose.pose.position.z)
#             # rospy.loginfo('ar_marker_pose.yaw : %f', math.degrees(self.getAngleBtwRobotAndMarker(self.ar_marker_pose)))

#             if self.ar_marker_pose == False:
#                 # rospy.loginfo('Failed to get pose of the marker')
#                 self.cmd_vel.linear.x  = 0.0
#                 self.cmd_vel.angular.z = 0.0
            
#                 self.cmd_vel_pub.publish(self.cmd_vel)
#                 continue
            
#             dist    = self.getDistanceFromRobot(self.ar_marker_pose)   # meter
#             heading = self.getAngleBtwRobotAndMarker(self.ar_marker_pose)       # radian
#             radius=dist/(2*math.sin(heading))
#             rad_leng=  2*heading*radius
#             v= 0.2*dist
#             t= rad_leng / v
            
#             w=2*heading/t 
#             # if w >0:
#             #     w=w+0.05
#             # elif w<0:
#             #     w=w-0.05

#             if dist > 0.3  :
#                 self.cmd_vel.linear.x  = v
#                 self.cmd_vel.angular.z =w
#             elif heading >0.7 :
#                 self.cmd_vel.linear.x  = 0
#                 self.cmd_vel.angular.z =  w
#             else:
#                 self.cmd_vel.linear.x  = 0
#                 self.cmd_vel.angular.z =  0
#                 return 'succeeded'
#             self.cmd_vel_pub.publish(self.cmd_vel) 
#             rospy.loginfo((dist,heading,self.cmd_vel.linear.x,self.cmd_vel.angular.z))
#             # objective_function = (1.0 * abs(dist)) + (10.0 * abs(heading))        

#             # # rospy.logwarn('dist: %f, heading: %f, obj_func_result: %f', dist, heading, objective_function)

#             # # dist tolerance: 0.170 meter, heading tolerance: +-0.09 rad (+-5.0 deg)
#             # if objective_function >= 0.210:            
#             #     self.cmd_vel.linear.x  = (0.2 * dist) + (0.02 * (dist - self.priv_dist))
#             #     self.cmd_vel.linear.y  = 0.0 
#             #     self.cmd_vel.linear.z  = 0.0

#             #     self.cmd_vel.angular.x = 0.0  
#             #     self.cmd_vel.angular.y = 0.0 
#             #     self.cmd_vel.angular.z = (1.0 * heading) + (0.01 * (heading - self.priv_heading))
            
#             #     self.cmd_vel_pub.publish(self.cmd_vel)
#             # else:
#             #     self.cmd_vel.linear.x  = 0.0
#             #     self.cmd_vel.angular.z = 0.0
            
#             #     self.cmd_vel_pub.publish(self.cmd_vel)  

#             #     return 'succeeded'

#             # self.priv_dist = dist
#             # self.priv_heading = heading




class Go(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Y','N'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Pick')

        if self.counter <3:
            self.counter += 1
            return 'N'
        else:
            return 'Y'


def main():
    rospy.init_node('pick_and_place_state_machine')
    namespace = rospy.get_param("~robot_name")
    planning_group = rospy.get_param("~planning_group")

    # Create the sub SMACH state machine
    task_center = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])

    # Open the container
    with task_center:
        # the_location_of_the_object = MoveBaseGoal()
        # the_location_of_the_object.target_pose.header.frame_id = "map"
        # the_location_of_the_object.target_pose.header.stamp    = rospy.Time.now()
        # the_location_of_the_object.target_pose.pose.position.x = 1.35
        # the_location_of_the_object.target_pose.pose.position.y = -0.00
        # the_location_of_the_object.target_pose.pose.position.z = 0.0
        # the_location_of_the_object.target_pose.pose.orientation.w = 1
        # the_location_of_the_object.target_pose.pose.orientation.x = 0.0
        # the_location_of_the_object.target_pose.pose.orientation.y = 0.0
        # the_location_of_the_object.target_pose.pose.orientation.z = 0.0
        # smach.StateMachine.add('GO_TO_THE_OBJECT',
        #                         SimpleActionState(namespace + "/move_base", 
        #                                         MoveBaseAction,
        #                                         goal=the_location_of_the_object),
        #                         transitions={'succeeded':'PICK'})

        smach.StateMachine.add('GET_CLOSER_TO_OBJECT', getCloserToGoal(),
                                transitions={'succeeded':'PICK',
                                            'failed':'GET_CLOSER_TO_OBJECT'})
        # smach.StateMachine.add('Pick', Go(),transitions={'N':'PICK','Y':'PICK'})

        # Create the sub SMACH state machine
        # pick_center = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])

        pick_center = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])

        with pick_center:
            pick_center.userdata.planning_group = planning_group
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
                    pick_center.userdata.align_arm_with_object_tolerance += 0.005
                    rospy.logwarn('Set more tolerance[%f]', pick_center.userdata.align_arm_with_object_tolerance)
                    return 'aborted'
                else:
                    OFFSET_FOR_STRETCH = 0.030
                    pick_center.userdata.object_pose.position.x += OFFSET_FOR_STRETCH
                    rospy.sleep(3.)
                    return 'succeeded'

            def close_to_object_response_cb(userdata, response):
                if response.is_planned == False:
                    pick_center.userdata.close_to_object_tolerance += 0.005
                    rospy.logwarn('Set more tolerance[%f]', pick_center.userdata.close_to_object_tolerance)
                    return 'aborted'
                else:
                    OFFSET_FOR_OBJECT_HEIGHT = 0.020
                    pick_center.userdata.object_pose.position.z += OFFSET_FOR_OBJECT_HEIGHT
                    rospy.sleep(3.)
                    return 'succeeded'

            def pick_up_object_response_cb(userdata, response):
                if response.is_planned == False:
                    pick_center.userdata.pick_up_object_tolerance += 0.005
                    rospy.logwarn('Set more tolerance[%f]', pick_center.userdata.pick_up_object_tolerance)
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
#       如果align环节abort，为避免设置同样的初始位置，就进入这个位置。
#       然后这个位置如果abort 就进入 init 的出事位置
            pick_center.userdata.abort_position = [0.0, -0.55, 1.10, -0.44]
            smach.StateMachine.add('SET_Abort_POSITION',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                    SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_position']),
                                   transitions={'succeeded':'ABort_OPEN_GRIPPER'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_position':'abort_position'})
            pick_center.userdata.open_gripper = [0.005]
            smach.StateMachine.add('ABort_OPEN_GRIPPER',
                                    ServiceState(namespace + '/gripper',
                                                    SetJointPosition,
                                                    request_cb=gripper_request_cb,
                                                    response_cb=gripper_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_gripper']),
                                   transitions={'succeeded':'ABort_GET_POSE_OF_THE_OBJECT'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_gripper':'open_gripper'})

            pick_center.userdata.object_pose = Pose()
            smach.StateMachine.add('ABort_GET_POSE_OF_THE_OBJECT', getPoseOfTheObject(),
                                    transitions={'succeeded':'ALIGN_ARM_WITH_OBJECT',
                                                'aborted':'SET_INIT_POSITION'},
                                    remapping={'output_object_pose':'object_pose'})
#       这里设置正常的init position， 对齐不成功，再进入 abort的init position
            pick_center.userdata.init_position = [0.0, -0.65, 1.20, -0.54]
            smach.StateMachine.add('SET_INIT_POSITION',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                    SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_position']),
                                   transitions={'succeeded':'OPEN_GRIPPER'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_position':'init_position'})

            pick_center.userdata.open_gripper = [0.005]
            smach.StateMachine.add('OPEN_GRIPPER',
                                    ServiceState(namespace + '/gripper',
                                                    SetJointPosition,
                                                    request_cb=gripper_request_cb,
                                                    response_cb=gripper_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_gripper']),
                                   transitions={'succeeded':'GET_POSE_OF_THE_OBJECT'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_gripper':'open_gripper'})

            pick_center.userdata.object_pose = Pose()
            smach.StateMachine.add('GET_POSE_OF_THE_OBJECT', getPoseOfTheObject(),
                                    transitions={'succeeded':'ALIGN_ARM_WITH_OBJECT',
                                                'aborted':'SET_Abort_POSITION'},
                                    remapping={'output_object_pose':'object_pose'})
#          对齐后的流程照旧
            pick_center.userdata.align_arm_with_object_tolerance = 0.01
            smach.StateMachine.add('ALIGN_ARM_WITH_OBJECT',
                                    ServiceState(planning_group + '/moveit/set_kinematics_pose',
                                                    SetKinematicsPose,
                                                    request_cb=eef_pose_request_cb,
                                                    response_cb=align_arm_with_object_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_pose',
                                                                'input_tolerance']),                                                    
                                   transitions={'succeeded':'CLOSE_TO_OBJECT',
                                                'aborted':'ALIGN_ARM_WITH_OBJECT'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_pose':'object_pose',
                                            'input_tolerance':'align_arm_with_object_tolerance'})

            pick_center.userdata.close_to_object_tolerance = 0.01
            smach.StateMachine.add('CLOSE_TO_OBJECT',
                                    ServiceState(planning_group + '/moveit/set_kinematics_pose',
                                                    SetKinematicsPose,
                                                    request_cb=eef_pose_request_cb,
                                                    response_cb=close_to_object_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_pose',
                                                                'input_tolerance']),                                                    
                                   transitions={'succeeded':'GRIP_OBJECT',
                                                'aborted':'CLOSE_TO_OBJECT'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_pose':'object_pose',
                                            'input_tolerance':'close_to_object_tolerance'})

            pick_center.userdata.close_gripper = [-0.005]
            smach.StateMachine.add('GRIP_OBJECT',
                                    ServiceState(namespace + '/gripper',
                                                    SetJointPosition,
                                                    request_cb=gripper_request_cb,
                                                    response_cb=gripper_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_gripper']),
                                   transitions={'succeeded':'PICK_UP_OBJECT'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_gripper':'close_gripper'})

            pick_center.userdata.pick_up_object_tolerance = 0.01
            smach.StateMachine.add('PICK_UP_OBJECT',
                                    ServiceState(planning_group + '/moveit/set_kinematics_pose',
                                                    SetKinematicsPose,
                                                    request_cb=eef_pose_request_cb,
                                                    response_cb=pick_up_object_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_pose',
                                                                'input_tolerance']),                                                    
                                   transitions={'succeeded':'SET_HOLDING_POSITION',
                                                'aborted':'PICK_UP_OBJECT'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_pose':'object_pose',
                                            'input_tolerance':'pick_up_object_tolerance'})

            pick_center.userdata.holding_position = [0.0, 0, -0.8, 0.20]
            smach.StateMachine.add('SET_HOLDING_POSITION',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                    SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_position']),
                                   transitions={'succeeded':'SET_PLACE_POSITION'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_position':'holding_position'})
 
            pick_center.userdata.place_position = [1.57, 0.0, 0.00, 0.4]
            smach.StateMachine.add('SET_PLACE_POSITION',
                                    ServiceState(planning_group + '/moveit/set_joint_position',
                                                    SetJointPosition,
                                                    request_cb=joint_position_request_cb,
                                                    response_cb=joint_position_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_position']),
                                   transitions={'succeeded':'OPEN_GRIPPER_PLACE'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_position':'place_position'})
            smach.StateMachine.add('OPEN_GRIPPER_PLACE',
                                    ServiceState(namespace + '/gripper',
                                                    SetJointPosition,
                                                    request_cb=gripper_request_cb,
                                                    response_cb=gripper_response_cb,
                                                    input_keys=['input_planning_group',
                                                                'input_gripper']),
                                   transitions={'succeeded':'SET_RELEASE_POSITION'},
                                   remapping={'input_planning_group':'planning_group',
                                            'input_gripper':'open_gripper'})   
            pick_center.userdata.release_position = [1.57, 0.0, -0.80, 0.4]
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
            pick_center.userdata.end_init_position = [0.0, -0.8, 0.5, -0.54]

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

        smach.StateMachine.add('PICK', pick_center,
                                transitions={'succeeded':'GET_CLOSER_TO_OBJECT', 'aborted':'GET_CLOSER_TO_OBJECT'})


 


    sis = smach_ros.IntrospectionServer(
        'server_name', task_center, '/TASKS_CENTER')
    sis.start()

    # Execute SMACH plan
    outcome = task_center.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
    # main2() # 设置一个单个点去调试 
