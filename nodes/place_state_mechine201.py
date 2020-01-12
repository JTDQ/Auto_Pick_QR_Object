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
from pick import pick
from drop import drop
from goDestination import goDestiny
# Define
goodPosition=0.24

if __name__ == '__main__':
    rospy.init_node('pick_and_drop_state_machine')
    namespace = rospy.get_param("~robot_name")
    planning_group = rospy.get_param("~planning_group")
    # Create the sub SMACH state machine
    task_center = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])
    # Open the container
    with task_center:
        # step 1: running to the object
        smach.StateMachine.add('GET_CLOSER_TO_OBJECT', getCloserToGoal(goodPosition),
                                transitions={'succeeded':'PICK',
                                            'failed':'GET_CLOSER_TO_OBJECT'})
        # step 2: picking the object
        pick_center = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])
        pick_center = pick(pick_center)
        smach.StateMachine.add('PICK', pick_center,
                                transitions={'succeeded':'GO_DESTINY', 'aborted':'GET_CLOSER_TO_OBJECT'})
        # step 3: running to the destination
        smach.StateMachine.add('GO_DESTINY', goDestiny(),transitions={'N':'DROP','Y':'DROP'})
        # step 4: placing the object
        drop_center = smach.StateMachine(outcomes=['succeeded', 'aborted','preempted'])
        drop_center = drop(drop_center)
        smach.StateMachine.add('DROP', drop_center,
                                transitions={'succeeded':'GET_CLOSER_TO_OBJECT', 'aborted':'GET_CLOSER_TO_OBJECT'})
        #**
    sis = smach_ros.IntrospectionServer(
        'server_name', task_center, '/TASKS_CENTER')
    sis.start()

    # Execute SMACH plan
    outcome = task_center.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()