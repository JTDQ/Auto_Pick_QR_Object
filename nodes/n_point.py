#!/usr/bin/env python
#from launch_demo import launch_demo
import rospy

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
import subprocess
import re
import time

class navigation_demo:
    def __init__(self,pos):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        print(pos)
        self.init_pose_from_tf(pos)

    def init_pose_from_tf(self,pos):
        
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = pos[0][0]
        pose.pose.pose.position.y = pos[0][1]
        # q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = pos[1][0]
        pose.pose.pose.orientation.y = pos[1][1]
        pose.pose.pose.orientation.z = pos[1][2]
        pose.pose.pose.orientation.w = pos[1][3]
        self.set_pose_pub.publish(pose)

    def set_pose(self, p):  
        if self.move_base is None:
            return False

        x, y, th = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        feedback
        # rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)
        pass

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s"%p)

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True

    def goto_array(self, p):
        rospy.loginfo("[Navi] goto %s"%p[0][0])

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0][0]
        goal.target_pose.pose.position.y = p[0][1]

        # q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = p[1][0]
        goal.target_pose.pose.orientation.y = p[1][1]
        goal.target_pose.pose.orientation.z = p[1][2]
        goal.target_pose.pose.orientation.w = p[1][3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p[0][0])
            elif state==GoalStatus.ABORTED:
                p4=subprocess.Popen("rosservice call /move_base/clear_costmaps \"{}\"")
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

if __name__ == "__main__":

    rospy.init_node('navigation_demo',anonymous=True)
    # goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    # goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    # goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')

    # goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","),goalListY.split(","),goalListYaw.split(","))]
    p4=subprocess.Popen("roslaunch turtlebot3_navigation turtlebot3_navigation.launch open_rviz:=true map_file:=/home/sc/maps/map.yaml",shell=True,stdout=subprocess.PIPE)
    time.sleep(10)
    
    goals=[[1,0,0],[7,-3,0],[2,-3.5,0]]
    init_pos=([2.250189407141016, -2.466148282204304, 0.009999999686847824], [0.0, 0.0, 0.07283271385144283, 0.9973441554838611])
    navi = navigation_demo(init_pos)

    r = rospy.Rate(1)
    r.sleep()
    while True:
        for goal in goals:
            navi.goto(goal)

    while not rospy.is_shutdown():
        r.sleep()
