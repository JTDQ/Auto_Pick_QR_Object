#!/usr/bin/env python
#-*- codinig: UTF-8 -*-
#from launch_demo import launch_demo
import rospy

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
import tf
from std_srvs.srv import SetBool
from std_msgs.msg import Int32
import os
import time
import re
import subprocess
from std_srvs.srv import Trigger

def main():
    rospy.wait_for_service('test')
    try:
        val = rospy.ServiceProxy('test', Trigger)
        resp1 = val(False)
        print resp1.success, resp1.message
    except rospy.ServiceException, e:
        print e

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
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True
class Rc100_service:
    def __init__(self):
        # rospy.Service.
        self.listener=tf.TransformListener()
        self.poseList=[]
        rospy.Subscriber('rc_partol_cmd',Int32,self.rc_call_back)
        rospy.Service('rc_100_service',SetBool,self.rc_serv)
        self.cnt=0
        self.old_msg=-10
        self.stop_partol=False

    def rc_call_back(self,msg):
        if msg.data==self.old_msg:
            return
        else:
            self.old_msg=msg.data
            if msg.data==0:
                p1=subprocess.Popen("roslaunch turtlebot3_slam turtlebot3_slam.launch open_rviz:=true",shell=True,stdout=subprocess.PIPE)
            elif msg.data>=1 and msg.data<=3:
                while True:
                    try:
                        # (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                        pose= self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                        self.poseList.append(pose)
                        rospy.loginfo("get_on")
                        self.cnt+=1
                        break
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.loginfo('no no ')
                        time.sleep(1)
                        continue
            elif msg.data==4:
                self.stop_partol=False
                while self.stop_partol==False:
                    for i in range(3):
                        p2=subprocess.Popen("rosrun map_server map_saver -f /home/sc/map",shell=True,stdout=subprocess.PIPE)
                        print('-----------------------')
                        p2.wait()
                        std_out_s=p2.communicate()
                        m=re.search('Done',std_out_s[0])
                        print (std_out_s[0])
                        # print(m.group())
                        if m is not None:
                            print ('get done')
                            break
                        else:
                            print('fail ...')
                    start_navigation(rc_s)
                    while not rospy.is_shutdown():
                        r.sleep()
            elif msg.data==5:
                self.stop_partol=True
                p3=subprocess.Popen("rosnode kill /turtlebot3_slam_gmapping",shell=True,stdout=subprocess.PIPE)
                p5=subprocess.Popen("rosnode kill /move_base",shell=True,stdout=subprocess.PIPE)
                time.sleep(5)



    def rc_serv(self,date):

        if self.cnt >3:
            print (self.poseList)
            return [False,'efg']
        while True:
            try:
                # (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                pose= self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                self.poseList.append(pose)
                rospy.loginfo("get_on")
                self.cnt+=1
                
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo('no no ')
                continue
        return [True,'abc']
        # pose=self.listener.lookupTransform('/map','/base_link',)
def start_navigation(rc_s):

    listener=tf.TransformListener()
    listener.waitForTransform("map", "base_footprint", rospy.Time(), rospy.Duration(4.0))
    pos = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
    p3=subprocess.Popen("rosnode kill /turtlebot3_slam_gmapping",shell=True,stdout=subprocess.PIPE)
    p5=subprocess.Popen("rosnode kill /move_base",shell=True,stdout=subprocess.PIPE)
    time.sleep(5)


    # p.wait()            
    p4=subprocess.Popen("roslaunch turtlebot3_navigation turtlebot3_navigation.launch open_rviz:=true map_file:=/home/sc/map.yaml",shell=True,stdout=subprocess.PIPE)
    # p4.wait()
    time.sleep(10)
    navi = navigation_demo(pos)
    # navi.set_pose(init_pose)
    while True:
        for pose in rc_s.poseList:
            navi.goto_array(pose)
if __name__ == "__main__":
    # os.system("gnome-terminal -e 'roslaunch turtlebot3_slam turtlebot3_slam.launch open_rviz:=false'")
    # time.sleep(5)

    # p.wait()           

    rospy.init_node('navigation_demo',anonymous=True)
    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')
    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","),goalListY.split(","),goalListYaw.split(","))]
    rc_s=Rc100_service()
    r = rospy.Rate(1)
    r.sleep()
    while not rospy.is_shutdown():
        if rc_s.cnt>3:
            
            for i in range(3):
                p2=subprocess.Popen("rosrun map_server map_saver -f /home/sc/map",shell=True,stdout=subprocess.PIPE)
                print('-----------------------')
                p2.wait()
                std_out_s=p2.communicate()
                m=re.search('Done',std_out_s[0])
                print (std_out_s[0])
                # print(m.group())
                if m is not None:
                    print ('get done')
                    break
                else:
                    print('fail ...')
            start_navigation(rc_s)
            while not rospy.is_shutdown():
                r.sleep()
    rospy.spin()
