import rospy
import re
import subprocess
import tf
from tf_conversions import transformations
import json
# p=subprocess.Popen("roslaunch turtlebot3_slam turtlebot3_slam.launch open_rviz:=false",shell=True,stdout=subprocess.PIPE)
# p.wait()
# std_out_s=p.communicate()
# print (std_out_s[0])

class CruisePose:
    def __init__(self):
        self.cruisePoseList=[1,2]
        self.initTfLink=[3]
        self.filename='cruisePoseList.json'
    def loadPose(self):
        try:
            with open(self.filename) as f_obj:
                (self.cruisePoseList,self.initTfLink)=json.load(f_obj)
        # except expression as identifier:
        except FileNotFoundError:
            # pass
            rospy.logerr('when loading:FileNotFoundError')
        else:
            pass

    def savePose(self):
        try:
            with open(self.filename,'w') as f_obj:
                json.dump((self.cruisePoseList,self.initTfLink),f_obj)
        except FileNotFoundError:
            rospy.logerr('when saving:FileNotFoundError')
            pass
        else:
            print('json save ok')
            pass
# 
# c=CruisePose()
# 
# c.savePose()
# c.loadPose()
# print(c.cruisePoseList)
# print(c.initTfLink)
# 
rospy.init_node('navigation_demo',anonymous=True)
print(rospy.Duration(1))
print(rospy.Time.now()-rospy.Duration(1) )
# p4=subprocess.Popen('''rostopic pub /cmd_vel geometry_msgs/Twist "linear:\n x: 0.0\ny: 0.0\nz: 0.0\nangular:\nx: 0.0\ny: 0.0\nz: 0.0"''',shell=True,stdout=subprocess.PIPE)
# rospy.init_node('navigsation_demo',anonymous=True)

# listener=tf.TransformListener()
# #要等待一会，否则没有连接上
# listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(4.0))
# pos= listener.lookupTransform('/map', '/base_link', rospy.Time(0))
# print(pos)


# for i in range(3):
#     p=subprocess.Popen("rosrun map_server map_saver -f /home/sc/map.yaml",shell=True,stdout=subprocess.PIPE)
#     print('-----------------------')
#     print(p.returncode)
#     p.wait()
#     std_out_s=p.communicate()
#     m=re.search('Done',std_out_s[0])
#     print (std_out_s[0])
#     # print(m.group())
#     if m is not None:
#         print ('get done')
#         break
#     else:
#         print('fail ...')

# print(p.returncode)
# print (p.stdout)