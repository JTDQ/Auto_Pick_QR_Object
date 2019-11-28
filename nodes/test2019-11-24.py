import rospy
import re
import subprocess
import tf
from tf_conversions import transformations

p=subprocess.Popen("roslaunch turtlebot3_slam turtlebot3_slam.launch open_rviz:=false",shell=True,stdout=subprocess.PIPE)
p.wait()
std_out_s=p.communicate()
print (std_out_s[0])


# rospy.init_node('navigsation_demo',anonymous=True)

# listener=tf.TransformListener()
# #要等待一会，否则没有连接上
# listener.waitForTransform("odom", "base_footprint", rospy.Time(), rospy.Duration(4.0))
# pos= listener.lookupTransform('odom','base_footprint', rospy.Time(0))
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