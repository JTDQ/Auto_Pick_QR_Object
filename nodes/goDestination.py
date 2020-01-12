import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
from smach_ros import ServiceState

class goDestiny(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Y','N'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Pick')
        # while True:
        #     rospy.Subscriber('')
        if self.counter <3:
            self.counter += 1
            return 'N'
        else:
            return 'Y'