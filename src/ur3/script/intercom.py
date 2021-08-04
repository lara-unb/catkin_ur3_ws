#!/usr/bin/env python
import rospy
import rospkg
from std_msgs.msg import String

class  InterCom():
    def __init__(self):

        self.listener_sub = rospy.Subscriber("ur3/listener", String, self.listener_Callback)

    def listener_Callback(self, msg):

        rospy.logwarn(msg)


if __name__ == '__main__':
    rospy.init_node('ur3_listener')
    rospy.loginfo("Starting ur3_listener node")
   
    try:
        start = InterCom()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass