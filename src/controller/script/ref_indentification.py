#!/usr/bin/env python
import rospy
import rospkg
import os
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64MultiArray


class IdentificationClass():
    def __init__(self):

        self.ref_vel_msg = Float64MultiArray()

        self.ref_vel_msg.append(0.0)
        self.ref_vel_msg.append(0.0)
        self.ref_vel_msg.append(0.0)
        self.ref_vel_msg.append(0.0)
        self.ref_vel_msg.append(0.0)
        self.ref_vel_msg.append(0.0)

        self.start = False 

        rospy.Service("~start", Trigger, self.start_iden)

        self.ref_vel_pub = rospy.Publisher('ur3/ref_vel', , queue_size = 1)

        self.timer = rospy.Timer(rospy.Duration(2), self.loop_ref_vel)

    def start_iden(self, req):

        self.start = True
        return SetStringResponse(True, " Stratting identification")

    def loop_ref_vel(self, event):
        
        if self.start is True:
            self.ref_vel_pub.publish(self.ref_vel_msg)
        else:
            pass
    
    def ref_test(self):

        return 


if __name__ == '__main__':
    rospy.init_node('iden')
    rospy.loginfo("Starting ref to identification")

    try:
        start = dentificationClass()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass