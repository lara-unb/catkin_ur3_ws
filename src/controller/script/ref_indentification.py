#!/usr/bin/env python
import rospy
import rospkg
import csv
import os
import numpy as np
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64MultiArray


class IdentificationClass():
    def __init__(self):

        self.ref_vel_msg = Float64MultiArray()

        self.ref_vel_msg_zero = Float64MultiArray()

        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)

        self.ref_vel_msg_zero = self.ref_vel_msg

        self.data_from_csv = self.read_data()

        self.interator = 0
        self.length_data = len(self.data_from_csv['q6'])

        self.start = False 

        rospy.Service("~start", Trigger, self.start_iden)

        self.ref_vel_pub = rospy.Publisher('ur3/ref_vel', Float64MultiArray, queue_size = 1)

        self.timer = rospy.Timer(rospy.Duration(0.008), self.loop_ref_vel)

    def start_iden(self, req):

        self.start = True
        return TriggerResponse(True, " Stratting identification")

    def loop_ref_vel(self, event):
        
        if self.start is True:
            
            self.ref_vel_msg.data[0] =  self.data_from_csv['q6'][self.interator]
            self.ref_vel_pub.publish(self.ref_vel_msg)

            self.interator = self.interator + 1

            if self.interator > self.length_data - 10:
                self.interator = 0

        else:
            self.ref_vel_pub.publish(self.ref_vel_msg_zero)
    
    def read_data(self):

        data_from_csv = {}        
        data_from_csv['q1'] = []

        rospack = rospkg.RosPack()
        path_to_csv_file = rospack.get_path('controller')

        with open(path_to_csv_file + '/csv/vel_ur3.csv', 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                data_from_csv['q1'].append(float(row[0]))

        return data_from_csv


if __name__ == '__main__':
    rospy.init_node('iden')
    rospy.loginfo("Starting ref to identification")

    try:
        start = IdentificationClass()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass