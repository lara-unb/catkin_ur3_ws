#!/usr/bin/env python
import rospy
import rospkg
import csv
import os
from copy import deepcopy
import numpy as np
from std_srvs.srv import SetBool, SetBoolResponse
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

        self.ref_vel_msg_zero = deepcopy(self.ref_vel_msg)

        self.data_from_csv = self.read_data()

        self.interator = 0
        self.length_data = len(self.data_from_csv['q3'])

        self.start = False 

        rospy.Service("~start", SetBool, self.start_iden)

        self.ref_vel_pub = rospy.Publisher('ur3/ref_vel', Float64MultiArray, queue_size = 1)

        self.timer = rospy.Timer(rospy.Duration(0.008), self.loop_ref_vel)

    def start_iden(self, req):

        if req.data == True:

            self.start = True
            return SetBoolResponse(self.start, " Stratting identification")
        else:
            self.start = False
            return SetBoolResponse(self.start, " Strop identification")


    def loop_ref_vel(self, event):
        
        if self.start is True:
            
            self.ref_vel_msg.data[2] =  self.data_from_csv['q3'][self.interator]/2.0
            self.ref_vel_pub.publish(self.ref_vel_msg)

            self.interator = self.interator + 1

            if self.interator > self.length_data - 10:
                self.interator = 0

        else:
            self.ref_vel_pub.publish(self.ref_vel_msg_zero)
    
    def read_data(self):

        data_from_csv = {}        
        data_from_csv['q3'] = []

        rospack = rospkg.RosPack()
        path_to_csv_file = rospack.get_path('controller')
        csv_name = rospy.get_param('vel_ur3')
        with open(path_to_csv_file + '/csv/' + csv_name', 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                data_from_csv['q3'].append(float(row[0]))

        return data_from_csv


if __name__ == '__main__':
    rospy.init_node('iden')
    rospy.loginfo("Starting ref to identification")

    try:
        start = IdentificationClass()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass