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
            return SetBoolResponse(self.start, " Stratting exp1")
        else:
            self.start = False
            return SetBoolResponse(self.start, " Strop exp1")


    def loop_ref_vel(self, event):
        
        if self.start is True:
            
            self.ref_vel_msg.data[0] =  0 #self.data_from_csv['q1'][self.interator]/5.0
            self.ref_vel_msg.data[1] =  0 #self.data_from_csv['q2'][self.interator]/5.0
            self.ref_vel_msg.data[2] =  self.data_from_csv['q3'][self.interator]/2.0
            self.ref_vel_msg.data[3] =  0 # self.data_from_csv['q4'][self.interator]/5.0
            self.ref_vel_msg.data[4] =  0 #self.data_from_csv['q5'][self.interator]/5.0
            self.ref_vel_msg.data[5] =  0 #self.data_from_csv['q6'][self.interator]/5.0
            self.ref_vel_pub.publish(self.ref_vel_msg)

            self.interator = self.interator + 1

            if self.interator > self.length_data - 2:
                self.interator = 0

        else:
            self.ref_vel_pub.publish(self.ref_vel_msg_zero)
    
    def read_data(self):

        data_from_csv = {}        
        data_from_csv['q1'] = []
        data_from_csv['q2'] = []
        data_from_csv['q3'] = []
        data_from_csv['q4'] = []
        data_from_csv['q5'] = []
        data_from_csv['q6'] = []

        rospack = rospkg.RosPack()
        path_to_csv_file = rospack.get_path('demos_ur3')
        csv_name = rospy.get_param('/demos_ur3/csv_name')

        with open(path_to_csv_file + '/csv/' + csv_name, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                data_from_csv['q1'].append(float(row[0]))
                data_from_csv['q2'].append(float(row[0]))
                data_from_csv['q3'].append(float(row[0]))
                data_from_csv['q4'].append(float(row[0]))
                data_from_csv['q5'].append(float(row[0]))
                data_from_csv['q6'].append(float(row[0]))

        return data_from_csv


if __name__ == '__main__':
    rospy.init_node('exp1')
    rospy.loginfo("Starting ref to identification")

    try:
        start = IdentificationClass()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass