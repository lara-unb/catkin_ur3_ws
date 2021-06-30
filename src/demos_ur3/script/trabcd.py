#!/usr/bin/env python
import rospy
import rospkg
import csv
import os
from copy import deepcopy
import numpy as np
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from default_msgs.srv import SetFloat, SetFloatResponse
from sensor_msgs.msg import JointState


class  Trabcd():
    def __init__(self):

        self.qrk = 0
        self.qrkmenos1 = 0

        self.qok = 0
        self.qokmenos1 = 0

        self.ukmenos1 = 0
        self.ekmenos1 = 0

        self.target_pos = 0 

        self.arm_msg = 0  

        self.ref_vel_msg = Float64MultiArray()
        self.ref_pos_msg = Float64()

        self.ref_vel_msg_zero = Float64MultiArray()

        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)

        self.ref_vel_msg_zero = deepcopy(self.ref_vel_msg)
    
        self.ref_type = rospy.get_param('/demos_ur3/ref_type')
        csv_name = rospy.get_param('/demos_ur3/csv_name')

            
        if self.ref_type['type'] == "square":
            self.csv = csv_name["square"]
        if self.ref_type['type'] == "sine":
            self.csv = csv_name["sine"]

         
        self.data_from_csv = self.read_data()
        self.interator = 0
        self.length_data = len(self.data_from_csv['q3'])


        self.start = False 

        rospy.Service("~target_position", SetFloat, self.target_position)

        rospy.Service("~start", SetBool, self.start_control)

        rospy.Subscriber("/ur3/arm", JointState, self.arm_callback)

        self.ref_vel_pub = rospy.Publisher('ur3/ref_vel', Float64MultiArray, queue_size = 1)
        self.ref_pos_pub = rospy.Publisher('ur3/ref_pos', Float64, queue_size = 1)


    def start_control(self, req):

        if req.data == True:

            self.start = True
            return SetBoolResponse(self.start, " Stratting tracd")
        else:
            self.start = False
            self.interator = 0
            return SetBoolResponse(self.start, " Strop trabcd")

    def target_position(self, req):
        
        if self.ref_type["online"] is True:
            self.target_pos = req.newFloat

            return SetFloatResponse(True, "Setted position: " + str(req.newFloat))
        else:
            return SetFloatResponse(False, "Type of control is offline!")

    def arm_callback(self, msg):

        if (self.ref_type["online"] is False) and (self.start is True):
            self.target_pos = self.data_from_csv['q3'][self.interator]
            self.interator = self.interator + 1

            if self.interator > self.length_data -10:
                self.start = False
                self.interator = 0

        self.ref_pos_msg.data = self.target_pos
        self.qrk = self.target_pos
        self.qok = msg.position[2]
        
        ek = self.qrk - self.qok

        uk = 0.958*self.ukmenos1 + 0.30067*ek - 0.2108*self.ekmenos1

        if uk > 1.5:
            uk = 1.5

        if uk < -1.5:
            uk = -1.5


        if self.start is True:

            self.ref_vel_msg.data[2] = uk
            
            self.ref_vel_pub.publish(self.ref_vel_msg)
            self.ref_pos_pub.publish(self.ref_pos_msg)

        else:
            
            self.ref_vel_pub.publish(self.ref_vel_msg_zero)
            self.ref_pos_pub.publish(self.ref_pos_msg)

        

        self.ukmenos1 = uk
        self.ekmenos1 = ek

    def read_data(self):

        data_from_csv = {}        
        data_from_csv['q3'] = []
       
        rospack = rospkg.RosPack()
        path_to_csv_file = rospack.get_path('demos_ur3')
        
        
        with open(path_to_csv_file + '/csv/' + self.csv, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                
                data_from_csv['q3'].append(float(row[0]))
          
        return data_from_csv

if __name__ == '__main__':
    rospy.init_node('trabcd')
    rospy.loginfo("Starting control trabcd")

    try:
        start = Trabcd()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass