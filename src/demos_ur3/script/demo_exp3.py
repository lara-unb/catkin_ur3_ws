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


class  Exp3():
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
        self.ref_pos_msg = Float64MultiArray()

        self.ref_vel_msg_zero = Float64MultiArray()

        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)
        self.ref_vel_msg.data.append(0.0)

        self.ref_pos_msg.data.append(0.0)
        self.ref_pos_msg.data.append(0.0)
        self.ref_pos_msg.data.append(0.0)
        self.ref_pos_msg.data.append(0.0)
        self.ref_pos_msg.data.append(0.0)
        self.ref_pos_msg.data.append(0.0)

        self.ref_vel_msg_zero = deepcopy(self.ref_vel_msg)
    
        self.ref_type = rospy.get_param('/demos_ur3/ref_type')
        csv_name = rospy.get_param('/demos_ur3/csv_name')

            
        if self.ref_type['type'] == "square":
            self.csv = csv_name["square"]
        elif self.ref_type['type'] == "sine":
            self.csv = csv_name["sine"]

         
        self.data_from_csv = self.read_data()
        self.interator = 0
        self.length_data = len(self.data_from_csv['q1'])


        self.start = False 

        rospy.Service("~target_position", SetFloat, self.target_position)

        rospy.Service("~start_exp3", SetBool, self.start_control)

        self.arm_sub = rospy.Subscriber("/ur3/arm", JointState, self.arm_callback)
        self.arm_sub.unregister()
        
        self.ref_vel_pub = rospy.Publisher('ur3/ref_vel', Float64MultiArray, queue_size = 1)
        self.ref_pos_pub = rospy.Publisher('ur3/ref_pos', Float64MultiArray, queue_size = 1)


    def start_control(self, req):

        if req.data == True:
            rospy.logwarn("Startting Exp3!!!")
            
            if self.start is True:
                return SetBoolResponse(False, " Exp3 was aready started")

            self.arm_sub = rospy.Subscriber("/ur3/arm", JointState, self.arm_callback)
            self.start = True
            return SetBoolResponse(True, " Stratting demo_exp3")
        else:
            if self.start is False:
                return SetBoolResponse(False, " Exp3 was aready stoped")
            
            self.start = False
            self.interator = 0
            return SetBoolResponse(True, " Strop demo_exp3")

    def target_position(self, req):
        
        if self.ref_type["online"] is True and self.start is True:
            self.reset_control()
            self.target_pos = req.newFloat
            
            return SetFloatResponse(True, "Setted position: " + str(req.newFloat))
        else:
            return SetFloatResponse(False, "Type of control is offline or was not startted!")

    def arm_callback(self, msg):

        if self.start is True:

            if self.ref_type["online"] is False:
                self.target_pos = self.data_from_csv['q1'][self.interator]
                self.interator = self.interator + 1

                if self.interator > self.length_data -2:
                    self.start = False

            self.ref_pos_msg.data[0] = self.target_pos
            self.qrk = self.target_pos
            self.qok = msg.position[0]
            
            ek = self.qrk - self.qok

            uk = 0.9*self.ukmenos1 + 0.3*ek - 0.12*self.ekmenos1

            if uk > 1.5:
                uk = 1.5

            if uk < -1.5:
                uk = -1.5

            self.ref_vel_msg.data[0] = uk  
            self.ref_vel_pub.publish(self.ref_vel_msg)
            self.ref_pos_pub.publish(self.ref_pos_msg)

            self.ukmenos1 = uk
            self.ekmenos1 = ek

        else:
            self.arm_sub.unregister()
            self.stop_arm()
            self.reset_control()
            

  

    def reset_control(self):
        self.qrk = 0
        self.qrkmenos1 = 0

        self.qok = 0
        self.qokmenos1 = 0

        self.ukmenos1 = 0
        self.ekmenos1 = 0

        self.target_pos = 0 

        self.arm_msg = 0
        self.interator = 0

    def stop_arm(self):

        self.interator = 0
        counter = 0.0
        rospy.loginfo("Stopping ARM!")

        while abs(self.ref_vel_msg.data[0]) > 0:
            
            self.ref_vel_msg.data[0] = self.ref_vel_msg.data[0]*np.exp(-counter)
            self.ref_vel_pub.publish(self.ref_vel_msg)
            counter = counter + 0.01
            rospy.sleep(0.008)

            if abs(self.ref_vel_msg.data[0]) < 0.05:
                rospy.loginfo("Ref vel is zero !!!")
                self.ref_vel_msg.data[0] = 0.0
                self.ref_vel_pub.publish(self.ref_vel_msg)
                rospy.logwarn("Exp3 was fineshed !!!")


    def read_data(self):

        data_from_csv = {}        
        data_from_csv['q1'] = []
       
        rospack = rospkg.RosPack()
        path_to_csv_file = rospack.get_path('demos_ur3')
        
        
        with open(path_to_csv_file + '/csv/' + self.csv, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                
                data_from_csv['q1'].append(float(row[0]))
          
        return data_from_csv

if __name__ == '__main__':
    rospy.init_node('demo_exp3')
    rospy.loginfo("Starting demo_exp3 node")
    rospy.logwarn("Call start sevice as true to move ur3")


    try:
        start = Exp3()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass