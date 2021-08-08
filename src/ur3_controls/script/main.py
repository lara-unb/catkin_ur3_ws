#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import csv
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64MultiArray
from default_msgs.srv import SetFloat, SetFloatResponse
from default_msgs.msg import JointPosition
from sensor_msgs.msg import JointState


class MainNode:

    def __init__(self):
        
        name_control = rospy.get_param('/ur3_controls/control/control_name')
        file_control = rospy.get_param('/ur3_controls/control/file_name')
        class_control = rospy.get_param('/ur3_controls/control/class_name')
        fuul_name = file_control + "." + class_control

        mod = __import__(file_control)
        MyControl = getattr(mod, class_control)
        self.controller = MyControl()
        
        self.activated_joints = rospy.get_param('/ur3_controls/activated_joints')
        self.reference_type = rospy.get_param('/ur3_controls/reference_type')
        csv_name = rospy.get_param('/ur3_controls/csv_name')

        if self.reference_type['type'] == "square":
            self.csv = csv_name["square"]
        if self.reference_type['type'] == "sine":
            self.csv = csv_name["sine"]
        if self.reference_type['type'] == "your_wave":
            self.csv = csv_name["your_wave"]

        self.data_from_csv = self.read_data()
        self.interator = 0
        self.length_data = len(self.data_from_csv['q1'])
       
        ####################################################################################
        # Construco de topicos, servicos e mensagens ROS
        self.arm_sub = rospy.Subscriber("/ur3/arm", JointState, self.arm_callback)
        self.arm_initial_pose_sub = rospy.Subscriber("/ur3/arm", JointState, self.arm_initial_pose)
        self.arm_sub.unregister()
        self.ref_vel_pub = rospy.Publisher('ur3/ref_vel', Float64MultiArray, queue_size = 1)
        self.ref_pose_pub = rospy.Publisher('ur3/ref_pose', Float64MultiArray, queue_size = 1)

        self.target_pos = rospy.Subscriber("~" + name_control + "/target_position", JointPosition, self.target_position)
        rospy.Service("~start_" + name_control, SetBool, self.start_control)

        self.ref_vel_msg = Float64MultiArray()
        self.ref_pose_msg = Float64MultiArray()
        self.ref_pose_msg_on = Float64MultiArray()
        self.ref_pose_msg_off = Float64MultiArray()
        self.ref_vel_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ref_pose_msg_on.data = [0.0, -1.570796, 0.0, -1.570796 ,0.0 , 0.0]
        self.ref_pose_msg_off.data = [0.0, -1.570796, 0.0, -1.570796 ,0.0 , 0.0]
        # self.target_pose = [0.0, -1.570796, 0.0, -1.570796 ,0.0 , 0.0]

        self.start = False
        self.max_vel = 0.1
        #####################################################################################

    def arm_initial_pose(self, msg):

        self.ref_pose_msg_on.data[0] =  msg.position[0]
        self.ref_pose_msg_on.data[1] =  msg.position[1]
        self.ref_pose_msg_on.data[2] =  msg.position[2]
        self.ref_pose_msg_on.data[3] =  msg.position[3]
        self.ref_pose_msg_on.data[4] =  msg.position[4]
        self.ref_pose_msg_on.data[5] =  msg.position[5]

        self.ref_pose_msg_off = self.ref_pose_msg_on

        self.arm_initial_pose_sub.unregister()




    def arm_callback(self, msg):

        if self.start is True:

            if self.reference_type["online"] is False:
                self.ref_pose_msg_off.data[0] = self.data_from_csv['q1'][self.interator]
                self.ref_pose_msg_off.data[1] = self.data_from_csv['q2'][self.interator]
                self.ref_pose_msg_off.data[2] = self.data_from_csv['q3'][self.interator]
                self.ref_pose_msg_off.data[3] = self.data_from_csv['q4'][self.interator]
                self.ref_pose_msg_off.data[4] = self.data_from_csv['q5'][self.interator]
                self.ref_pose_msg_off.data[5] = self.data_from_csv['q6'][self.interator]
                self.ref_pose_msg =  self.ref_pose_msg_off
                self.interator = self.interator + 1

                if self.interator > self.length_data -2:
                    self.interator = 0
                    rospy.logwarn("The experiment " + self.csv + " reached the end")
                    self.start = False
            else:
                self.ref_pose_msg = self.ref_pose_msg_on

            ref_vel_msg = self.controller.control_law(msg, self.ref_pose_msg)

            for (idx, data) in enumerate(ref_vel_msg):
                
                self.ref_vel_msg.data[idx] = ref_vel_msg[idx]

                if abs(data) > self.max_vel:
                    self.ref_vel_msg.data[idx] = (data/abs(data))*self.max_vel
            ##############################################################################################
            # publicando as mensagens para Interface de Comunicacao
            if self.activated_joints["Base"] == False:
                self.ref_vel_msg.data[0] = 0.0
            if self.activated_joints["Shoulder"] == False:
                self.ref_vel_msg.data[1] = 0.0
            if self.activated_joints["Elbow"] == False:
                self.ref_vel_msg.data[2] = 0.0
            if self.activated_joints["Wrist1"] == False:
                self.ref_vel_msg.data[3] = 0.0
            if self.activated_joints["Wrist2"] == False:
                self.ref_vel_msg.data[4] = 0.0
            if self.activated_joints["Wrist3"] == False:
                self.ref_vel_msg.data[5] = 0.0

            self.ref_vel_pub.publish(self.ref_vel_msg)
            self.ref_pose_pub.publish(self.ref_pose_msg)
            ##############################################################################################
        else:
            self.arm_sub.unregister()
            self.stop_arm()



    def target_position(self, msg):
        
        if self.start is True and self.reference_type["online"] is True:
            self.ref_pose_msg_on.data[0] = msg.Base
            self.ref_pose_msg_on.data[1] = msg.Shoulder
            self.ref_pose_msg_on.data[2] = msg.Elbow
            self.ref_pose_msg_on.data[3] = msg.Wrist1
            self.ref_pose_msg_on.data[4] = msg.Wrist2
            self.ref_pose_msg_on.data[5] = msg.Wrist3

        else:
            pass
            
    def start_control(self, req):

        if req.data == True:
            rospy.logwarn("Startting control_aa!!!")
            
            if self.start is True:
                return SetBoolResponse(False, " control_aa was aready started")

            self.arm_sub = rospy.Subscriber("/ur3/arm", JointState, self.arm_callback)
            self.start = True
            return SetBoolResponse(True, " Stratting control_aa")
        else:
            if self.start is False:
                return SetBoolResponse(False, " control_aa was aready stoped")

            self.arm_sub.unregister()
            self.target_pos.unregister()

            self.start = False
            self.stop_arm()
            return SetBoolResponse(True, " Strop control_aa")

    def stop_arm(self):

        counter = 0.0
        rospy.loginfo("Stopping ARM!")

        while (abs(self.ref_vel_msg.data[0]) > 0 or abs(self.ref_vel_msg.data[1]) > 0 or abs(self.ref_vel_msg.data[2]) > 0
            or abs(self.ref_vel_msg.data[3]) > 0 or abs(self.ref_vel_msg.data[4]) > 0 or abs(self.ref_vel_msg.data[5]) > 0):
            
            self.ref_vel_msg.data[0] = self.ref_vel_msg.data[0]*np.exp(-counter)
            self.ref_vel_msg.data[1] = self.ref_vel_msg.data[1]*np.exp(-counter)
            self.ref_vel_msg.data[2] = self.ref_vel_msg.data[2]*np.exp(-counter)
            self.ref_vel_msg.data[3] = self.ref_vel_msg.data[3]*np.exp(-counter)
            self.ref_vel_msg.data[4] = self.ref_vel_msg.data[4]*np.exp(-counter)
            self.ref_vel_msg.data[5] = self.ref_vel_msg.data[5]*np.exp(-counter)
            self.ref_vel_pub.publish(self.ref_vel_msg)
            counter = counter + 0.03
            rospy.sleep(0.008)

            vel_average = (abs(self.ref_vel_msg.data[0]) + abs(self.ref_vel_msg.data[1]) + abs(self.ref_vel_msg.data[2])
                           + abs(self.ref_vel_msg.data[3]) +abs(self.ref_vel_msg.data[4]) +abs(self.ref_vel_msg.data[5]))/6.0


            if vel_average < 0.02:
                rospy.loginfo("Ref vel is zero !!!")
                self.ref_vel_msg.data[0] = 0.0
                self.ref_vel_msg.data[1] = 0.0
                self.ref_vel_msg.data[2] = 0.0
                self.ref_vel_msg.data[3] = 0.0
                self.ref_vel_msg.data[4] = 0.0
                self.ref_vel_msg.data[5] = 0.0
                self.ref_vel_pub.publish(self.ref_vel_msg)
                rospy.logwarn("control_aa was fineshed !!!")

    def read_data(self):

        data_from_csv = {}        
        data_from_csv['q1'] = []
        data_from_csv['q2'] = []
        data_from_csv['q3'] = []
        data_from_csv['q4'] = []
        data_from_csv['q5'] = []
        data_from_csv['q6'] = []
       
        rospack = rospkg.RosPack()
        path_to_csv_file = rospack.get_path('ur3_controls')
        
        
        with open(path_to_csv_file + '/csv/' + self.csv, 'r') as file:
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
    rospy.init_node('control_aa')
    rospy.loginfo("Starting control_aa node")
    rospy.logwarn("Call start sevice as true to move ur3")


    try:
        start = MainNode()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass