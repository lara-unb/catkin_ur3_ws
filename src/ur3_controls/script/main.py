#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
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
        # print(self.controller.ping())
       
        ####################################################################################
        # Construco de topicos, servicos e mensagens ROS
        self.arm_sub = rospy.Subscriber("/ur3/arm", JointState, self.arm_callback)
        self.arm_sub.unregister()
        self.ref_vel_pub = rospy.Publisher('ur3/ref_vel', Float64MultiArray, queue_size = 1)
        self.ref_pose_pub = rospy.Publisher('ur3/ref_pose', Float64MultiArray, queue_size = 1)

        self.target_pos = rospy.Subscriber("~" + name_control + "/target_position", JointPosition, self.target_position)
        rospy.Service("~start_" + name_control, SetBool, self.start_control)

        self.ref_vel_msg = Float64MultiArray()
        self.ref_pose_msg = Float64MultiArray()
        self.ref_vel_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.ref_pose_msg.data = [0.0, -1.570796, 0.0, -1.570796 ,0.0 , 0.0]
        # self.target_pose = [0.0, -1.570796, 0.0, -1.570796 ,0.0 , 0.0]

        self.start = False
        #####################################################################################


    def arm_callback(self, msg):

        ref_vel_msg = self.controller.control_law(msg, self.ref_pose_msg)

        for (idx, data) in enumerate(self.ref_vel_msg.data):
            
            self.ref_vel_msg.data[idx] = ref_vel_msg[idx]
            
            if data > 1.5:
                self.ref_vel_msg.data[idx] = 1.5
        ##############################################################################################
        # publicando as mensagens para Interface de Comunicacao 
        self.ref_vel_pub.publish(self.ref_vel_msg)
        self.ref_pose_pub.publish(self.ref_pose_msg)
        ##############################################################################################


    def target_position(self, msg):
        
        if self.start is True:
            self.ref_pose_msg.data[0] = msg.Base
            self.ref_pose_msg.data[1] = msg.Shoulder
            self.ref_pose_msg.data[2] = msg.Elbow
            self.ref_pose_msg.data[3] = msg.Wrist1
            self.ref_pose_msg.data[4] = msg.Wrist2
            self.ref_pose_msg.data[5] = msg.Wrist3

        else:
            self.stop_arm()
            
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

        while abs(self.ref_vel_msg.data[2]) > 0:
            
            self.ref_vel_msg.data[2] = self.ref_vel_msg.data[2]*np.exp(-counter)
            self.ref_vel_pub.publish(self.ref_vel_msg)
            counter = counter + 0.01
            rospy.sleep(0.008)

            if abs(self.ref_vel_msg.data[2]) < 0.05:
                rospy.loginfo("Ref vel is zero !!!")
                self.ref_vel_msg.data[0] = 0.0
                self.ref_vel_msg.data[1] = 0.0
                self.ref_vel_msg.data[2] = 0.0
                self.ref_vel_msg.data[3] = 0.0
                self.ref_vel_msg.data[4] = 0.0
                self.ref_vel_msg.data[5] = 0.0
                self.ref_vel_pub.publish(self.ref_vel_msg)
                rospy.logwarn("Exp2 was fineshed !!!")


if __name__ == '__main__':
    rospy.init_node('control_aa')
    rospy.loginfo("Starting control_aa node")
    rospy.logwarn("Call start sevice as true to move ur3")


    try:
        start = MainNode()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass