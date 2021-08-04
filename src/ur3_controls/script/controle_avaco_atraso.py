#!/usr/bin/env python
import rospy
import rospkg
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64MultiArray
from default_msgs.srv import SetFloat, SetFloatResponse
from sensor_msgs.msg import JointState


class MyControl:

    def __init__(self):
        # Defina os parametros do controlador e variaveis gobais nestre metodo (__init__)

        #################################################################################
        # Para todas as juntas do ur3, da junta da base ate a junta do efetuador terminal
        self.qrk = [0, 0, 0, 0, 0, 0]  # saida Qr[k]  
        self.qrkmenos1 = [0, 0, 0, 0, 0, 0] # saida Qr[k-1]

        self.qok = [0, 0, 0, 0, 0, 0] # entrada Qo[k] 
        self.qokmenos1 = [0, 0, 0, 0, 0, 0] # entrada Qo[k-1]

        self.uk = [0, 0, 0, 0, 0, 0] # sinal de controle U[k] 
        self.ukmenos1 = [0, 0, 0, 0, 0, 0] # sinal de controle U[k-1] 

        self.ek = [0, 0, 0, 0, 0, 0] # sinal de erro E[k] 
        self.ekmenos1 = [0, 0, 0, 0, 0, 0] # sinal de erro E[k-1] 
        #################################################################################

        #################################################################################
        # Construção de todos os topicos e seviços ros

        self.build_ros()


    def build_ros(self):
        # Definição dos topicos 
        self.arm_sub = rospy.Subscriber("/ur3/arm", JointState, self.control_loop)
        self.ref_vel_pub = rospy.Publisher('ur3/ref_vel', Float64MultiArray, queue_size = 1)
        self.ref_pos_pub = rospy.Publisher('ur3/ref_pos', Float64MultiArray, queue_size = 1)

        self.target_pos = [0, 0, 0, 0, 0, 0]
        self.ref_vel_msg = Float64MultiArray()
        self.ref_pose_msg = Float64MultiArray()

    def control_loop(self, msg):
        
        ##############################################################################################
        # Loop que gere todas as variaveis do controlador 
        for idx in range(5):

            self.qrk[idx] = self.target_pos[idx]
            self.qok[idx] = msg.position[idx]

            self.ek[idx] = self.qrk[idx] - self.qok[idx]

            ##########################################################################################
            # Lei de controle para o controlador poroposto
            self.uk[idx] = 0.958*self.ukmenos1[idx] + 0.30067*self.ek[idx] - 0.2108*self.ekmenos1[idx]
            ##########################################################################################

            self.ukmenos1[idx] = self.uk[idx]
            self.ekmenos1[idx] = self.ek[idx]

            self.ref_vel_msg.data[idx] = self.uk[idx]
            self.ref_pose_msg.data[idx] = self.target_pos[idx]
        ##############################################################################################
        
        ##############################################################################################
        # puplicando as mensagens para Interface de Comunicação 
        self.ref_vel_pub.publish(self.ref_vel_msg)
        self.ref_pose_pub.publish(self.ref_pose_msg)
        ##############################################################################################
