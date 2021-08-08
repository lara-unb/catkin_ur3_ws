#!/usr/bin/env python

# CONTROLADOR DE POSICAO AVANCO-ATRASO
# Esse codico eh para ser usado como template para 
# a construcao de outros controladores
class MyControl:
    def __init__(self):
        # Defina os parametros do controlador e variaveis 
        # globais neste metodo (__init__)
        #############################################################
        # Para todas as juntas do ur3, da junta da base 
        # ate a junta do efetuador terminal, defina o estado inicial
        # aqui
        self.qrk = [0, 0, 0, 0, 0, 0] # entrada de posicao Qr[k]  
        self.qok = [0, 0, 0, 0, 0, 0] # saida de posicao Qo[k] 
        
        self.uk = [0, 0, 0, 0, 0, 0] # sinal de controle U[k] 
        self.ukmenos1 = [0, 0, 0, 0, 0, 0] # sinal de controle U[k-1] 

        self.ek = [0, 0, 0, 0, 0, 0] # sinal de erro E[k]
        self.ekmenos1 = [0, 0, 0, 0, 0, 0] # sinal de erro E[k-1] 
        ##############################################################
        ##############################################################
        # velocidade referencia inicial;
        self.ref_vel_msg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ##############################################################
        
    def control_law(self, robot_state, ref_pose_msg):
        #########################################################################
        # argumendo de control law:
        # self: indica que essa funcao eh acessada apenas pela classe
        # MyControl ou o nome que o usuario queira dar a classe
        # robot_state: mensagem ROS dos tipo JointState
        # mais detalhes da mensagem pode ser encontrado em 
        #(http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html)
        #ref_pose_msg: mensagem de referencia de posicao para todas as juntas
        # do robo do tipo Float64MultiArray()
        # [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3] 
        #########################################################################
        # for que gere todas as variaveis do controlador 
        for idx in range(6):
            # idx eh um interador que vai de 0 a 5:
            self.qrk[idx] = ref_pose_msg.data[idx]
            self.qok[idx] = robot_state.position[idx]
            #self.ek[idx]: sinal de erro de posicao para cada junta(entra - saida)
            self.ek[idx] = self.qrk[idx] - self.qok[idx]
            ##########################################################################################
            # Lei de controle para o controlador proposto
            # para todas as juntas 
            self.uk[idx] = 0.958*self.ukmenos1[idx] + 0.30067*self.ek[idx] - 0.2108*self.ekmenos1[idx]
            ##########################################################################################
            self.ukmenos1[idx] = self.uk[idx]
            self.ekmenos1[idx] = self.ek[idx]

            self.ref_vel_msg[idx] = self.uk[idx]
        ###############################################################
        #sinal de controle parada junta do robo(sinal de velocidade)
        return self.ref_vel_msg
        ###############################################################
