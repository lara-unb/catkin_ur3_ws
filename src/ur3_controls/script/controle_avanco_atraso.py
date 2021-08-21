#!/usr/bin/env python

# CONTROLADOR DE POSICAO AVANCO-ATRASO
# Esse codigo eh para ser usado como template para 
# a construcao de outros controladores
class MyControl:
    def __init__(self):
        # Defina os parametros do controlador e variaveis 
        # globais neste metodo (__init__)
        #############################################################
        # Para todas as juntas do ur3, da junta da base 
        # ate a junta do efetuador terminal, defina o estado inicial
        # aqui
        self.Refence_Position = [0, 0, 0, 0, 0, 0] # entrada de posicao qr[k]  
        self.Position = [0, 0, 0, 0, 0, 0] # saida de posicao qo[k] 
        
        self.Refence_Velocity = [0, 0, 0, 0, 0, 0] # sinal de controle u[k] 
        self.Refence_Velocity_Old = [0, 0, 0, 0, 0, 0] # sinal de controle u[k-1] 

        self.Position_Erro = [0, 0, 0, 0, 0, 0] # sinal de erro e[k]
        self.Position_Erro_Old = [0, 0, 0, 0, 0, 0] # sinal de erro e[k-1] 
        
    def control_law(self, robot_state, ref_pose_msg):
        ##############################################################
        # argumendo de control law:
        # self: indica que essa funcao eh acessada apenas pela classe
        # MyControl ou o nome que o usuario queira dar a classe
        # robot_state: mensagem ROS dos tipo JointState
        # mais detalhes da mensagem pode ser encontrado em 
        # http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html
        # ref_pose_msg: mensagem de referencia de posicao para todas as
        # juntas do robo do tipo Float64MultiArray() para 
        # [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3] 
        # mais detalhes da mensagem pode ser encontrado em 
        # http://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html
        ###############################################################
        # for que gere todas as variaveis do controlador 
        for idx in range(6):
            # idx eh um interador que vai de 0 a 5:
            self.Refence_Position[idx] = ref_pose_msg.data[idx]
            self.Position[idx] = robot_state.position[idx]
            # self.ek[idx]: sinal de erro de posicao para
            # cada junta(entra - saida)
            self.Position_Erro[idx] = (self.Refence_Position[idx]
             - self.Position[idx][idx])
            ###########################################################
            # Lei de controle para o controlador proposto
            # para todas as juntas 
            self.Reference_Velocity[idx] = (0.958*self.Velocity_Old[idx]
             + 0.30067*self.Position_Erro[idx]
             - 0.2108*self.Position_Erro_Old[idx])
            ###########################################################
            self.Reference_Velocity_Old[idx] = self.Reference_Velocity[idx]
            self.Position_Erro_Old[idx] = self.Position_Erro[idx]
        ###############################################################
        # sinal de controle parada junta do robo(sinal de velocidade)
        return self.Reference_Velocity
        ###############################################################