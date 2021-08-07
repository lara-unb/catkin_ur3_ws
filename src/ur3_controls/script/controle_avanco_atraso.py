#!/usr/bin/env python
# CONTROLADOR DE POSICAO
class MyControl:
    def __init__(self):
        # Defina os parametros do controlador e variaveis 
        # globais neste metodo (__init__)
        #############################################################
        # Para todas as juntas do ur3, da junta da base 
        # ate a junta do efetuador terminal, defina o estado inicial
        # aqui
        self.qrk = [0, 0, 0, 0, 0, 0]  # saida Qr[k]  
        self.qrkmenos1 = [0, 0, 0, 0, 0, 0] # saida Qr[k-1]

        self.qok = [0, 0, 0, 0, 0, 0] # entrada Qo[k] 
        self.qokmenos1 = [0, 0, 0, 0, 0, 0] # entrada Qo[k-1]

        self.uk = [0, 0, 0, 0, 0, 0] # sinal de controle U[k] 
        self.ukmenos1 = [0, 0, 0, 0, 0, 0] # sinal de controle U[k-1] 

        self.ek = [0, 0, 0, 0, 0, 0] # sinal de erro E[k] 
        self.ekmenos1 = [0, 0, 0, 0, 0, 0] # sinal de erro E[k-1] 
        ##############################################################

        ##############################################################
        # Construcao das mensagens de posicao de referencia e de
        # velocidade referencia;
        self.ref_vel_msg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ##############################################################
        
    def control_law(self, msg, ref_pose_msg):
        ##############################################################
        # Loop que gere todas as variaveis do controlador 
        for idx in range(6):

            self.qrk[idx] = ref_pose_msg.data[idx]
            self.qok[idx] = msg.position[idx]

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
        #saida do controlador
        return self.ref_vel_msg
        ###############################################################
