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