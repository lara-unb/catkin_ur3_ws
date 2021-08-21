#!/usr/bin/env python

# CONTROLADOR DE POSICAO AVANCO-ATRASO
# Esse codigo eh para ser usado como template para 
# a construcao de outros controladores
class MyControl:
    def __init__(self):
        # Defina os parametros do controlador e variaveis 
        # globais neste metodo (__init__)
        # Para todas as juntas do ur3, da junta da base 
        # ate a junta do efetuador terminal, defina o estado
        # inicial aqui

        self.Refence_Position = [0, 0, 0, 0, 0, 0] # entrada de posicao   
        self.Position = [0, 0, 0, 0, 0, 0] # saida de posicao 
        
        self.Refence_Velocity = [0, 0, 0, 0, 0, 0] # sinal de controle 
        self.Refence_Velocity_Old = [0, 0, 0, 0, 0, 0] # sinal de controle 
        # atrasado de uma amostra

        self.Position_Erro = [0, 0, 0, 0, 0, 0] # sinal de erro
        self.Position_Erro_Old = [0, 0, 0, 0, 0, 0] # sinal de erro  
        # atrasado de uma amostra 