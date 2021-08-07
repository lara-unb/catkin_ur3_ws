def control_law(self, msg, ref_pose_msg):
    ##############################################################
    # Loop que gere todas as variaveis do controlador 
    for idx in range(6):

        self.qrk[idx] = ref_pose_msg.data[idx]
        self.qok[idx] = msg.position[idx]

        self.ek[idx] = self.qrk[idx] - self.qok[idx]

        ############################################################
        # Lei de controle para o controlador proposto
        # para todas as juntas 
        self.uk[idx] = (0.958*self.ukmenos1[idx]
         + 0.30067*self.ek[idx] - 0.2108*self.ekmenos1[idx])
        ############################################################]

        self.ukmenos1[idx] = self.uk[idx]
        self.ekmenos1[idx] = self.ek[idx]

        self.ref_vel_msg[idx] = self.uk[idx]

    ###############################################################
    #saida do controlador
    return self.ref_vel_msg
    ###############################################################