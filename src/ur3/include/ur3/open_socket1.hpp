#ifndef OPEN_SOCKET_H
#define OPEN_SOCKET_H


#include <iostream>
#include <vector>
#include <array>

#include <sys/socket.h>
#include <stdlib.h> 
#include <netinet/in.h> 
#include <stdio.h>   /* Standard input/output definitions */
#include <errno.h>
#include <ros/ros.h>



namespace ur3{

    class OpenSocket{
        public:
            OpenSocket();
            ~OpenSocket() {};

            int connect();
            void disconnect();
            void check_connection(const ros::TimerEvent& event);
            void initialize(ros::NodeHandle &node);


        
        private:

            ros::Timer loop_check_connection_;

            int server_fd_, new_socket_, valread_; 
            struct sockaddr_in address_; 
            int opt_ = 1; 
            int addrlen_ = sizeof(address_);
            
    };

}

#endif