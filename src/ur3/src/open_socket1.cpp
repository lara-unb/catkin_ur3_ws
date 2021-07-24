#include "open_socket1.hpp"

#define PORT 30013

namespace ur3{

    OpenSocket::OpenSocket(){
        // initialize();	
	}

	int OpenSocket::connect(){

		ROS_INFO("Opening Socket Communication ...");	
		
	
		if ((server_fd_ = socket(AF_INET, SOCK_STREAM, 0)) == 0){ 
			perror("socket failed"); 
			exit(EXIT_FAILURE); 
		} 
		
		
		if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt_, sizeof(opt_))){ 
			perror("setsockopt"); 
			exit(EXIT_FAILURE); 
		} 
		address_.sin_family = AF_INET; 
		address_.sin_addr.s_addr = INADDR_ANY; 
		address_.sin_port = htons( PORT ); 
		
		if (bind(server_fd_, (struct sockaddr *)&address_, sizeof(address_))<0){ 
			perror("bind failed"); 
			exit(EXIT_FAILURE); 
		}

		if (listen(server_fd_, 3) < 0){ 
			perror("listen"); 
			exit(EXIT_FAILURE); 
		} 

		if ((new_socket_ = accept(server_fd_, (struct sockaddr *)&address_, (socklen_t*)&addrlen_))<0){ 
			perror("accept"); 
			exit(EXIT_FAILURE); 
		}

		loop_check_connection_.start();


		return new_socket_;
    }

	void OpenSocket::disconnect(){

		close(server_fd_);
        ROS_INFO("Closing Socket Communication ...");

    }

	void OpenSocket::check_connection(const ros::TimerEvent& event){
		int check  = recv(new_socket_,NULL,1, MSG_PEEK | MSG_DONTWAIT);
		// ROS_INFO_STREAM(check);
		if (check != -1)
		{

			disconnect();
			ROS_ERROR("The connection with ur3 was broken. Please, reset the Interface Communication");
			loop_check_connection_.stop();
		}
		
	}

	void OpenSocket::initialize(ros::NodeHandle &node){
		
		loop_check_connection_ = node.createTimer(ros::Duration(1.0), &OpenSocket::check_connection, this);
		loop_check_connection_.stop();
	}
}   
