// Programa para teste na junta 5 do ur3////////////////////////////////
//commando to setup joystick
//rosparam set joy_node/dev "/dev/input/jsX" change X for your divice.
//rosrun joy joy_node 
//rostopic echo joy 
////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <boost/thread/thread.hpp>
#include "sensor_msgs/JointState.h"
#include "control_msgs/GripperCommand.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Joy.h"
#include "ur3/end_Effector_msg.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "ur3/ref_msg.h"
#include "std_msgs/Float64.h"// lib demo_1
#include <X11/keysymdef.h>
#include <sys/socket.h>
#include <stdlib.h> 
#include <netinet/in.h> 
#include <iostream>
#include <stdio.h>   
#include <string.h>  
#include <string> 
#include <stdlib.h>
#include <sstream>
#include <inttypes.h>
#include <ctime>
#include "open_socket.h"
#include "send_script.h"
#include "read_data.h"


float refe[10];
int gripper_boll[2];

inline int reverse_word(int32_t num){
	uint32_t b0,b1,b2,b3;
	uint32_t res;
	b0 = (num & 0x000000ff) << 24u;
	b1 = (num & 0x0000ff00) << 8u;
	b2 = (num & 0x00ff0000) >> 8u;
	b3 = (num & 0xff000000) >> 24u;
	res = b0 | b1 | b2 | b3;
	return res;
}  

void send_data(int new_socket){
   
  int32_t buffer_in_[8];
  ros::NodeHandle node;
  ros::Publisher ref_pub = node.advertise<ur3::ref_msg>("ref",10);
  ur3::ref_msg ref;

  ref.refer.data.resize(6);
  ros::Rate loop_rate(125);
  float norma_float = 1000000.0;
  
  //Experimento usando um sinal de entrada prbs
	float* prbs_inp = read_data(); 

  int prbs_count = 0;
  while (ros::ok()){
	  	// //referencia 
		// if(prbs_count >=7498){
		// 	 prbs_count = 0;
		// }		
		// prbs_count ++;

		// //printf("%i\n",prbs_count);

	  	ref.refer.data[0] = refe[0];
		buffer_in_[0] = (int)(refe[0]*norma_float);
		buffer_in_[0] = reverse_word(buffer_in_[0]);
		ref.refer.data[1] = refe[1];
		buffer_in_[1] = (int)(refe[1]*norma_float);
		buffer_in_[1] = reverse_word(buffer_in_[1]);
		ref.refer.data[2] = refe[2];
		buffer_in_[2] = (int)(refe[2]*norma_float);
		buffer_in_[2] = reverse_word(buffer_in_[2]);
		ref.refer.data[3] = refe[3];
		buffer_in_[3] = (int)(refe[3]*norma_float);
		buffer_in_[3] = reverse_word(buffer_in_[3]);
		ref.refer.data[4] = refe[4];
		buffer_in_[4] = (int)(refe[4]*norma_float);
		buffer_in_[4] = reverse_word(buffer_in_[4]);
		
		// buffer_in_[5] = (int)((prbs_inp[prbs_count]/4)*norma_float); //sending prbs signal to joint 5
		ref.refer.data[5] = buffer_in_[5];
		buffer_in_[5] = (int)(refe[5]*norma_float);
		buffer_in_[5] = reverse_word(buffer_in_[5]);
		// gripper
		
		refe[6] = gripper_boll[0];
		refe[7] = gripper_boll[1];
		//printf("\n%f",refe[7]);
		buffer_in_[6] = (int)(refe[6]*norma_float);
		buffer_in_[6] = reverse_word(buffer_in_[6]);
		buffer_in_[7] = (int)(refe[7]*norma_float);
		buffer_in_[7] = reverse_word(buffer_in_[7]);
		send(new_socket, buffer_in_, 32, 0);
		/////////////////////////////////////////////////////////
		ref.header.stamp = ros::Time::now();
		ref_pub.publish(ref);
		loop_rate.sleep();
  }
}


/// little endian <-> big endian ///////
inline void reverse (int32_t n[3]){
	// int* vector = new int[2]; 
	n[0] = ( ((n[0] & 0x000000FF)<<24) + ((n[0] & 0x0000FF00)<<8) + ((n[0] & 0x00FF0000)>>8) + (( n[0] & 0xFF000000)>>24) );
	n[1] = ( ((n[1] & 0x000000FF)<<24) + ((n[1] & 0x0000FF00)<<8) + ((n[1] & 0x00FF0000)>>8) + (( n[1] & 0xFF000000)>>24) );
	n[2] = ( ((n[2] & 0x000000FF)<<24) + ((n[2] & 0x0000FF00)<<8) + ((n[2] & 0x00FF0000)>>8) + (( n[2] & 0xFF000000)>>24) );
   return;
}
///////////////////////////////////////
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_data){
	 int button0 = joy_data->buttons[0];
	 int button2 = joy_data->buttons[2];
	 int button3 = joy_data->buttons[3];
	 refe[8] = joy_data->axes[2];
	 if (button0 == 0){
     	//refe[5] = joy_data->axes[3]/2;
	 	// refe[1] = (joy_data->axes[0])*-1/2;
	 	// refe[2] = joy_data->axes[1]/2;
	 	// refe[3] = 0;
	 	// refe[4] = 0;
	 	// refe[5] = 0;
	 } 
	 else
	 {
		// refe[0] = 0;
	 	// refe[1] = 0;
	 	// refe[2] = 0;
		// refe[3] = joy_data->axes[1]/2;
	 	// refe[4] = joy_data->axes[3]/2;
	 	// refe[5] = joy_data->axes[0]/2;
	 }
	 
	 gripper_boll[0] = button2;
	 gripper_boll[1] = button3; 
	
}

void demo_Callback(const std_msgs::Float64::ConstPtr& demo_data){
	refe[0] = demo_data->data;
	refe[1] = refe[0];
	refe[2] = refe[0];
	refe[3] = refe[0];
	refe[4] = refe[0];
	refe[5] = refe[0];	
}
///////////////////////////////////////

int main(int argc, char **argv){ 

	bool statado;
	refe[0] = 0; refe[1] = 0; refe[2] = 0; refe[3] = 0; refe[4] = 0;
	refe[5] = 0; refe[6] = 40; refe[7] = 0; refe[8] = 0;
	
	
	// primeira coisa:
	// tem que enviar o arquivo urscript
	send_script(); // a função send_script envia o arquivo para o robô
	///////////////////////////////////
	int new_socket = open_socket();
	// abrindo a comunicaçção tcp socket
	///////////////////////////////////////////////////////
	//int8_t buffer_out[1024]; 
   	/////////////////////////////
	float norma_float = 1000000.0;
	///////////////////////
	//ROS
	ros::init(argc, argv, "ur3");
	boost::thread thread_b(send_data,new_socket);
	ros::NodeHandle node;
	//Declaração das publicões 
	ros::Publisher arm_pub = node.advertise<sensor_msgs::JointState>("arm",10);
	ros::Publisher end_Effector_pub = node.advertise<ur3::end_Effector_msg>("end_effector",10);
	///////////////////////////////////////////////////////////////////////////////////
	ros::Subscriber sub_joy = node.subscribe("joy", 10, joyCallback);
	ros::Subscriber sub_demo = node.subscribe("demo_1", 10, demo_Callback);
	
	ros::Rate loop_rate(125);
	//Declaração das estruturas de dados para as publicações
	sensor_msgs::JointState arm;
	ur3::end_Effector_msg end_effector;

	arm.header.frame_id = " ";
	arm.name.resize(6);
	arm.position.resize(6);
	arm.velocity.resize(6);
	arm.effort.resize(6); 
	arm.name[0] = "Base";
	arm.name[1] = "Shoulder";
	arm.name[2] = "Elbow";
	arm.name[3] = "Wrist 1";
	arm.name[4] = "Wrist 2";
	arm.name[5] = "Wrist 3";
	
	int prbs_count= 0;
	int32_t vector_arm[3];
	int8_t buffer_out[156]; 
	int b;

	//////////////////////////////////////////////////////////
	
	printf("UR3 is ready!\n");
	
    while (ros::ok()){
		/////////////////////////////////////////////////////
		
		
		/////////////////////////////////////////////////////////////
		b = recv(new_socket, &buffer_out, 156, 0);
		///////////////////////////////////////////////////////////
		//beginning arm 
		memcpy(&vector_arm, &buffer_out[0], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[0] = ((float)vector_arm[0])/norma_float;
		arm.velocity[0] = ((float)vector_arm[1])/norma_float;
		arm.effort[0] = -0.012925*((float)vector_arm[2]);
		//arm.velocity[0] = arm.position[0]/arm.effort[0];
		//printf("%f  %f  %f\n",arm.effort[0],arm.position[0] ,arm.velocity[0]);
		//////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[12], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[1] = ((float)vector_arm[0])/norma_float;
		arm.velocity[1] = ((float)vector_arm[1])/norma_float;
		arm.effort[1] = -0.013088*((float)vector_arm[2]); // 
		// arm.velocity[1] = arm.position[1]/arm.effort[1];
		// printf("%f  %f  %f\n",arm.effort[1],arm.position[1] ,arm.velocity[1]);
		//////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[24], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[2] = ((float)vector_arm[0])/norma_float;
		arm.velocity[2] = ((float)vector_arm[1])/norma_float;
		arm.effort[2] = -0.009358*((float)vector_arm[2]);
		// arm.velocity[2] = arm.position[2]/arm.effort[2];
		// printf("%f  %f  %f\n",arm.effort[2],arm.position[2] ,arm.velocity[2]);
		//////////////////////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[36], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[3] = ((float)vector_arm[0])/norma_float;
		arm.velocity[3] = ((float)vector_arm[1])/norma_float;
		arm.effort[3] = -0.004572*((float)vector_arm[2]);
		// arm.velocity[3] = arm.position[3]/arm.effort[3];
		// printf("%f  %f  %f\n",arm.effort[3],arm.position[3] ,arm.velocity[3]);
		//////////////////////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[48], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[4] = ((float)vector_arm[0])/norma_float;
		arm.velocity[4] = ((float)vector_arm[1])/norma_float;
		arm.effort[4] = -0.004572*((float)vector_arm[2]);
		// arm.velocity[4] = arm.position[4]/arm.effort[4];
		// printf("%f\n",arm.velocity[4]);
		//////////////////////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[60], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[5] = ((float)vector_arm[0])/norma_float;
		arm.velocity[5] = ((float)vector_arm[1])/norma_float;
		arm.effort[5] = -0.004548*((float)vector_arm[2]);
		// arm.velocity[5] = arm.position[5]/arm.effort[5];
		// printf("%f\n",arm.velocity[5]);
		//////////////////////////////////////////////////////
		//end arm
		/////////////////////////////////////////////////
		// beginning gripper
		memcpy(&vector_arm, &buffer_out[72], 3*sizeof(int32_t));
		reverse(vector_arm);
		end_effector.gripper.position = ((float)vector_arm[0])/norma_float;
		end_effector.gripper.max_effort = ((float)vector_arm[1])/norma_float;
		end_effector.state.data = ((float)vector_arm[2])/norma_float;
		/////////////////////////////////////////
		// tcp pose
		////position
		memcpy(&vector_arm, &buffer_out[84], 3*sizeof(int32_t));
		reverse(vector_arm);
		end_effector.pose.position.x = ((float)vector_arm[0])/norma_float;
		end_effector.pose.position.y = ((float)vector_arm[1])/norma_float;
		end_effector.pose.position.z = ((float)vector_arm[2])/norma_float;
		////orientation
		memcpy(&vector_arm, &buffer_out[96], 3*sizeof(int32_t));
		reverse(vector_arm);
		end_effector.pose.orientation.x = ((float)vector_arm[0])/norma_float;
		end_effector.pose.orientation.y = ((float)vector_arm[1])/norma_float;
		end_effector.pose.orientation.z = ((float)vector_arm[2])/norma_float;
		////////////////////////////////////////////////////
		// tcp velocity
		//// linear
		memcpy(&vector_arm, &buffer_out[108], 3*sizeof(int32_t));
		reverse(vector_arm);
		end_effector.velocity.linear.x = ((float)vector_arm[0])/norma_float;
		end_effector.velocity.linear.y = ((float)vector_arm[1])/norma_float;
		end_effector.velocity.linear.z = ((float)vector_arm[2])/norma_float;
		//// angular
		memcpy(&vector_arm, &buffer_out[120], 3*sizeof(int32_t));
		reverse(vector_arm);
		end_effector.velocity.angular.x = ((float)vector_arm[0])/norma_float;
		end_effector.velocity.angular.y = ((float)vector_arm[1])/norma_float;
		end_effector.velocity.angular.z = ((float)vector_arm[2])/norma_float;
		////////////////////////////////////////
		// tcp force
		//// force
		memcpy(&vector_arm, &buffer_out[132], 3*sizeof(int32_t));
		reverse(vector_arm);
		end_effector.wrench.force.x = ((float)vector_arm[0])/norma_float;
		end_effector.wrench.force.y = ((float)vector_arm[1])/norma_float;
		end_effector.wrench.force.z = ((float)vector_arm[2])/norma_float;
		//// torque
		memcpy(&vector_arm, &buffer_out[144], 3*sizeof(int32_t));
		reverse(vector_arm);
		end_effector.wrench.torque.x = ((float)vector_arm[0])/norma_float;
		end_effector.wrench.torque.x = ((float)vector_arm[1])/norma_float;
		end_effector.wrench.torque.x = ((float)vector_arm[2])/norma_float;
		////////////////////////////////////////
		arm.header.stamp = ros::Time::now();
		end_effector.header.stamp = ros::Time::now();
		arm_pub.publish(arm);
		end_Effector_pub.publish(end_effector);
		ros::spinOnce();
		loop_rate.sleep();	
	}
	return 0;
} 

