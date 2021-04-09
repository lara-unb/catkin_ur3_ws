// Programa para teste na junta 5 do ur3////////////////////////////////
//commando to setup joystick
//rosparam set joy_node/dev "/dev/input/jsX" change X for your divice.
//rosrun joy joy_node 
//rostopic echo joy 
////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "ur3/end_Effector_msg.h"
#include "ur3/ref_msg.h"
#include "std_msgs/Float64.h"// lib demo_1
#include <sys/socket.h>
#include <string.h>  
#include "open_socket.h"
#include "send_script.h"
#include "read_data.h"

#define RATE_LOOP (0.008F) // in sec

class Interface{
	private:

		int32_t buffer_in_[8];
		float ref_vel_[10];
		ros::NodeHandle  class_node_;

		ros::Publisher arm_pub_;
		ros::Publisher end_Effector_pub_;
		ros::Subscriber sub_ref_vel_;

		int new_socket_; 

		ros::Timer l_timer_;

		float norma_float = 1000000.0;
		sensor_msgs::JointState arm;
		ur3::end_Effector_msg end_effector;
		int32_t vector_arm[3];
		int8_t buffer_out[156]; 
		int b;


	public:

		Interface(ros::NodeHandle &node){
			
			class_node_ = node;
			std::cout << "init Interface" << std::endl;

			new_socket_ = open_socket();

			arm_pub_ = node.advertise<sensor_msgs::JointState>("~arm",10);
			end_Effector_pub_ = node.advertise<ur3::end_Effector_msg>("~end_effector",10);
			
			l_timer_ = class_node_.createTimer(ros::Duration(RATE_LOOP), boost::bind(&Interface::arm_pub_state, this));
			sub_ref_vel_ = node.subscribe("~ref_vel", 10, &Interface::ref_vel_Callback, this);

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
			
	};

	void reverse_word(int32_t &num){

		uint32_t b0,b1,b2,b3;

		b0 = (num & 0x000000ff) << 24u;
		b1 = (num & 0x0000ff00) << 8u;
		b2 = (num & 0x00ff0000) >> 8u;
		b3 = (num & 0xff000000) >> 24u;
		num = b0 | b1 | b2 | b3;
	} 

	void reverse (int32_t n[3]){

		n[0] = ( ((n[0] & 0x000000FF)<<24) + ((n[0] & 0x0000FF00)<<8) + ((n[0] & 0x00FF0000)>>8) + (( n[0] & 0xFF000000)>>24) );
		n[1] = ( ((n[1] & 0x000000FF)<<24) + ((n[1] & 0x0000FF00)<<8) + ((n[1] & 0x00FF0000)>>8) + (( n[1] & 0xFF000000)>>24) );
		n[2] = ( ((n[2] & 0x000000FF)<<24) + ((n[2] & 0x0000FF00)<<8) + ((n[2] & 0x00FF0000)>>8) + (( n[2] & 0xFF000000)>>24) );
	}

	void arm_pub_state(){

		b = recv(new_socket_, &buffer_out, 156, 0);
		///////////////////////////////////////////////////////////
		//beginning arm 
		memcpy(&vector_arm, &buffer_out[0], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[0] = ((float)vector_arm[0])/norma_float;
		arm.velocity[0] = ((float)vector_arm[1])/norma_float;
		arm.effort[0] = -0.012925*((float)vector_arm[2]);
		//////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[12], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[1] = ((float)vector_arm[0])/norma_float;
		arm.velocity[1] = ((float)vector_arm[1])/norma_float;
		arm.effort[1] = -0.013088*((float)vector_arm[2]); // 
		//////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[24], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[2] = ((float)vector_arm[0])/norma_float;
		arm.velocity[2] = ((float)vector_arm[1])/norma_float;
		arm.effort[2] = -0.009358*((float)vector_arm[2]);
		//////////////////////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[36], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[3] = ((float)vector_arm[0])/norma_float;
		arm.velocity[3] = ((float)vector_arm[1])/norma_float;
		arm.effort[3] = -0.004572*((float)vector_arm[2]);
		/////////////////////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[48], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[4] = ((float)vector_arm[0])/norma_float;
		arm.velocity[4] = ((float)vector_arm[1])/norma_float;
		arm.effort[4] = -0.004572*((float)vector_arm[2]);
		//////////////////////////////////////////////////////
		memcpy(&vector_arm, &buffer_out[60], 3*sizeof(int32_t));
		reverse(vector_arm);
		arm.position[5] = ((float)vector_arm[0])/norma_float;
		arm.velocity[5] = ((float)vector_arm[1])/norma_float;
		arm.effort[5] = -0.004548*((float)vector_arm[2]);
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
		arm_pub_.publish(arm);
		end_Effector_pub_.publish(end_effector);

  	}

	void ref_vel_Callback(const std_msgs::Float64::ConstPtr& ref_vel_sub){
	
		float norma_float = 1000000.0;

		ref_vel_[0] = ref_vel_sub->data;
		ref_vel_[1] = ref_vel_[0];
		ref_vel_[2] = ref_vel_[0];
		ref_vel_[3] = ref_vel_[0];
		ref_vel_[4] = ref_vel_[0];
		ref_vel_[5] = ref_vel_[0];
		
	
		buffer_in_[0] = (int)(ref_vel_[0]*norma_float);
		reverse_word(buffer_in_[0]);
		
		buffer_in_[1] = (int)(ref_vel_[1]*norma_float);
		reverse_word(buffer_in_[1]);

		buffer_in_[2] = (int)(ref_vel_[2]*norma_float);
		reverse_word(buffer_in_[2]);

		buffer_in_[3] = (int)(ref_vel_[3]*norma_float);
		reverse_word(buffer_in_[3]);

		buffer_in_[4] = (int)(ref_vel_[4]*norma_float);
		reverse_word(buffer_in_[4]);

		buffer_in_[5] = (int)(ref_vel_[5]*norma_float);
		reverse_word(buffer_in_[5]);
		// gripper
		
		buffer_in_[6] = (int)(ref_vel_[6]*norma_float);
		reverse_word(buffer_in_[6]);
		buffer_in_[7] = (int)(ref_vel_[7]*norma_float);
		reverse_word(buffer_in_[7]);
		send(new_socket_, buffer_in_, 32, 0);

	}

};

int main(int argc, char **argv){ 	
	ROS_WARN("Init Interface ur3");
	// primeira coisa:
	// tem que enviar o arquivo urscript
	ROS_WARN("Wainting for ur3 response ...");
	send_script(); // a função send_script envia o arquivo para o robô
	ROS_WARN("Send script to ur3 ...");
	int new_socket = open_socket();
	// abrindo a comunicaçção tcp socket
	//ROS
	ros::init(argc, argv, "ur3");	
	ros::NodeHandle node;
	Interface interface(node);

  	ROS_INFO("ur3 node is running");
  	ros::spin();
  	return 0;
} 

