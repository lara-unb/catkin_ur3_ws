#include "ros/ros.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>

int send_script(){
    std::string robot_ip;
    int robot_port;
    ros::param::get("~robot_ip", robot_ip);
    ros::param::get("~robot_port", robot_port);
    
    ROS_WARN("%s",("ur3 is setted in ip " + robot_ip).c_str());
    ROS_WARN("%s",("ur3 is setted in port " + std::to_string(robot_port)).c_str());
  
    int sfd =0, n=0, b;
    char rbuff[256];
    char sendbuffer[256];

    struct sockaddr_in serv_addr;

    memset(rbuff, '0', sizeof(rbuff));
    sfd = socket(AF_INET, SOCK_STREAM, 0);

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(robot_port);
    serv_addr.sin_addr.s_addr = inet_addr(robot_ip.c_str()); // real 

    b=connect(sfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    if (b==-1) {
        perror("Connect");
        return 1;
    }


    //printf("Waiting 5 secods ...");
    // resetar o robô
    FILE *fp1 = fopen("prog_reset.script", "rb");
    if(fp1 == NULL){
        perror("File");
        return 2;
    }

    while( (b = fread(sendbuffer, 1, sizeof(sendbuffer), fp1))>0 ){
      send(sfd, sendbuffer, b, 0);
    
    }

    fclose(fp1);
    
    sleep(5);
    // manda o arquivo que será esxecutado
    FILE *fp = fopen("ur3_arm.script", "rb");
    if(fp == NULL){
        perror("File");
        return 2;
    }

    while( (b = fread(sendbuffer, 1, sizeof(sendbuffer), fp))>0 ){
        send(sfd, sendbuffer, b, 0);
        // zmq_send (requester, sendbuffer, b, 0);
        // printf("%i\n",b);
    }

    fclose(fp);
    return 0;

}