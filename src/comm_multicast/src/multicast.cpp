#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"

struct sockaddr_in localSock;
struct ip_mreq group;
int sd;
int datalen;
char databuf[1024];

struct sockaddr src_addr;
socklen_t addr_len = sizeof(src_addr);

int main(int argc, char *argv[]){
    ros::init(argc, argv, "multicast");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);
    ros::Timer timer_cllbck;

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sd < 0){
        perror("Opening datagram socket error");
        exit(1);
    }else{
        printf("Opening the datagram socket...OK.\n");
    }

    int reuse = 1;

    if(setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(reuse)) < 0){
        perror("Setting SO_REUSEADDR error");
        close(sd);
        exit(1);
    }else{
        printf("Setting SO_REUSEADDR...OK.\n");
    }
 
    memset((char *) &localSock, 0, sizeof(localSock));
    localSock.sin_family = AF_INET;
    localSock.sin_port = htons(1026);
    localSock.sin_addr.s_addr = INADDR_ANY;
    if(bind(sd, (struct sockaddr*)&localSock, sizeof(localSock)))
    {
        perror("Binding datagram socket error");
        close(sd);
        exit(1);
    }else{
        printf("Binding datagram socket...OK.\n");
    }

    /* datagrams are to be received. */

    group.imr_multiaddr.s_addr = inet_addr("224.16.32.80");
    group.imr_interface.s_addr = inet_addr("0.0.0.0");
    if(setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0){
        perror("Adding multicast group error");
        close(sd);
        exit(1);
    }else{
        printf("Adding multicast group...OK.\n");
    }

    timer_cllbck = n.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent& event){
    /* Read from the socket. */
        char recv_buf[64];
        uint8_t nrecv = recvfrom(sd, recv_buf, 64, MSG_DONTWAIT, &src_addr, &addr_len);
        if(nrecv > 0 && nrecv < 255) {
            ROS_INFO("%s\n", recv_buf);
        }
    });


    spinner.spin();

    return 0;
}