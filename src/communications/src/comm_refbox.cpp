#include <stdio.h>
#include "ros/ros.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "comm_refbox");
    ros::NodeHandle n;

    printf("Hello World");
    return 0;
}