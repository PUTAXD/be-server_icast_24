//--ROS Headers
#include <ros/package.h>
#include <ros/ros.h>

//--Include Libraries
#include <icast.hpp>
#include <icast_type.h>

//--ROS Messages
#include "communications/bs2server.h"
#include "communications/server2bs.h"

//--Publisher
ros::Publisher pub_server2bs;

//--Subscriber
ros::Subscriber sub_bs2server;

//--Timer
ros::Timer tim_routine;

//--Singleton Initialization
Icast* icast = Icast::getInstance();

//--Prototypes
void timeCallback(const ros::TimerEvent& event);
void setDataToBeSend(Dictionary* dc_ptr);
void bsToServerCallback(const communications::bs2server::ConstPtr& msg);

//-Global Variables
command_t command_bs;
pos_t my_robot_pose;
pos_t agent_robot_pose;

communications::server2bs msg_server2bs;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);

    pub_server2bs = nh.advertise<communications::server2bs>("server2bs", 1);

    char config_file[100];
    std::string current_dir = ros::package::getPath("communications");
    sprintf(config_file, "%s/../../", current_dir.c_str());
    printf("%s\n", config_file);
    icast->init(config_file);

    if (!icast->mc->initialized()) {
        std::cout << "Multicast not ready" << std::endl;
        ros::shutdown();
        return 0;
    }

    tim_routine = nh.createTimer(ros::Duration(0.01), timeCallback);

    spinner.spin();

    return 0;
}

void timeCallback(const ros::TimerEvent& event)
{

    icast->update();

    size_t offset, size;
    icast->dc->getOffsetSize(3, "pos", offset, size);
    memcpy(&my_robot_pose, icast->dc->dictionary_data_.data() + offset, size);
    pub_server2bs.publish(msg_server2bs);
}

void setDataToBeSend(Dictionary* dc_ptr)
{
    size_t offset, size;
    dc_ptr->getOffsetSize(1, "pos", offset, size);

    agent_robot_pose.x = 900.0;
    agent_robot_pose.y = 600.0;
    agent_robot_pose.theta = 90;

    std::memcpy(dc_ptr->dictionary_data_.data() + offset, &agent_robot_pose, size);

    dc_ptr->setResetUpdate(1, "pos", false, true);
}

void bsToServerCallback(const communications::bs2server::ConstPtr& msg)
{
    command_bs.data = msg->command;
}