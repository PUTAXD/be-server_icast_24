#include "ros/ros.h"
#include "talker_listener/Message.h"

void chatterCallback(const talker_listener::Message::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->first_name.c_str());
  ROS_INFO("I heard: [%s]", msg->last_name.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("topic_chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}