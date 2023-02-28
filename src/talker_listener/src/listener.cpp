#include "ros/ros.h"
#include "talker_listener/Message.h"
#include "talker_listener/Callback.h"

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

  ros::Publisher pub = n.advertise<talker_listener::Callback>("pub_topic_listener", 1000);
  ros::Rate loop_rate(10);

  int counter = 0;
  while (ros::ok())
  {
    counter++;
    talker_listener::Callback msg;
    msg.number1 = counter % 200;
    msg.number2 = counter + 10 % 215;
    msg.number3 = counter + 111 % 100;

    ROS_INFO("%d", msg.number1);
    pub.publish(msg);
  }
  

  ros::spin();

  return 0;
}