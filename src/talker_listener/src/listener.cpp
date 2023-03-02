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
  ros::MultiThreadedSpinner spinner(4);
  ros::Timer tim_pub;

  ros::Subscriber sub = n.subscribe("topic_chatter", 1000, chatterCallback);

  ros::Publisher pub = n.advertise<talker_listener::Callback>("pub_topic_listener", 1000);
  ros::Rate loop_rate(10);

  int counter = 0;
  // while (ros::ok())
  // {
  //   counter++;
  //   talker_listener::Callback msg;
  //   msg.number1 = counter % 200;
  //   msg.number2 = counter + 10 % 215;
  //   msg.number3 = counter + 111 % 100;

  //   ROS_INFO("%d", msg.number1);
  //   pub.publish(msg);
  // }

  tim_pub = n.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent& event) {
    counter++;
    talker_listener::Callback msg;
    msg.number1 = counter % 200;
    msg.number2 = counter + 10 % 215;
    msg.number3 = counter + 111 % 100;

    ROS_INFO("%d", msg.number1);
    ROS_INFO("%d", msg.number2);
    ROS_INFO("%d", msg.number3);
    printf("\n");
    pub.publish(msg);
  });


  

  // ros::spin();
  spinner.spin();

  return 0;
}