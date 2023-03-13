#include "ros/ros.h"
#include "talker_listener/Message.h"
#include "talker_listener/Callback.h"

void chatterCallback(const talker_listener::Message::ConstPtr &msg)
{
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
  tim_pub = n.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent &event)
                          {
    counter++;
    talker_listener::Callback msg;
    msg.number1 = counter % 200;
    msg.number2 = counter + 10 % 215;
    msg.number3 = counter + 111 % 100;

    pub.publish(msg); });

  spinner.spin();
  return 0;
}