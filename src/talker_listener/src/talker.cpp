#include "ros/ros.h"
#include "talker_listener/Message.h"
#include "talker_listener/Callback.h"

#include <sstream>

void subCallback(const talker_listener::Callback::ConstPtr& msg){}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_talker");
  ros::NodeHandle n;
  ros::Timer tim_pub;
  
  ros::Subscriber sub = n.subscribe<talker_listener::Callback>("pub_topic_listener",1000, subCallback);
  ros::MultiThreadedSpinner spinner(4);

  ros::Publisher chatter_pub = n.advertise<talker_listener::Message>("topic_chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  tim_pub = n.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent& event) {
    talker_listener::Message msg2;

    std::stringstream ss;
    std::stringstream ss2;
    ss << "hello world " << count;
    msg2.first_name = ss.str();
    ss2 << "i am tested " << count + 2;
    msg2.last_name = ss2.str();
    msg2.age = (count * 2) % 200;
    msg2.score = count * 2;

    ROS_INFO("%s", msg2.first_name.c_str());
    ROS_INFO("%s", msg2.last_name.c_str());

    chatter_pub.publish(msg2);

    ++count;
  });

  spinner.spin();
  return 0;
}