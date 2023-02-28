#include "ros/ros.h"
#include "std_msgs/String.h"
#include "talker_listener/Message.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("topic_chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    // std_msgs::String msg2;
    std_msgs::String msg;
    // back-end::Message msg;

    std::stringstream ss;
    // std::stringstream ss2;
    ss << "hello world " << count;
    msg.data = ss.str();
    // msg.first_name = ss.str();
    // ss2 << "i am tested " << count + 2;
    // msg.last_name = ss2.str();
    // msg.age = (count * 2) % 200;
    // msg.score = count * 2;


    ROS_INFO("%s", msg.data.c_str());
    // ROS_INFO("%s", msg.first_name.c_str());
    // ROS_INFO("%s", msg.first_name.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}