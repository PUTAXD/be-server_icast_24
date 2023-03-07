#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <refbox/Message.h>

using namespace std;

void CllbackMsgRefbox(const refbox::Message::ConstPtr &msg)
{
    ROS_INFO("command: [%s]", msg->command.c_str());
    ROS_INFO("target_team: [%s]", msg->target_team.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recv_bs");
    ros::NodeHandle nh;

    ros::Subscriber sub_refbox = nh.subscribe("refbox_msg", 1000, CllbackMsgRefbox);

    ros::spin();
    return 0;
}
