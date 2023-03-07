#include "ros/ros.h"
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <nlohmann/json.hpp>
#include <refbox/Message.h>

using namespace std;
using json = nlohmann::json;

#define PORT 28097

int main(int argc, char **argv)
{
  ros::init(argc, argv, "refbox_node");
  ros::NodeHandle nh;

  uint16_t server_port = 28097;

  int stat, valread, client_fd;
  struct sockaddr_in serv_addr;
  char buffer[1024] = {0};

  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(server_port);

  ros::Publisher pub_msg = nh.advertise<refbox::Message>("refbox_msg", 1000);

  if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    ROS_INFO("\n Socket creation error \n");
    return -1;
  }

  if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
  {
    ROS_INFO(
        "\nInvalid address/ Address not supported \n");
    return -1;
  }

  if ((stat = connect(client_fd, (struct sockaddr *)&serv_addr,
                      sizeof(serv_addr))) < 0)
  {
    ROS_INFO("\nConnection Failed \n");
    return -1;
  }

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    refbox::Message msg;
    valread = read(client_fd, buffer, 1024);
    if (valread <= 0)
    {
      connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
      ROS_INFO("Connection closed\n");
      msg.command = "STOP";
      msg.target_team = "";
      msg.status = 0;
    }
    else
    {
      json j = json::parse(buffer);
      msg.command = j["command"];
      msg.target_team = j["targetTeam"];
      msg.status = 1;
      ROS_INFO("%s\n", buffer);
    }

    pub_msg.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }

  close(client_fd);
  return 0;
}
