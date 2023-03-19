#include "ros/ros.h"
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
// #include <nlohmann/json.hpp>
#include <refbox/Message.h>

// using json = nlohmann::json;

using boost::asio::ip::tcp;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "refbox_node");
  ros::NodeHandle nh;

  ros::Publisher pub_msg = nh.advertise<refbox::Message>("refbox_msg", 1000);
  try
  {
    boost::asio::io_context io_context;

    tcp::resolver resolver(io_context);
    tcp::resolver::results_type endpoints =
        resolver.resolve("127.0.0.1", "28097");

    tcp::socket socket(io_context);
    boost::asio::connect(socket, endpoints);

    while (ros::ok())
    {
      refbox::Message msg;
      boost::array<char, 128> buf;
      boost::system::error_code error;

      size_t len = socket.read_some(boost::asio::buffer(buf), error);

      if (error == boost::asio::error::eof)
      {
        break; // Connection closed cleanly by peer.
      }
      else if (error)
      {
        throw boost::system::system_error(error); // Some other error.
      }

      // std::cout.write(buf.data(), len);
      std::string message(buf.data(), len);
      message = message.substr(0, message.size() - 1);
      // json ms_ref = json::parse(message);

      // msg.command = ms_ref["command"];
      // msg.target_team = ms_ref["targetTeam"];
      msg.status = 1;

      pub_msg.publish(msg);
    }
  }
  catch (std::exception &e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
