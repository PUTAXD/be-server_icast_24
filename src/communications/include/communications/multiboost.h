#include <iostream>
#include <ros/ros.h>
#include <array>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <thread>
#include "communications/PC2BS.h"
#include "communications/BS2PC.h"

using boost::asio::ip::udp;

#define N_ROBOT 5
#define LEN_MSG 256

const std::string mtcast_address = "224.16.32.82";
constexpr short multicast_port = 1027;
const int max_ttl = 2;

void cllbckRcvMtcast(char *rcv_buf_);
void cllbckSndMtcast(const communications::BS2PC::ConstPtr &msg);

class receiver
{
public:
    receiver(boost::asio::io_context &io_context)
        : socket_(io_context)
    {
        // Create the socket so that multiple may be bound to the same address.
        boost::asio::ip::udp::endpoint multicast_endpoint(
            boost::asio::ip::address::from_string(mtcast_address), multicast_port);
        socket_.open(multicast_endpoint.protocol());
        socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
        socket_.bind(multicast_endpoint);
        socket_.non_blocking(true);

        // Join the multicast group.
        socket_.set_option(boost::asio::ip::multicast::join_group(
            boost::asio::ip::address::from_string(mtcast_address)));
        socket_.set_option(boost::asio::ip::multicast::hops(max_ttl));
        socket_.set_option(boost::asio::ip::multicast::enable_loopback(false));

        do_receive();
    }

private:
    void do_receive()
    {
        socket_.async_receive_from(
            boost::asio::buffer(data_), sender_endpoint_,
            [this](boost::system::error_code ec, std::size_t length)
            {
                if (!ec)
                {
                    cllbckRcvMtcast(data_.data());
                    do_receive();
                }
            });
    }

    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint sender_endpoint_;
    std::array<char, 1024> data_;
};

class sender
{
public:
    sender(boost::asio::io_context &io_context,
           const boost::asio::ip::address &multicast_address)
        : endpoint_(multicast_address, multicast_port),
          socket_(io_context, endpoint_.protocol()),
          timer_(io_context)
    {
    }

    void do_send(char *msg, int len)
    {
        socket_.async_send_to(
            boost::asio::buffer(msg, len), endpoint_,
            [this](boost::system::error_code ec, std::size_t /*length*/) {
            });
    }

private:
    void do_timeout()
    {
        timer_.expires_after(std::chrono::milliseconds(25));
        timer_.async_wait(
            [this](boost::system::error_code ec) {
            });
    }

private:
    boost::asio::ip::udp::endpoint endpoint_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::steady_timer timer_;
    size_t message_count_ = 0;
};
