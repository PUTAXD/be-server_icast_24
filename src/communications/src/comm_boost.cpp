#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <cstring>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include "communications/multicast.h"
#include "communications/PC2BS.h"
#include "communications/BS2PC.h"

using boost::asio::ip::udp;

#define N_ROBOT 5
#define LEN_MSG 256

const std::string multicast_address = "224.16.32.80";
const unsigned short multicast_port = 1027;

void cllbckRcvMtcast(char *rcv_buf_);
void cllbckSndMtcast(const communications::BS2PC::ConstPtr &msg);

communications::PC2BS pc2bs_msg;

ros::Subscriber bs2pc_sub;
ros::Publisher pc2bs_pub[N_ROBOT];

boost::array<char, 256> recv_buffer_;
std::size_t nrecv = 0;

class udp_server
{
public:
    udp_server(boost::asio::io_context &io_context)
        : socket_(io_context)
    {
        udp::endpoint multicast_endpoint(
            boost::asio::ip::address::from_string(multicast_address),
            multicast_port);

        socket_.open(multicast_endpoint.protocol());
        socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
        socket_.bind(multicast_endpoint);

        socket_.set_option(boost::asio::ip::multicast::join_group(
            boost::asio::ip::address::from_string(multicast_address)));

        socket_.non_blocking(true);

        start_receive();
    }

private:
    void start_receive()
    {
        socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            [this](boost::system::error_code ec, std::size_t /*bytes_transferred*/)
            {
                handle_receive(ec);
            });
    }

    void handle_receive(const boost::system::error_code &error)
    {

        if (!error)
        {
            cllbckRcvMtcast(recv_buffer_.data());
            start_receive();
        }
    }

    void handle_send(boost::shared_ptr<std::string> /*message*/,
                     const boost::system::error_code & /*error*/,
                     std::size_t /*bytes_transferred*/)
    {
    }
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "comm_boost");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);

    ros::Timer timer_cllbck_rcv;

    boost::asio::io_context io_context;
    udp_server server(io_context);
    std::thread io_thread([&io_context]()
                          { io_context.run(); });

    bs2pc_sub = n.subscribe("bs2pc", 1, cllbckSndMtcast);

    for (int i = 0; i < N_ROBOT; i++)
    {
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i + 1);
        pc2bs_pub[i] = n.advertise<communications::PC2BS>(str_topic, 1);
    }

    spinner.spin();

    io_context.stop();
    io_thread.join();

    return 0;
}

void cllbckRcvMtcast(char *recv_buf)
{

    if ((recv_buf[3] > '0' && recv_buf[3] <= '5') && (recv_buf[0] == 'i' && recv_buf[1] == 't' && recv_buf[2] == 's'))
    {
        uint8_t n_robot = recv_buf[3] - '0';
        int counter = 4;
        int data_size = 0;

        data_size = sizeof(int64_t);
        memcpy(&pc2bs_msg.epoch, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.pos_x, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.pos_y, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.theta, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(uint8_t);
        memcpy(&pc2bs_msg.status_bola, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.bola_x, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.bola_y, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.robot_condition, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(uint8_t);
        memcpy(&pc2bs_msg.target_umpan, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(uint8_t);
        memcpy(&pc2bs_msg.index_point, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(uint8_t);
        memcpy(&pc2bs_msg.obs_length, recv_buf + counter, data_size);
        counter += data_size;

        int16_t obs_dist = 0;
        uint8_t obs_index = 0;
        pc2bs_msg.obs_dist.clear();
        pc2bs_msg.obs_index.clear();

        uint8_t obs_length = pc2bs_msg.obs_length;
        for (int i = 0; i < obs_length; i++)
        {
            data_size = sizeof(int16_t);
            memcpy(&obs_dist, recv_buf + counter, data_size);
            counter += data_size;

            data_size = sizeof(uint8_t);
            memcpy(&obs_index, recv_buf + counter, data_size);
            counter += data_size;

            pc2bs_msg.obs_dist.push_back(obs_dist);
            pc2bs_msg.obs_index.push_back(obs_index);
        }

        data_size = sizeof(float_t);
        memcpy(&pc2bs_msg.battery_health, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.goalkeeper_field_x, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.goalkeeper_field_y, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.ball_next_x, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.ball_next_y, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.robot_next_x, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.robot_next_y, recv_buf + counter, data_size);
        counter += data_size;

        pc2bs_msg.n_robot = n_robot;

        pc2bs_pub[n_robot - 1].publish(pc2bs_msg);
    }
}

void cllbckSndMtcast(const communications::BS2PC::ConstPtr &msg)
{
    int counter = 0;
    int data_size = 0;

    char send_buf[LEN_MSG];

    data_size = 4;
    memcpy(send_buf, "its0", data_size);
    counter += data_size;

    data_size = sizeof(int8_t);
    memcpy(send_buf + counter, &msg->header_manual_and_calibration, data_size);
    counter += data_size;

    data_size = sizeof(int8_t);
    memcpy(send_buf + counter, &msg->command, data_size);
    counter += data_size;

    data_size = sizeof(int8_t);
    memcpy(send_buf + counter, &msg->style, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->ball_x_in_field, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->ball_y_in_field, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->target_manual_x, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->target_manual_y, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->target_manual_theta, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->offset_robot_x, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->offset_robot_y, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->offset_robot_theta, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->mux1, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->mux2, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->mux_bs_control, data_size);
    counter += data_size;

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        data_size = sizeof(uint8_t);
        memcpy(send_buf + counter, &msg->control_v_linear[i], data_size);
        counter += data_size;
    }

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        data_size = sizeof(uint8_t);
        memcpy(send_buf + counter, &msg->control_v_angular[i], data_size);
        counter += data_size;
    }

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        data_size = sizeof(uint8_t);
        memcpy(send_buf + counter, &msg->control_power_kicker[i], data_size);
        counter += data_size;
    }

    data_size = sizeof(uint8_t);
    memcpy(send_buf + counter, &msg->passing_counter, data_size);
    counter += data_size;

    uint sent = sendto(sd, send_buf, counter, 0, (struct sockaddr *)&localSock, sizeof(struct sockaddr));
}