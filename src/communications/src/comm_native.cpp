#include "communications/multicast.h"
#include "communications/PC2BS.h"
#include "communications/BS2PC.h"
#include <signal.h>

#define N_ROBOT 5
#define LEN_MSG 256

communications::PC2BS pc2bs_msg;

ros::Subscriber bs2pc_sub;
ros::Publisher pc2bs_pub[N_ROBOT];

void cllbckRcvMtcast(const ros::TimerEvent &event);
void cllbckSndMtcast(const communications::BS2PC::ConstPtr &msg);

void signal_catch(int sig)
{
    if (sig == SIGINT)
    {
        closeSocket();
        usleep(50000);
        ros::shutdown();
    }
}

double time_kirim;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "comm_native");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);

    ros::Timer timer_cllbck_rcv;
    ros::Timer timer_cllbck_snd;

    openSocket();

    timer_cllbck_rcv = n.createTimer(ros::Duration(0.0000001), cllbckRcvMtcast);

    bs2pc_sub = n.subscribe("bs2pc", 1, cllbckSndMtcast);

    for (int i = 0; i < N_ROBOT; i++)
    {
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i + 1);
        pc2bs_pub[i] = n.advertise<communications::PC2BS>(str_topic, 1000);
    }

    signal(SIGINT, signal_catch);

    spinner.spin();

    return 0;
}

void cllbckRcvMtcast(const ros::TimerEvent &event)
{
    char recv_buf[LEN_MSG];
    int16_t nrecv = recvfrom(sd, recv_buf, LEN_MSG, 0, &src_addr, &addr_len);

    if (nrecv > 0 && (recv_buf[3] > '0' && recv_buf[3] <= '5') && (recv_buf[0] == 'i' && recv_buf[1] == 't' && recv_buf[2] == 's'))
    {
        uint8_t n_robot = recv_buf[3] - '0';
        double recv_time = ros::Time::now().toSec();

        int counter = 4;
        int data_size = 0;

        pc2bs_msg.mcast_delay = (uint64_t)((recv_time - time_kirim) * 1000000);

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

        // data_size = sizeof(int16_t);
        // memcpy(&pc2bs_msg.ball_next_x, recv_buf + counter, data_size);
        // counter += data_size;

        // data_size = sizeof(int16_t);
        // memcpy(&pc2bs_msg.ball_next_y, recv_buf + counter, data_size);
        // counter += data_size;

        // data_size = sizeof(int16_t);
        // memcpy(&pc2bs_msg.robot_next_x, recv_buf + counter, data_size);
        // counter += data_size;

        // data_size = sizeof(int16_t);
        // memcpy(&pc2bs_msg.robot_next_y, recv_buf + counter, data_size);
        // counter += data_size;

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

    static double prev_time_send = 0;
    uint sent = sendto(sd, send_buf, counter, 0, (struct sockaddr *)&localSock, sizeof(struct sockaddr));
    time_kirim = ros::Time::now().toSec();
    // ROS_ERROR("send: %lf -> %lf", time_kirim, time_kirim - prev_time_send);
    prev_time_send = time_kirim;
}