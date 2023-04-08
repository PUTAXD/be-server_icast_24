#include "communications/multicast.h"
#include "communications/PC2BS.h"
#include "communications/BS2PC.h"

/////////////
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <fstream>
#include <sys/time.h>
#include <unistd.h>
#include <sstream>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#include <linux/if_ether.h>
// #include "yaml-cpp/yaml.h"
#include <chrono>
#include <vector>

#define N_ROBOT 5
#define LEN_MSG 256

/* Define pre-processor functions */
#define PERR(txt, par...) \
    printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ##par)
#define PERRNO(txt) \
    printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

communications::PC2BS pc2bs_msg;

ros::Subscriber bs2pc_sub;
ros::Publisher pc2bs_pub[N_ROBOT];

////////////////
// Socket
typedef struct multiSocket_tag
{
    struct sockaddr_in destAddress;
    int socketID;
} multiSocket_t;
typedef struct nw_config
{
    char multicast_ip[16];
    char iface[10];
    char identifier[1];
    unsigned int port;
    uint8_t compress_type;
} config;
multiSocket_t multiSocket;
multiSocket_t *recv_socket;
struct sockaddr src_addr;
socklen_t addr_len = sizeof(src_addr);

// Config
config nw_config;

/**
 * This function is to search index of interface in linux,
 * So, we not use IP, we just use Wifi-Interface
 * Use "iwconfig" linux command
 *
 * @return IF_index
 */
int if_NameToIndex(char *ifname, char *address)
{
    int fd;
    struct ifreq if_info;
    int if_index;

    memset(&if_info, 0, sizeof(if_info));
    strncpy(if_info.ifr_name, ifname, IFNAMSIZ - 1);

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
        PERRNO("socket");
        return -1;
    }
    if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
    {
        PERRNO("ioctl");
        close(fd);
        return -1;
    }
    if_index = if_info.ifr_ifindex;

    if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
    {
        PERRNO("ioctl");
        close(fd);
        return -1;
    }

    close(fd);

    sprintf(address, "%d.%d.%d.%d\n",
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[2],
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[3],
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[4],
            (int)((unsigned char *)if_info.ifr_hwaddr.sa_data)[5]);
#ifdef COMM_DEBUG
    printf("**** Using device %s -> %s\n", if_info.ifr_name, address);
#endif

    return if_index;
}

/**
 * This function is to open multicast socket
 *
 * @return zero on success
 */
int openSocket2()
{
    struct sockaddr_in multicastAddress;
    struct ip_mreqn mreqn;
    struct ip_mreq mreq;
    int opt;
    char address[16]; // IPV4: xxx.xxx.xxx.xxx/0

    /* It use to receive data */
    bzero(&multicastAddress, sizeof(struct sockaddr_in));
    multicastAddress.sin_family = AF_INET;
    multicastAddress.sin_port = htons(1027);
    multicastAddress.sin_addr.s_addr = INADDR_ANY;

    /* It use to send data */
    bzero(&multiSocket.destAddress, sizeof(struct sockaddr_in));
    multiSocket.destAddress.sin_family = AF_INET;
    multiSocket.destAddress.sin_port = htons(1027);
    multiSocket.destAddress.sin_addr.s_addr = inet_addr("224.16.32.82");

    /* Set socket as Datagram (UDP) */
    if ((multiSocket.socketID = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        PERRNO("socket");
        return -1;
    }

    /* Assign WiFi Interface to use multicast */
    memset((void *)&mreqn, 0, sizeof(mreqn));
    mreqn.imr_ifindex = if_NameToIndex("wlp0s20f3", address);
    if ((setsockopt(multiSocket.socketID, SOL_IP, IP_MULTICAST_IF, &mreqn, sizeof(mreqn))) == -1)
    {
        PERRNO("setsockopt 1");
        return -1;
    }

    /* It allow to use more than one process that bind on the same UPD socket */
    opt = 1;
    if ((setsockopt(multiSocket.socketID, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) == -1)
    {
        PERRNO("setsockopt 2");
        return -1;
    }

    memset((void *)&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr.s_addr = inet_addr("224.16.32.82");
    mreq.imr_interface.s_addr = inet_addr(address);

    /* Assign socket to multicast IP, so he will collect all message on them */
    if ((setsockopt(multiSocket.socketID, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) == -1)
    {
        PERRNO("setsockopt 3");
        return -1;
    }

    /**
     * 0 for Disable reception of our own multicast
     * 1 for Receive our data
     * */
    opt = 0;
    if ((setsockopt(multiSocket.socketID, IPPROTO_IP, IP_MULTICAST_LOOP, &opt, sizeof(opt))) == -1)
    {
        PERRNO("setsockopt");
        return -1;
    }

    /* Bind to socket, so i can receive data  */
    if (bind(multiSocket.socketID, (struct sockaddr *)&multicastAddress, sizeof(struct sockaddr_in)) == -1)
    {
        PERRNO("bind");
        return -1;
    }

    return 0;
}

/* First time program is to close socket before open new socket */
void closeSocket()
{
    if (multiSocket.socketID != -1)
        shutdown(multiSocket.socketID, SHUT_RDWR);
}

void cllbckRcvMtcast(const ros::TimerEvent &event);
void cllbckSndMtcast(const communications::BS2PC::ConstPtr &msg);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);

    ros::Timer timer_cllbck_rcv;
    ros::Timer timer_cllbck_snd;

    // openSocket();

    closeSocket();
    if (openSocket2() == -1)
    {
        printf("Error open socket\n");
        ros::shutdown();
        return -1;
    }

    recv_socket = &multiSocket;

    timer_cllbck_rcv = n.createTimer(ros::Duration(0.005), cllbckRcvMtcast);

    bs2pc_sub = n.subscribe("bs2pc", 1000, cllbckSndMtcast);

    for (int i = 0; i < N_ROBOT; i++)
    {
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i + 1);
        pc2bs_pub[i] = n.advertise<communications::PC2BS>(str_topic, 1000);
    }

    spinner.spin();

    return 0;
}

void cllbckRcvMtcast(const ros::TimerEvent &event)
{
    char recv_buf[LEN_MSG];
    uint8_t nrecv = recvfrom(recv_socket->socketID, recv_buf, LEN_MSG, 0, &src_addr, &addr_len);

    if (nrecv > 0 && nrecv < 255)
    {
        ROS_INFO("rcv = > %s", recv_buf);
    }

    if ((nrecv > 0 && nrecv < 255) && (recv_buf[3] > '0' && recv_buf[3] <= '5') && (recv_buf[0] == 'i' && recv_buf[1] == 't' && recv_buf[2] == 's'))
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

    uint sent = sendto(multiSocket.socketID, send_buf, counter, 0, (struct sockaddr *)&multiSocket.destAddress, sizeof(struct sockaddr));
}