/**
 * September, 2022
 *
 * Reference:
 * https://tldp.org/HOWTO/Multicast-HOWTO-6.html
 * https://man7.org/linux/man-pages/man2/recv.2.html
 */

#include <ros/ros.h>
#include <ros/package.h>

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
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <vector>

/* Define pre-processor functions */
#define PERR(txt, par...) \
    printf("ERROR: (%s / %s): " txt "\n", __FILE__, __FUNCTION__, ##par)
#define PERRNO(txt) \
    printf("ERROR: (%s / %s): " txt ": %s\n", __FILE__, __FUNCTION__, strerror(errno))

#define RAD2DEG 57.295780
#define DEG2RAD 0.017452925

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

ros::Timer recv_timer;
ros::Timer send_timer;

ros::Publisher pub_bs2pc;
ros::Publisher pub_sh_robot_data;

ros::Subscriber sub_stm2pc;
ros::Subscriber sub_mc_out;
ros::Subscriber sub_obs_data;
ros::Subscriber sub_icp_data;

// PC2MC_out datas
int16_t pos_x;
int16_t pos_y;
int16_t theta;
int8_t ball_status;
int16_t ball_x;
int16_t ball_y;
uint16_t robot_cond;
int8_t pass_target;
uint8_t target_on_field;
float battery;
int16_t ball_x_next;
int16_t ball_y_next;
int16_t robot_x_next;
int16_t robot_y_next;

// UDP data
char send_buf[512];
uint16_t actual_data_size;
char its[4] = "its";

/* Shared data robots */
uint8_t sh_data_valid[6];
int16_t sh_pos_x[6];
int16_t sh_pos_y[6];
int16_t sh_theta[6];
uint16_t sh_robot_cond[6];
uint16_t sh_obs_r_final[6][144];
uint16_t sh_obs_angle_final[6][144];
uint8_t sh_total_obs[6];
uint8_t sh_target_on_field[6];

/* Robot's logic if it online or not */
uint8_t delay_tolerance = 3;
int64_t robot_epoch[6];

// Obs processing

uint16_t obs_on_field[145];
uint16_t obs_r_final[144];
uint8_t obs_angle_final[144];
uint8_t total_obs = 0;
int16_t keeper_on_field[2];
std::vector<uint16_t> dumped_obs_on_field;
uint16_t obs_r_final_dumped[144];
uint8_t obs_angle_final_dumped[144];
uint8_t total_obs_dumped = 0;

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
int openSocket()
{
    struct sockaddr_in multicastAddress;
    struct ip_mreqn mreqn;
    struct ip_mreq mreq;
    int opt;
    char address[16]; // IPV4: xxx.xxx.xxx.xxx/0

    /* It use to receive data */
    bzero(&multicastAddress, sizeof(struct sockaddr_in));
    multicastAddress.sin_family = AF_INET;
    multicastAddress.sin_port = htons(nw_config.port);
    multicastAddress.sin_addr.s_addr = INADDR_ANY;

    /* It use to send data */
    bzero(&multiSocket.destAddress, sizeof(struct sockaddr_in));
    multiSocket.destAddress.sin_family = AF_INET;
    multiSocket.destAddress.sin_port = htons(nw_config.port);
    multiSocket.destAddress.sin_addr.s_addr = inet_addr(nw_config.multicast_ip);

    /* Set socket as Datagram (UDP) */
    if ((multiSocket.socketID = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        PERRNO("socket");
        return -1;
    }

    /* Assign WiFi Interface to use multicast */
    memset((void *)&mreqn, 0, sizeof(mreqn));
    mreqn.imr_ifindex = if_NameToIndex(nw_config.iface, address);
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
    mreq.imr_multiaddr.s_addr = inet_addr(nw_config.multicast_ip);
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

void loadConfig()
{
    char *robot_num = getenv("ROBOT");
    char config_file[100];
    std::string current_dir = ros::package::getPath("comm");
    sprintf(config_file, "%s/../../config/IRIS%s/static_conf.yaml", current_dir.c_str(), robot_num);
    printf("config file: %s\n", config_file);

    YAML::Node config = YAML::LoadFile(config_file);
    strcpy(nw_config.identifier, robot_num);
    strcpy(nw_config.iface, config["Multicast"]["iface"].as<std::string>().c_str());
    strcpy(nw_config.multicast_ip, config["Multicast"]["multicast_ip"].as<std::string>().c_str());
    nw_config.port = config["Multicast"]["port"].as<int>();

    printf("identifier: %s\n", nw_config.identifier);
    printf("iface: %s\n", nw_config.iface);
    printf("multicast_ip: %s\n", nw_config.multicast_ip);
    printf("port: %d\n", nw_config.port);
}

/* This will get data from master */
void cllbck_mc_out(const iris_msgs::mc_outConstPtr &msg)
{
    pos_x = msg->pos_x;
    pos_y = msg->pos_y;
    theta = msg->theta;
    ball_status = msg->ball_status;
    ball_x = msg->ball_x;
    ball_y = msg->ball_y;
    robot_cond = msg->robot_cond;
    pass_target = msg->pass_target;
    target_on_field = msg->target_on_field;
    ball_x_next = msg->ball_x_next;
    ball_y_next = msg->ball_y_next;
    robot_x_next = msg->pos_x_next;
    robot_y_next = msg->pos_y_next;
}

/* This will get data from vision */
void cllbck_sub_obstacle(const iris_msgs::VisionConstPtr &msg)
{
    total_obs_dumped = 0;
    memcpy(obs_on_field, &msg->obs_on_field[0], 288);
    dumped_obs_on_field = msg->dumped_obs_on_field;

    for (uint8_t i = 0; i < dumped_obs_on_field.size(); i++)
    {
        obs_r_final_dumped[i] = dumped_obs_on_field[i];
        obs_angle_final_dumped[i] = i;
        total_obs_dumped++;
    }

    keeper_on_field[0] = (int16_t)msg->keeper_on_field_x;
    keeper_on_field[1] = (int16_t)msg->keeper_on_field_y;
}

/* Get data from stm */
void cllbck_sub_stm2pc(const iris_msgs::stm2pcConstPtr &msg)
{
    battery = msg->battery;
}

void recv_cllbck(const ros::TimerEvent &)
{
    char recv_buf[64];

    /* Use MSG_DONTWAIT because of non-blocking */
    int8_t nrecv = recvfrom(recv_socket->socketID, recv_buf, 64, MSG_DONTWAIT, &src_addr, &addr_len);

    if (nrecv > 0 && (recv_buf[3] >= '0' && recv_buf[3] <= '5') && recv_buf[0] == 'i')
    {
        /**
         * 0 is from Basestation
         * 1-5 is from robot based on his number
         */
        int8_t identifier = recv_buf[3] - '0';
        // Recv from Basestation
        if (identifier == 0)
        {
            iris_msgs::mc_in msg_mc_to_pc;
            memcpy(&msg_mc_to_pc.base, recv_buf + 4, 1);
            memcpy(&msg_mc_to_pc.command, recv_buf + 5, 1);
            memcpy(&msg_mc_to_pc.style, recv_buf + 6, 1);
            memcpy(&msg_mc_to_pc.ball_x_field, recv_buf + 7, 2);
            memcpy(&msg_mc_to_pc.ball_y_field, recv_buf + 9, 2);
            memcpy(&msg_mc_to_pc.manual_x, recv_buf + 11, 2);
            memcpy(&msg_mc_to_pc.manual_y, recv_buf + 13, 2);
            memcpy(&msg_mc_to_pc.manual_th, recv_buf + 15, 2);
            memcpy(&msg_mc_to_pc.offset_x, recv_buf + 17, 2);
            memcpy(&msg_mc_to_pc.offset_y, recv_buf + 19, 2);
            memcpy(&msg_mc_to_pc.offset_th, recv_buf + 21, 2);
            memcpy(&msg_mc_to_pc.data_mux1, recv_buf + 23, 2);
            memcpy(&msg_mc_to_pc.data_mux2, recv_buf + 25, 2);
            memcpy(&msg_mc_to_pc.mux_control, recv_buf + 27, 2);
            memcpy(&msg_mc_to_pc.trans_vel_trim, recv_buf + 29 + atoi(nw_config.identifier) - 1, 1);
            memcpy(&msg_mc_to_pc.rotary_vel_trim, recv_buf + 34 + atoi(nw_config.identifier) - 1, 1);
            memcpy(&msg_mc_to_pc.kick_power_trim, recv_buf + 39 + atoi(nw_config.identifier) - 1, 1);
            memcpy(&msg_mc_to_pc.pass_counter, recv_buf + 44, 1);

            pub_bs2pc.publish(msg_mc_to_pc);
        }
        // Recv from another robots
        else
        {
            // Update other robots data_buffer
            int64_t epoch_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            // memcpy(robot_epoch + identifier, recv_buf + 4, 8);
            memcpy(sh_pos_x + identifier, recv_buf + 12, 2);
            memcpy(sh_pos_y + identifier, recv_buf + 14, 2);
            memcpy(sh_theta + identifier, recv_buf + 16, 2);
            memcpy(sh_robot_cond + identifier, recv_buf + 23, 2);
            memcpy(sh_target_on_field + identifier, recv_buf + 26, 1);
            memcpy(sh_total_obs + identifier, recv_buf + 27, 1);

            static uint16_t counter = 28;
            counter = 28;

            for (int i = 1; i < 6; i++)
            {
                for (int j = 0; j < sh_total_obs[i]; j++)
                {
                    memcpy(sh_obs_r_final[i] + j, recv_buf + counter, 2);
                    counter += 2;
                    memcpy(sh_obs_angle_final[i] + j, recv_buf + counter, 2);
                    counter += 2;
                }
                counter = 28;
            }

            robot_epoch[identifier] = epoch_now;

            // Update other robots data_valid
            sh_data_valid[1] = (abs(epoch_now - robot_epoch[1]) <= delay_tolerance);
            sh_data_valid[2] = (abs(epoch_now - robot_epoch[2]) <= delay_tolerance);
            sh_data_valid[3] = (abs(epoch_now - robot_epoch[3]) <= delay_tolerance);
            sh_data_valid[4] = (abs(epoch_now - robot_epoch[4]) <= delay_tolerance);
            sh_data_valid[5] = (abs(epoch_now - robot_epoch[5]) <= delay_tolerance);

            // Sending datas to master node
            iris_msgs::mc_in_rbts msg_mc_in_rbts;
            msg_mc_in_rbts.data_valid.assign(sh_data_valid, sh_data_valid + 6);
            msg_mc_in_rbts.pos_x.assign(sh_pos_x, sh_pos_x + 6);
            msg_mc_in_rbts.pos_y.assign(sh_pos_y, sh_pos_y + 6);
            msg_mc_in_rbts.theta.assign(sh_theta, sh_theta + 6);
            msg_mc_in_rbts.robot_cond.assign(sh_robot_cond, sh_robot_cond + 6);
            msg_mc_in_rbts.target_on_field.assign(sh_target_on_field, sh_target_on_field + 6);
            msg_mc_in_rbts.obs_total.assign(sh_total_obs, sh_total_obs + 6);
            msg_mc_in_rbts.obs_r_final_1.assign(sh_obs_r_final[1], sh_obs_r_final[1] + sh_total_obs[1]);
            msg_mc_in_rbts.obs_r_final_2.assign(sh_obs_r_final[2], sh_obs_r_final[2] + sh_total_obs[2]);
            msg_mc_in_rbts.obs_r_final_3.assign(sh_obs_r_final[3], sh_obs_r_final[3] + sh_total_obs[3]);
            msg_mc_in_rbts.obs_r_final_4.assign(sh_obs_r_final[4], sh_obs_r_final[4] + sh_total_obs[4]);
            msg_mc_in_rbts.obs_r_final_5.assign(sh_obs_r_final[5], sh_obs_r_final[5] + sh_total_obs[5]);
            msg_mc_in_rbts.obs_angle_final_1.assign(sh_obs_angle_final[1], sh_obs_angle_final[1] + sh_total_obs[1]);
            msg_mc_in_rbts.obs_angle_final_2.assign(sh_obs_angle_final[2], sh_obs_angle_final[2] + sh_total_obs[2]);
            msg_mc_in_rbts.obs_angle_final_3.assign(sh_obs_angle_final[3], sh_obs_angle_final[3] + sh_total_obs[3]);
            msg_mc_in_rbts.obs_angle_final_4.assign(sh_obs_angle_final[4], sh_obs_angle_final[4] + sh_total_obs[4]);
            msg_mc_in_rbts.obs_angle_final_5.assign(sh_obs_angle_final[5], sh_obs_angle_final[5] + sh_total_obs[5]);
            pub_sh_robot_data.publish(msg_mc_in_rbts);
        }
    }
}

void send_cllbck(const ros::TimerEvent &)
{
    // Get current unix epoch time
    int64_t epoch = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    obsFilter();

    // Creating data-frame
    memcpy(send_buf, its, 3);
    memcpy(send_buf + 3, nw_config.identifier, 1);
    memcpy(send_buf + 4, &epoch, 8);
    memcpy(send_buf + 12, &pos_x, 2);
    memcpy(send_buf + 14, &pos_y, 2);
    memcpy(send_buf + 16, &theta, 2);
    memcpy(send_buf + 18, &ball_status, 1);
    memcpy(send_buf + 19, &ball_x, 2);
    memcpy(send_buf + 21, &ball_y, 2);
    memcpy(send_buf + 23, &robot_cond, 2);
    memcpy(send_buf + 25, &pass_target, 1);
    memcpy(send_buf + 26, &target_on_field, 1);
    memcpy(send_buf + 27, &total_obs, 1);

    static uint16_t counter = 28;
    counter = 28;
    for (int i = 0; i < total_obs; i++)
    {
        memcpy(send_buf + counter, &obs_r_final[i], 2);
        counter += 2;
        memcpy(send_buf + counter, &obs_angle_final[i], 1);
        counter++;
    }
    memcpy(send_buf + counter, &battery, 4);
    counter += 4;

    memcpy(send_buf + counter, keeper_on_field, 4);
    counter += 4;

    memcpy(send_buf + counter, &ball_x_next, 2);
    counter += 2;
    memcpy(send_buf + counter, &ball_y_next, 2);
    counter += 2;

    memcpy(send_buf + counter, &robot_x_next, 2);
    counter += 2;
    memcpy(send_buf + counter, &robot_y_next, 2);
    counter += 2;

    // for (uint8_t i = 0; i < total_obs_dumped; i++)
    // {
    //     memcpy(send_buf + counter, &obs_r_final_dumped[i], 2);
    //     counter += 2;
    //     memcpy(send_buf + counter, &obs_angle_final_dumped[i], 1);
    //     counter++;
    // }

    // Set data actual size
    actual_data_size = counter;

    // Send over UDP multicast
    uint16_t nsent = sendto(multiSocket.socketID, send_buf, actual_data_size, 0, (struct sockaddr *)&multiSocket.destAddress, sizeof(struct sockaddr));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner spinner(0);

    loadConfig();

    memset(sh_data_valid, 0, 6);
    memset(sh_pos_x, 0, 6);
    memset(sh_pos_y, 0, 6);
    memset(sh_theta, 0, 6);
    memset(sh_robot_cond, 0, 6);

    closeSocket();
    if (openSocket() == -1)
    {
        PERR("openMulticastSocket");
        return -1;
    }

    recv_socket = &multiSocket;

    send_buf[0] = 'q'; // Dummy data, indicates that this node doesn't get a valid data from robot
    send_buf[3] = '9'; // Dummy data, indicates that this node doesn't get a valid data from robot

    pub_bs2pc = NH.advertise<iris_msgs::mc_in>("bs2pc", 4);

    pub_sh_robot_data = NH.advertise<iris_msgs::mc_in_rbts>("sh_robot_data", 5);

    recv_timer = NH.createTimer(ros::Duration(0.01), recv_cllbck);
    send_timer = NH.createTimer(ros::Duration(0.05), send_cllbck);
    sub_mc_out = NH.subscribe("mc_out", 1, cllbck_mc_out);
    sub_obs_data = NH.subscribe("vision_data", 1, cllbck_sub_obstacle);
    sub_stm2pc = NH.subscribe("stm2pc", 1, cllbck_sub_stm2pc);

    spinner.spin();

    recv_timer.stop();
    send_timer.stop();

    return 0;
}