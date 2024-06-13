#include "ros/ros.h"
#include "communications/PC2BS.h"
#include "basestation/Collection.h"
#include <cstring>     // For memset
#include <arpa/inet.h> // For sockaddr_in and inet_addr
#include <unistd.h>    // For close

#define SERVER_PORT 10020
#define SERVER_IP "10.7.101.126"
#define BUFFER_SIZE 1024

typedef struct
{
    float pose_xyo[5][3];
    uint8_t n_robot_has_ball;
} data_robot_to_be_sent_t;

data_robot_to_be_sent_t data_buffer;
int sock;
struct sockaddr_in serverAddr;
char buffer[BUFFER_SIZE];
socklen_t addrLen = sizeof(serverAddr);

ros::Timer tim_send_to_redfbox;

ros::Subscriber sub_robot_collection;
ros::Subscriber sub_robot_pc2bs[5];

void callback_sub_robot_collection(const basestation::CollectionConstPtr &msg);
void callback_sub_robot_pc2bs(const communications::PC2BSConstPtr &msg);
void callback_routine(const ros::TimerEvent &event);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basestation");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(0);

    for (uint8_t i = 0; i < 5; i++)
    {
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i + 1);
        sub_robot_pc2bs[i] = n.subscribe(str_topic, 1, callback_sub_robot_pc2bs);
    }

    sub_robot_collection = n.subscribe("/collection", 1, callback_sub_robot_collection);

    tim_send_to_redfbox = n.createTimer(ros::Duration(0.1), callback_routine);

    // Create socket
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        std::cerr << "Socket creation failed" << std::endl;
        return -1;
    }

    // Setup server address structure
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);

    spinner.spin();

    return 0;
}

void callback_routine(const ros::TimerEvent &event)
{
    printf("data: %.2f %.2f %.2f || %d\n", data_buffer.pose_xyo[0][0], data_buffer.pose_xyo[0][1], data_buffer.pose_xyo[0][2], data_buffer.n_robot_has_ball);

    char data_send[BUFFER_SIZE];

    data_send[0] = 'm';
    data_send[1] = 's';
    data_send[2] = 'l';

    uint16_t offset = 3;

    for (uint8_t i = 0; i < 5; i++)
    {
        memcpy(data_send + offset, &data_buffer.pose_xyo[i][0], sizeof(float));
        offset += sizeof(float);
        memcpy(data_send + offset, &data_buffer.pose_xyo[i][1], sizeof(float));
        offset += sizeof(float);
        memcpy(data_send + offset, &data_buffer.pose_xyo[i][2], sizeof(float));
        offset += sizeof(float);
    }
    memcpy(data_send + offset, &data_buffer.n_robot_has_ball, 1);
    offset += 1;

    sendto(sock, data_send, offset, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
}

void callback_sub_robot_collection(const basestation::CollectionConstPtr &msg)
{
    data_buffer.n_robot_has_ball = msg->n_robot_dapat_bola;
}
void callback_sub_robot_pc2bs(const communications::PC2BSConstPtr &msg)
{
    uint8_t robot_idx = msg->n_robot - 1;
    data_buffer.pose_xyo[robot_idx][0] = msg->pos_x;
    data_buffer.pose_xyo[robot_idx][1] = msg->pos_y;
    data_buffer.pose_xyo[robot_idx][2] = msg->theta;
}