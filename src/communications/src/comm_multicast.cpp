#include "communications/multicast.h"
#include "communications/PC2BS.h"

#define N_ROBOT 5
#define LEN_MSG 256

communications::PC2BS pc2bs_msg;

ros::Publisher pc2bs_pub[N_ROBOT];

void cllbckRcv(const ros::TimerEvent& event);

int main(int argc, char *argv[]){
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);
    ros::Timer timer_cllbck_rcv;

    openSocket();
    
    timer_cllbck_rcv = n.createTimer(ros::Duration(0.005), cllbckRcv);

    for(int i = 0; i < N_ROBOT; i++){
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d_msg", i+1);
        pc2bs_pub[i] = n.advertise<communications::PC2BS>(str_topic, 1000);
    }

    spinner.spin();

    return 0;
}

void cllbckRcv(const ros::TimerEvent& event){
    char recv_buf[LEN_MSG];
    uint8_t nrecv = recvfrom(sd, recv_buf, LEN_MSG, MSG_DONTWAIT, &src_addr, &addr_len);
    
    if((nrecv > 0 && nrecv < 255) && (recv_buf[3] > '0' && recv_buf[3] <= '5') && (recv_buf[0] == 'i' && recv_buf[1] == 't' && recv_buf[2] == 's')) {
        uint8_t n_robot = recv_buf[3] - '0';
        int counter = 4;

        memcpy(&pc2bs_msg.epoch, recv_buf + counter, sizeof(int64_t));
        counter += sizeof(int64_t);

        memcpy(&pc2bs_msg.pos_x, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.pos_y, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.theta, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.status_bola, recv_buf + counter, sizeof(uint8_t));
        counter += sizeof(uint8_t);

        memcpy(&pc2bs_msg.bola_x, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.bola_y, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.robot_condition, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.target_umpan, recv_buf + counter, sizeof(uint8_t));
        counter += sizeof(uint8_t);

        memcpy(&pc2bs_msg.index_point, recv_buf + counter, sizeof(uint8_t));
        counter += sizeof(uint8_t);

        memcpy(&pc2bs_msg.obs_length, recv_buf + counter, sizeof(uint8_t));
        counter += sizeof(uint8_t);
        
        int16_t obs_dist = 0;
        uint8_t obs_index = 0;
        pc2bs_msg.obs_dist.clear();
        pc2bs_msg.obs_index.clear();

        for(int i = 0; i < pc2bs_msg.obs_length; i++){
            memcpy(&obs_dist, recv_buf + counter, sizeof(int16_t));
            counter += sizeof(int16_t);

            memcpy(&obs_index, recv_buf + counter, sizeof(uint8_t));
            counter += sizeof(uint8_t);

            pc2bs_msg.obs_dist.push_back(obs_dist);
            pc2bs_msg.obs_index.push_back(obs_index);
        }

        memcpy(&pc2bs_msg.battery_health, recv_buf + counter, sizeof(float_t));
        counter += sizeof(float_t);

        memcpy(&pc2bs_msg.pos_x_odom, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.pos_y_odom, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.pos_theta_odom, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.vx_icp, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        memcpy(&pc2bs_msg.vy_icp, recv_buf + counter, sizeof(int16_t));
        counter += sizeof(int16_t);

        pc2bs_pub[n_robot-1].publish(pc2bs_msg);
    }
}
