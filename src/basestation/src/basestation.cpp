#include "basestation/basestation.h"

#define N_ROBOT 5

ros::Subscriber pc2bs[N_ROBOT];
ros::Publisher bs2pc_pub;
ros::Publisher cllction_pub;
ros::Publisher entity_robot_pub;

ros::Timer timer_cllbck_bs2pc;
ros::Timer timer_update_data;

communications::BS2PC bs2pc_msg;    
communications::PC2BS pc2bs_msg[N_ROBOT];
basestation::FE2BE fe2bs_msg;
basestation::Collection cllction_data;
basestation::EntityRobot entity_robot;

int main(int argc, char *argv[]){
    ros::init(argc, argv, "basestation");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);

    for(uint8_t i=0; i<N_ROBOT; i++){
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i+1);
        pc2bs[i] = n.subscribe(str_topic, 1000, cllbckRcvPC2BS);
    }
    bs2pc_pub = n.advertise<communications::BS2PC>("bs2pc", 1000);
    entity_robot_pub = n.advertise<basestation::EntityRobot>("entity_robot", 1000);
    cllction_pub = n.advertise<basestation::Collection>("collection", 1000);

    timer_cllbck_bs2pc = n.createTimer(ros::Duration(0.01), cllbckSndBS2PC);
    timer_update_data = n.createTimer(ros::Duration(0.01), cllbckUpdateData);

    spinner.spin();
    return 0;
}

void cllbckSndBS2PC(const ros::TimerEvent& event){
    bs2pc_msg.header_manual_and_calibration = 0;
    bs2pc_msg.command = 1;
    bs2pc_msg.ball_x_in_field = 2;
    bs2pc_msg.ball_y_in_field = 3;
    bs2pc_msg.target_manual_x = 4;
    bs2pc_msg.target_manual_y = 5;
    bs2pc_msg.target_manual_theta = 6;
    bs2pc_msg.offset_robot_x = 7;
    bs2pc_msg.offset_robot_y = 8;
    bs2pc_msg.offset_robot_theta = 9;
    bs2pc_msg.mux1 = 12;
    bs2pc_msg.mux2 = 13;
    bs2pc_msg.mux_bs_control = 14;
    bs2pc_msg.control_v_linear = {15, 16, 17, 18, 19};
    bs2pc_msg.control_v_angular = {20, 21, 22, 23, 24};
    bs2pc_msg.control_power_kicker = {25, 26, 27, 28, 29};
    bs2pc_msg.passing_counter = 30;

    bs2pc_pub.publish(bs2pc_msg);
    entity_robot_pub.publish(entity_robot);
    cllction_pub.publish(cllction_data);
}

void cllbckUpdateData(const ros::TimerEvent& event){
    setNRobotData();
    setBallInField();
    setRole();
    setMux1();
    setMux2();
    setMuxNRobotCloser();
    setMuxNRobotControlledBS();
    setObs();
    getObsGroup();
    setCounterPass();
}

void cllbckRcvPC2BS(const communications::PC2BS::ConstPtr& msg){
    uint8_t robot_ind = msg->n_robot - 1;
    pc2bs_msg[robot_ind].n_robot = msg->n_robot;
    pc2bs_msg[robot_ind].epoch = msg->epoch;
    pc2bs_msg[robot_ind].pos_x = msg->pos_x;
    pc2bs_msg[robot_ind].pos_y = msg->pos_y;
    pc2bs_msg[robot_ind].theta = msg->theta;
    pc2bs_msg[robot_ind].status_bola = msg->status_bola;
    pc2bs_msg[robot_ind].bola_x = msg->bola_x;    
    pc2bs_msg[robot_ind].bola_y = msg->bola_y;
    pc2bs_msg[robot_ind].robot_condition = msg->robot_condition;
    pc2bs_msg[robot_ind].target_umpan = msg->target_umpan;
    pc2bs_msg[robot_ind].index_point = msg->index_point;
    pc2bs_msg[robot_ind].obs_length = msg->obs_length;
    pc2bs_msg[robot_ind].obs_dist = msg->obs_dist;
    pc2bs_msg[robot_ind].obs_index = msg->obs_index;
    pc2bs_msg[robot_ind].battery_health = msg->battery_health;
    pc2bs_msg[robot_ind].pos_x_odom = msg->pos_x_odom;
    pc2bs_msg[robot_ind].pos_y_odom = msg->pos_y_odom;
    pc2bs_msg[robot_ind].pos_theta_odom = msg->pos_theta_odom;
    pc2bs_msg[robot_ind].vx_icp = msg->vx_icp;
    pc2bs_msg[robot_ind].vy_icp = msg->vy_icp;

    entity_robot.is_active[robot_ind] = 1;
    std::chrono::seconds time_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
    entity_robot.time_coming[robot_ind] = time_now.count();
}

/* Update Data Global */

void setNRobotData(){
    std::chrono::seconds time_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
    uint8_t timeout = 2;

    for(uint8_t i=0; i<N_ROBOT; i++){
        if(entity_robot.is_active[i]){
            if(time_now.count() - entity_robot.time_coming[i] > timeout){
                entity_robot.is_active[i] = 0;
            } else{
                setNRobotFriend(i);
            }
        }
    }

    uint8_t n_robot_active = 0;
    for(uint8_t i=1; i<N_ROBOT; i++){
        if(entity_robot.is_active[i]){
            n_robot_active++;
        }
    }

    cllction_data.n_robot_aktif = n_robot_active;
}

void setBallInField(){};

void setRole(){};

void setMux1(){};

void setMux2(){};

void setMuxNRobotCloser(){};

void setMuxNRobotControlledBS(){};

void setObs(){};

void getObsGroup(){};

void setCounterPass(){};

/* Process Data which need sub function */
void setNRobotFriend(uint8_t robot_ind){
    uint8_t n_robot_friend = 0;
    if(cllction_data.n_robot_aktif <= 1){
        entity_robot.n_robot_teman[robot_ind] = 0;
    } else{
        entity_robot.n_robot_teman[robot_ind] = getNRobotCloser(robot_ind);
    }
}

/* GETTER function */
uint8_t getNRobotCloser(uint8_t robot_ind){
    uint8_t n_robot_closer = 0;
    int distance_closer = INT_MAX;

    for(uint8_t i=1; i<N_ROBOT; i++){
        if(i != robot_ind && entity_robot.is_active[i]){
            int distance = pythagoras(pc2bs_msg[robot_ind].pos_x, pc2bs_msg[robot_ind].pos_y, pc2bs_msg[i].pos_x, pc2bs_msg[i].pos_y);
            if(distance < distance_closer){
                distance_closer = distance;
                n_robot_closer = i;
            }
        }
    }

    return n_robot_closer;
}

/* Formula function */
int pythagoras(int x1, int y1, int x2, int y2){
    int x = x1 - x2;
    int y = y1 - y2;
    int z = sqrt(pow(x, 2) + pow(y, 2));
    return z;
}
