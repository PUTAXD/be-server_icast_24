#include "ros/ros.h"
#include "communications/BS2PC.h"

ros::Publisher bs2pc_pub;

ros::Timer timer_cllbck_be2fe;

communications::BS2PC msg;    

void cllbckSndBS2PC(const ros::TimerEvent& event);

int main(int argc, char *argv[]){
    ros::init(argc, argv, "basestation");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);

    bs2pc_pub = n.advertise<communications::BS2PC>("bs2pc", 1000);

    timer_cllbck_be2fe = n.createTimer(ros::Duration(0.01), cllbckSndBS2PC);

    spinner.spin();
    return 0;
}

void cllbckSndBS2PC(const ros::TimerEvent& event){
    msg.header_manual_and_calibration = 0;
    msg.command = 1;
    msg.ball_x_in_field = 2;
    msg.ball_y_in_field = 3;
    msg.target_manual_x = 4;
    msg.target_manual_y = 5;
    msg.target_manual_theta = 6;
    msg.offset_robot_x = 7;
    msg.offset_robot_y = 8;
    msg.offset_robot_theta = 9;
    msg.mux1 = 12;
    msg.mux2 = 13;
    msg.mux_bs_control = 14;
    msg.control_v_linear = {15, 16, 17, 18, 19};
    msg.control_v_angular = {20, 21, 22, 23, 24};
    msg.control_power_kicker = {25, 26, 27, 28, 29};
    msg.passing_counter = 30;

    bs2pc_pub.publish(msg);
}