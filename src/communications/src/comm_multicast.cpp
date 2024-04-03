//--ROS Headers
#include <ros/package.h>
#include <ros/ros.h>

//--Include Libraries
#include <icast.hpp>

//--ROS Messages
#include "basestation/EntityRobot.h"
#include "basestation/FE2BE.h"
#include "communications/BS2PC.h"
#include "communications/PC2BS.h"

#define N_ROBOT 5
#define LEN_MSG 256

//--Publisher
ros::Publisher pc2bs_pub[N_ROBOT];

//--Subscriber
ros::Subscriber bs2pc_sub;
ros::Subscriber entity_sub;
ros::Subscriber fe2be_sub;

//--Timer
ros::Timer tim_routine;

//--Singleton Initialization
Icast* icast = Icast::getInstance();

//--Prototypes
void timeCallback(const ros::TimerEvent& event);
void setDataToBeSend(Dictionary* dc_ptr);
void cllbckSndMtcast(const communications::BS2PC::ConstPtr& msg);

//-Global Variables
command_t command_bs;
pos_t my_robot_pose;
pos_t agent_robot_pose;

basestation::EntityRobot entity_msg;
basestation::FE2BE fe2be_msg;

void set_dummy_datas();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);

    icast->init();

    if (!icast->mc->initialized()) {
        std::cout << "Multicast not ready" << std::endl;
        ros::shutdown();
        return 0;
    }

    for (int i = 0; i < N_ROBOT; i++) {
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i + 1);
        pc2bs_pub[i] = nh.advertise<communications::PC2BS>(str_topic, 1);
    }

    bs2pc_sub = nh.subscribe("bs2pc", 1, cllbckSndMtcast);

    tim_routine = nh.createTimer(ros::Duration(0.02), timeCallback);

    spinner.spin();

    return 0;
}

void timeCallback(const ros::TimerEvent& event)
{
    static uint8_t prev_epoch[5];
    static communications::PC2BS msg_robot[5];

    icast->update(); //update data
///asdasdasdasd
    // set_dummy_datas();

    agent1_t agent[3];
    memcpy(&agent[0], &icast->dc->data_bus.agent1, sizeof(agent1_t));
    memcpy(&agent[1], &icast->dc->data_bus.agent2, sizeof(agent2_t));
    memcpy(&agent[2], &icast->dc->data_bus.agent3, sizeof(agent3_t));

    for (size_t i = 0; i < 3; i++) {
        msg_robot[i].pos_x = agent[i].pos.x;
        msg_robot[i].pos_y = agent[i].pos.y;
        msg_robot[i].theta = (int16_t)agent[i].pos.theta;

        // ROS_INFO("%d", msg_robot[i].theta);

        if (agent[i].ball.is_caught)
            msg_robot[i].status_bola = 2;
        else
            msg_robot[i].status_bola = agent[i].ball.is_visible;

        // if(agent[i].ball.is_visible){
        //     msg_robot[i].status_bola = 1;
        //     if(agent[i].ball.is_caught)
        //         msg_robot[i].status_bola = 2
        // }else{
        //     msg_robot[i].status_bola = 0;
        // }

        msg_robot[i].is_caught = agent[i].ball.is_caught;
        msg_robot[i].is_visible = agent[i].ball.is_visible;

        msg_robot[i].bola_x = agent[i].ball.x[0];
        msg_robot[i].bola_y = agent[i].ball.y[0];

        msg_robot[i].robot_condition = agent[i].state_machine.robot_condition;

        msg_robot[i].target_umpan = agent[i].passing.data;
        msg_robot[i].battery_health = agent[i].battery.voltage;

        msg_robot[i].goalkeeper_field_x = agent[i].keeper_on_field.target_x;
        msg_robot[i].goalkeeper_field_y = agent[i].keeper_on_field.target_y;

        msg_robot[i].bola_x_next = agent[i].prediction.ball_x;
        msg_robot[i].bola_y_next = agent[i].prediction.ball_y;

        msg_robot[i].n_robot = i + 1;

    }

    if (prev_epoch[0] != icast->dc->data_bus.agent1.epoch.data) {
        pc2bs_pub[0].publish(msg_robot[0]);
    }
    if (prev_epoch[1] != icast->dc->data_bus.agent2.epoch.data) {
        pc2bs_pub[1].publish(msg_robot[1]);
    }
    if (prev_epoch[2] != icast->dc->data_bus.agent3.epoch.data) {
        pc2bs_pub[2].publish(msg_robot[2]);
    }

    prev_epoch[0] = icast->dc->data_bus.agent1.epoch.data;
    prev_epoch[1] = icast->dc->data_bus.agent2.epoch.data;
    prev_epoch[2] = icast->dc->data_bus.agent3.epoch.data;
}

void setDataToBeSend(Dictionary* dc_ptr)
{
}

void cllbckSndMtcast(const communications::BS2PC::ConstPtr& msg)
{
    mode_base_t mode_base;
    mode_base.data = (uint8_t)msg->header_manual_and_calibration;
    icast->dc->setDataToBeSent("mode_base", (void*)&mode_base);

    command_t command;
    command.data = (uint8_t)msg->command;
    icast->dc->setDataToBeSent("command", (void*)&command);

    style_t style;
    style.data = (uint8_t)msg->style;
    icast->dc->setDataToBeSent("style", (void*)&style);

    target_manual_t target_manual;
    target_manual.x = msg->target_manual_x;
    target_manual.y = msg->target_manual_y;
    target_manual.theta = msg->target_manual_theta;
    icast->dc->setDataToBeSent("target_manual", (void*)&target_manual);

    offset_robot_t offset_robot;
    offset_robot.x = msg->offset_robot_x;
    offset_robot.y = msg->offset_robot_y;
    offset_robot.theta = msg->offset_robot_theta;
    icast->dc->setDataToBeSent("offset_robot", (void*)&offset_robot);

    data_mux_t data_mux;
    data_mux.mux_1 = msg->mux1;
    data_mux.mux_2 = msg->mux2;
    data_mux.mux_control = msg->mux_bs_control;
    icast->dc->setDataToBeSent("data_mux", (void*)&data_mux);

    trim_t trim;
    for (uint8_t i = 0; i < 5; i++) {
        trim.translation_vel[i] = msg->control_v_linear[i];
        trim.rotation_vel[i] = msg->control_v_angular[i];
        trim.kick_power[i] = msg->control_power_kicker[i];
    }
    icast->dc->setDataToBeSent("trim", (void*)&trim);

    pass_counter_t pass_counter;
    pass_counter.data = msg->passing_counter;
    icast->dc->setDataToBeSent("pass_counter", (void*)&pass_counter);
}


void set_dummy_datas(){

    static uint8_t epoch[3]={0,0,0};

    icast->dc->data_bus.agent1.pos.x = 54;
    icast->dc->data_bus.agent1.pos.y = 54;
    icast->dc->data_bus.agent1.pos.theta = 100;

    icast->dc->data_bus.agent1.epoch.data = epoch[0];
    icast->dc->data_bus.agent2.epoch.data = epoch[1];
    icast->dc->data_bus.agent3.epoch.data = epoch[2];

    icast->dc->data_bus.agent1.ball.x[0] = 400;
    icast->dc->data_bus.agent1.ball.x[0] = 400;

    icast->dc->data_bus.agent1.battery.voltage = 50;
    icast->dc->data_bus.agent1.ball.is_visible = 1;

    icast->dc->data_bus.agent2.pos.x = 90;
    icast->dc->data_bus.agent2.pos.y = 77;


    epoch[0]++;
    epoch[1]++;
    epoch[2]++;
}
