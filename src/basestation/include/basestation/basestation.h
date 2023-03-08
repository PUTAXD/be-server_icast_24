#include <chrono>
#include "ros/ros.h"
#include "communications/BS2PC.h"
#include "communications/PC2BS.h"
#include "basestation/FE2BE.h"
#include "basestation/Collection.h"
#include "basestation/EntityRobot.h"

void cllbckRcvPC2BS(const communications::PC2BS::ConstPtr& msg);
void cllbckSndBS2PC(const ros::TimerEvent& event);
void cllbckUpdateData(const ros::TimerEvent& event);

void setNRobotData();
void setBallInField();
void setRole();
void setMux1();
void setMux2();
void setMuxNRobotCloser();
void setMuxNRobotControlledBS();
void setObs();
void setCounterPass();

void setNRobotFriend(uint8_t robot_ind);

uint8_t getNRobotCloser(uint8_t robot_ind);
void getObsGroup();

int pythagoras(int x1, int y1, int x2, int y2);
uint8_t *isBallCatched();