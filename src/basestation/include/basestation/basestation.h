#include <chrono>
#include <algorithm>
#include "ros/ros.h"
#include "communications/BS2PC.h"
#include "communications/PC2BS.h"
#include "basestation/FE2BE.h"
#include "basestation/Collection.h"
#include "basestation/EntityRobot.h"
#include <math.h>

class CounterPass
{
public:
    uint8_t prev_state;
    std::time_t t1;
    std::time_t t2;
    uint16_t threshold_time_umpan;
    uint16_t threshold_time_lepas;
    bool sudah_umpan;
    uint8_t pass_counter;

    CounterPass() : prev_state(0),
                    t1(0),
                    t2(std::time(0)),
                    threshold_time_umpan(5000),
                    threshold_time_lepas(10000),
                    sudah_umpan(false),
                    pass_counter(0)
    {
    }
};

void cllbckRcvPC2BS(const communications::PC2BS::ConstPtr &msg);
void cllbckRcvFE2BE(const basestation::FE2BE::ConstPtr &msg);
void cllbckSndBS2PC(const ros::TimerEvent &event);
void cllbckUpdateData(const ros::TimerEvent &event);

void setNRobotData();
void setBallInField();
void setRole();
void setMux1();
void setMux2();
void setMuxNRobotCloser();
void setMuxNRobotControlledBS();
void setObs();
void setCounterPass();
void setBS2PC();
void setNRobotFriend(uint8_t robot_ind);
void setObsGroup();
void setGoalKeeper();
void setObsGlobal();

uint8_t getNRobotCloser(uint8_t robot_ind);
uint8_t *getRobotTarget();
uint8_t getNRobotClosestBall();
int16_t getAngleToPosX(uint8_t robot_ind, int angle, int dist);
int16_t getAngleToPosY(uint8_t robot_ind, int angle, int dist);

int pythagoras(int x1, int y1, int x2, int y2);
uint8_t *isBallCatched();
uint8_t isCondition20Exist();
uint8_t isRobotReady(uint8_t index_robot);
bool isBallAppear();