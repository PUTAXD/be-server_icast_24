#include "basestation/basestation.h"

#define N_ROBOT 5
#define DEG2RAD 0.017452925

ros::Subscriber pc2bs[N_ROBOT];
ros::Subscriber fe2be_sub;
ros::Subscriber auto_cmd_sub;

ros::Publisher bs2pc_pub;
ros::Publisher cllction_pub;
ros::Publisher entity_robot_pub;

ros::Timer timer_cllbck_bs2pc;
ros::Timer timer_update_data;
ros::Timer timer_role;

communications::BS2PC bs2pc_msg;
communications::PC2BS pc2bs_msg[N_ROBOT];
basestation::FE2BE fe2be_msg;
basestation::Collection cllction_data;
basestation::EntityRobot entity_robot;
basestation::AutoCmd auto_cmd_msg;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "basestation");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(1);

    fe2be_sub = n.subscribe("ui2server", 1, cllbckRcvFE2BE);
    auto_cmd_sub = n.subscribe("auto_cmd", 1, cllbckAutoCmd);
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i + 1);
        pc2bs[i] = n.subscribe(str_topic, 1, cllbckRcvPC2BS);
    }
    bs2pc_pub = n.advertise<communications::BS2PC>("bs2pc", 1);
    entity_robot_pub = n.advertise<basestation::EntityRobot>("entity_robot", 1);
    cllction_pub = n.advertise<basestation::Collection>("collection", 1);

    entity_robot.role[0] = 0; // default: kiper
    entity_robot.role[1] = 1; // default: att
    entity_robot.role[2] = 2; // default: defen
    entity_robot.role[3] = 3;
    entity_robot.role[4] = 4;

    timer_cllbck_bs2pc = n.createTimer(ros::Duration(0.001), cllbckSndBS2PC);
    timer_update_data = n.createTimer(ros::Duration(0.02), cllbckUpdateData);
    // timer_role = n.createTimer(ros::Duration(0.3), cllbckRole);

    spinner.spin();
    return 0;
}

void cllbckUpdateData(const ros::TimerEvent &event)
{
    setNRobotData();
    setBallInField();
    setRole();
    setMux1();
    setMux2();
    // setMux1JS();
    // setMux2JS();
    setMuxNRobotCloser();
    setMuxNRobotControlledBS();
    setObsGroupOnly();
    // setObs();
    // setObsGroup();
    // setObsGlobal();
    setCounterPass();
    setGoalKeeper();
    setBS2PC();
}

void cllbckAutoCmd(const basestation::AutoCmd::ConstPtr &msg)
{
    auto_cmd_msg.name = msg->name;
    auto_cmd_msg.ip = msg->ip;

    char auto_run_str[30];
    if (auto_cmd_msg.name.compare("run") == 0)
    {
        sprintf(auto_run_str, "nc -w 1 %s 65531", auto_cmd_msg.ip.c_str());
    }
    else if (auto_cmd_msg.name.compare("stop") == 0)
    {
        sprintf(auto_run_str, "nc -w 1 %s 65530", auto_cmd_msg.ip.c_str());
    }

    system(auto_run_str);
}

void cllbckRole(const ros::TimerEvent &event)
{
    setRole();
}

void write_u16bit(uint16_t *dst, int8_t *src, uint8_t total_bit, uint8_t offset_bit)
{
    for (uint8_t i = 0; i < total_bit; i++)
        *dst |= (((*src & (1 << i)) >> i) << i + offset_bit);
}

void cllbckRcvPC2BS(const communications::PC2BS::ConstPtr &msg)
{
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
    pc2bs_msg[robot_ind].bola_x_next = msg->bola_x_next;
    pc2bs_msg[robot_ind].bola_y_next = msg->bola_y_next;
    pc2bs_msg[robot_ind].pos_x_next = msg->pos_x_next;
    pc2bs_msg[robot_ind].pos_y_next = msg->pos_y_next;
    pc2bs_msg[robot_ind].goalkeeper_field_x = msg->goalkeeper_field_x;
    pc2bs_msg[robot_ind].goalkeeper_field_y = msg->goalkeeper_field_y;
    pc2bs_msg[robot_ind].pos_obs_length = msg->pos_obs_length;
    pc2bs_msg[robot_ind].pos_obs_x = msg->pos_obs_x;
    pc2bs_msg[robot_ind].pos_obs_y = msg->pos_obs_y;
    // pc2bs_msg[robot_ind].is_caught = msg ->

    entity_robot.is_active[robot_ind] = true;
    std::chrono::seconds time_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
    entity_robot.time_coming[robot_ind] = time_now.count();
}

void cllbckRcvFE2BE(const basestation::FE2BE::ConstPtr &msg)
{
    fe2be_msg = *msg;
}

bool isSendToPC()
{
    return (fe2be_msg.status_control_robot[0] || fe2be_msg.status_control_robot[1] || fe2be_msg.status_control_robot[2] || fe2be_msg.status_control_robot[3] || fe2be_msg.status_control_robot[4]);
}

void cllbckSndBS2PC(const ros::TimerEvent &event)
{
    entity_robot_pub.publish(entity_robot);
    cllction_pub.publish(cllction_data);
    if (isSendToPC())
    {
    bs2pc_pub.publish(bs2pc_msg);
    }
}

/* Update/setter Data Global */

void setNRobotData()
{
    std::chrono::seconds time_now =
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
    uint8_t timeout = 1;

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        if (entity_robot.is_active[i])
        {
            if (time_now.count() - entity_robot.time_coming[i] > timeout)
            {
                entity_robot.is_active[i] = 0;
            }
        }
        setNRobotFriend(i);
    }

    uint8_t n_robot_active = 0;
    uint8_t temp_robot_ready = 0;
    for (uint8_t i = 1; i < N_ROBOT; i++)
    {
        if (entity_robot.is_active[i])
        {
            n_robot_active++;
        }

        if (isRobotReady(i))
        {
            temp_robot_ready++;
        }
    }

    cllction_data.n_robot_aktif = n_robot_active;
    cllction_data.n_robot_ready = temp_robot_ready;
}

struct RobotWithBall
{
    uint8_t n_robot;
    float distance = 9999;
};

void setSortingBallDistance()
{
    struct RobotWithBall robots[5];
    for (uint8_t i = 1; i < N_ROBOT; i++)
    {
        robots[i].n_robot = i;
        if (isRobotReady(i))
        {
            robots[i].distance = pythagoras(pc2bs_msg[i].bola_x,
                                            pc2bs_msg[i].bola_y,
                                            pc2bs_msg[i].pos_x,
                                            pc2bs_msg[i].pos_y);
        }
    }

    std::sort(robots, robots + N_ROBOT, [](const RobotWithBall &a, const RobotWithBall &b)
              { return a.distance < b.distance; });

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        if (robots[i].distance != 9999)
        {
            cllction_data.n_array_robot_dekat_bola[i] = robots[i].n_robot;
        }
        else
        {
            cllction_data.n_array_robot_dekat_bola[i] = 0;
        }
    }
}

void setBallInField()
{
    /*
        isBallCatched[0] = status
        isBallCatched[1] = index robot
    */
    uint8_t *is_ball_catched = isBallCatched();
    if (is_ball_catched[0])
    {
        cllction_data.n_robot_dapat_bola = is_ball_catched[1] + 1;
        cllction_data.n_robot_dekat_bola = is_ball_catched[1] + 1;

        cllction_data.bola_x_pada_lapangan = pc2bs_msg[is_ball_catched[1]].bola_x;
        cllction_data.bola_y_pada_lapangan = pc2bs_msg[is_ball_catched[1]].bola_y;

        uint8_t *robot_target = getRobotTarget();
        cllction_data.n_robot_umpan = robot_target[0];
        cllction_data.n_robot_terima = robot_target[1];
    }
    else if (isBallAppear())
    {
        uint8_t n_robot_closest_ball = getNRobotClosestBall();
        cllction_data.n_robot_dapat_bola = 0;
        cllction_data.n_robot_dekat_bola = n_robot_closest_ball;
        if (n_robot_closest_ball)
        {
            cllction_data.bola_x_pada_lapangan = pc2bs_msg[n_robot_closest_ball - 1].bola_x;
            cllction_data.bola_y_pada_lapangan = pc2bs_msg[n_robot_closest_ball - 1].bola_y;
        }
        else
        {
            cllction_data.bola_x_pada_lapangan = 0;
            cllction_data.bola_y_pada_lapangan = 0;
        }
        cllction_data.n_robot_umpan = 0;
        cllction_data.n_robot_terima = 0;

        // uint8_t n_dekat_bola = 0;
        // int distance_ball = INT_MAX;
        // for (int i = 0; i < N_ROBOT; i++)
        // {
        //     if (isRobotReady(i))
        //     {
        //         int distance = pythagoras(pc2bs_msg[i].pos_x, pc2bs_msg[i].pos_y, cllction_data.bola_x_pada_lapangan, cllction_data.bola_y_pada_lapangan);
        //         if (distance < distance_ball)
        //         {
        //             distance_ball = distance;
        //             n_dekat_bola = i + 1;
        //         }
        //     }
        // }

        // cllction_data.n_robot_dekat_bola = n_dekat_bola;

        // cllction_data.bola_x_pada_lapangan = pc2bs_msg[n_dekat_bola - 1].bola_x;
        // cllction_data.bola_y_pada_lapangan = pc2bs_msg[n_dekat_bola - 1].bola_y;
    }
    else
    {
        cllction_data.n_robot_dapat_bola = 0;
        cllction_data.n_robot_dekat_bola = 0;
        cllction_data.bola_x_pada_lapangan = 0;
        cllction_data.bola_y_pada_lapangan = 0;
        cllction_data.n_robot_umpan = 0;
        cllction_data.n_robot_terima = 0;
    }

    // asdasdasdasd
    if (cllction_data.bola_x_pada_lapangan == 0 && cllction_data.bola_y_pada_lapangan == 0)
    {
        cllction_data.n_robot_dekat_bola = 0;
    }

    setSortingBallDistance();
};

uint8_t *isTargetUmpanExist()
{
    static uint8_t is_target_umpan_exist[3] = {0, 0, 0};
    is_target_umpan_exist[0] = 0;
    is_target_umpan_exist[1] = 0;
    is_target_umpan_exist[2] = 0;
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        if (pc2bs_msg[i].target_umpan != 0)
        {
            is_target_umpan_exist[0] = 1;
            is_target_umpan_exist[1] = i;
            is_target_umpan_exist[2] = pc2bs_msg[i].target_umpan;
        }
    }
    return is_target_umpan_exist;
}

int16_t pos_x = 0;
int16_t pos_y = 0;

void setRole()
{
    // 0 Goal Keeper
    // 1 attacker
    // 2 defender 1
    // 3 assist
    // 4 defender 2

    /*
        isTargetUmpanExist[0] = status
        isTargetUmpanExist[1] = source
        isTargetUmpanExist[2] = target
    */
    // uint8_t *is_target_umpan_exist = isTargetUmpanExist();

    // if (is_target_umpan_exist[0] && !isConditionExist(20))
    // {
    //     if (is_target_umpan_exist[1] != (is_target_umpan_exist[2] - 1) &&
    //         pc2bs_msg[is_target_umpan_exist[1]].status_bola != 2)
    //     {
    //         entity_robot.role[is_target_umpan_exist[1]] = 3;
    //         entity_robot.role[is_target_umpan_exist[2] - 1] = 1;
    //     }
    // }
    // else
    // if (!isConditionExist(20))
    // {
    //     uint8_t LEN_ARR_ROBOT_DEKAT_BOLA = sizeof(cllction_data.n_array_robot_dekat_bola) /
    //                                        sizeof(cllction_data.n_array_robot_dekat_bola[0]);
    //     uint8_t counter_role = 1;

    //     if (cllction_data.n_robot_dekat_bola != 0)
    //     {
    //         for (uint8_t i = 0; i < LEN_ARR_ROBOT_DEKAT_BOLA; i++)
    //         {
    //             if (cllction_data.n_array_robot_dekat_bola[i] != 0)
    //             {
    //                 int8_t INDEX_ROBOT = cllction_data.n_array_robot_dekat_bola[i];
    //                 switch (counter_role)
    //                 {
    //                 case 1:
    //                     entity_robot.role[INDEX_ROBOT] = 1;
    //                     break;
    //                 case 2:
    //                     entity_robot.role[INDEX_ROBOT] = 3;
    //                     break;
    //                 case 3:
    //                     entity_robot.role[INDEX_ROBOT] = 2;
    //                     break;
    //                 case 4:
    //                     entity_robot.role[INDEX_ROBOT] = 4;
    //                     break;
    //                 }
    //                 counter_role++;
    //             }
    //             if (!isRobotReady(i))
    //             {
    //                 entity_robot.role[i] = 0;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         for (int i = 0; i < N_ROBOT; i++)
    //         {
    //             if (isRobotReady(i))
    //             {
    //                 if (cllction_data.n_robot_ready == 1)
    //                 {
    //                     entity_robot.role[i] = 1;
    //                 }
    //                 else if (
    //                     cllction_data.n_robot_dapat_bola == 0 && entity_robot.role[0] == 0 && entity_robot.role[1] == 0 && entity_robot.role[2] == 0 && entity_robot.role[3] == 0 && entity_robot.role[4] == 0)
    //                 {
    //                     if (isRobotReady(1))
    //                     {
    //                         entity_robot.role[1] = 1;
    //                     }
    //                     if (isRobotReady(2))
    //                     {
    //                         entity_robot.role[2] = 3;
    //                     }
    //                     if (isRobotReady(3))
    //                     {
    //                         entity_robot.role[3] = 2;
    //                     }
    //                     if (isRobotReady(4))
    //                     {
    //                         entity_robot.role[4] = 4;
    //                     }
    //                 }
    //             }
    //             else
    //             {
    //                 entity_robot.role[i] = 0;
    //             }
    //         }
    //     }
    //     entity_robot.role[0] = 0;
    // }

    if (fe2be_msg.style == 67 || fe2be_msg.style == 68)
    {
        if (isRobotReady(2))
        {
            entity_robot.role[1] = 1;
            entity_robot.role[2] = 2;
        }
        if (isRobotReady(1))
        {
            entity_robot.role[1] = 1;
            entity_robot.role[2] = 2;
        }
    }
    else
    {
        if (cllction_data.n_robot_ready == 1)
        {
            if (isRobotReady(2))
            {
                entity_robot.role[1] = 1;
                entity_robot.role[2] = 2;
            }
            if (isRobotReady(1))
            {
                entity_robot.role[1] = 1;
                entity_robot.role[2] = 2;
            }
        }
        else if (cllction_data.n_robot_ready == 2)
        {

            static float buffer_bola_x;
            static float buffer_bola_y;

            if (cllction_data.bola_x_pada_lapangan != 0 && cllction_data.bola_y_pada_lapangan != 0)
            {
                buffer_bola_x = cllction_data.bola_x_pada_lapangan;
                buffer_bola_y = cllction_data.bola_y_pada_lapangan;
            }

            if (cllction_data.n_robot_dapat_bola == 0)
            {
                pos_x = buffer_bola_x;
                pos_y = buffer_bola_y;
            }

            float jarak_ = pythagoras(pos_x, pos_y, cllction_data.bola_x_pada_lapangan, cllction_data.bola_y_pada_lapangan);

            uint8_t n_robot_attacker = -1;
            uint8_t n_robot_def_1 = -1;

            for (int i = 0; i < N_ROBOT; i++)
            {
                if (entity_robot.role[i] == 1)
                    n_robot_attacker = i + 1; // Nama Robot Sebenarnya
                if (entity_robot.role[i] == 2)
                    n_robot_def_1 = i + 1; // Nama Robot Sebenarnya
            }

            if (jarak_ > 125 && cllction_data.n_robot_dapat_bola == n_robot_attacker)
            {
                uint8_t role_buffer = entity_robot.role[n_robot_attacker - 1];
                entity_robot.role[n_robot_attacker - 1] = entity_robot.role[n_robot_def_1 - 1];
                entity_robot.role[n_robot_def_1 - 1] = role_buffer;
            }
        }
    }

    // entity_robot.role[0] = 0; // 1
    // entity_robot.role[1] = 1; // 2
    // entity_robot.role[2] = 3;
    // entity_robot.role[3] = 2;
    // entity_robot.role[4] = 4;
};

void setMux1()
{
    uint8_t CONVERSION = 6;
    uint16_t mux = 0;

    write_u16bit(&mux, &cllction_data.n_robot_ready, 3, 0);
    write_u16bit(&mux, &cllction_data.n_robot_dekat_bola, 3, 3);
    write_u16bit(&mux, &cllction_data.n_robot_dapat_bola, 3, 6);
    write_u16bit(&mux, &cllction_data.n_robot_umpan, 3, 9);
    write_u16bit(&mux, &cllction_data.n_robot_terima, 3, 12);

    cllction_data.mux1 = mux;
};

void setMux1JS()
{
    uint8_t CONVERSION = 6;
    uint16_t mux = 0;

    mux += cllction_data.n_robot_ready;
    mux += cllction_data.n_robot_dekat_bola * CONVERSION;
    mux += cllction_data.n_robot_dapat_bola * CONVERSION * CONVERSION;
    mux += cllction_data.n_robot_umpan * CONVERSION * CONVERSION * CONVERSION;
    mux += cllction_data.n_robot_terima * CONVERSION * CONVERSION * CONVERSION * CONVERSION;

    cllction_data.mux1 = mux;
}

void setMux2()
{
    uint8_t CONVERSION = 6;
    uint16_t mux = 0;

    write_u16bit(&mux, &entity_robot.role[0], 3, 0);
    write_u16bit(&mux, &entity_robot.role[1], 3, 3);
    write_u16bit(&mux, &entity_robot.role[2], 3, 6);
    write_u16bit(&mux, &entity_robot.role[3], 3, 9);
    write_u16bit(&mux, &entity_robot.role[4], 3, 12);

    cllction_data.mux2 = mux;
};

void setMux2JS()
{
    uint8_t CONVERSION = 6;
    uint16_t mux = 0;

    mux += entity_robot.role[0];
    mux += entity_robot.role[1] * CONVERSION;
    mux += entity_robot.role[2] * CONVERSION * CONVERSION;
    mux += entity_robot.role[3] * CONVERSION * CONVERSION * CONVERSION;
    mux += entity_robot.role[4] * CONVERSION * CONVERSION * CONVERSION * CONVERSION;

    cllction_data.mux2 = mux;
}

void setMuxNRobotCloser()
{
    uint8_t CONVERSION = 10;
    uint16_t mux = 0;

    mux += entity_robot.n_robot_teman[0];
    mux += entity_robot.n_robot_teman[1] * CONVERSION;
    mux += entity_robot.n_robot_teman[2] * CONVERSION * CONVERSION;
    mux += entity_robot.n_robot_teman[3] * CONVERSION * CONVERSION * CONVERSION;
    mux += entity_robot.n_robot_teman[4] * CONVERSION * CONVERSION * CONVERSION * CONVERSION;

    cllction_data.mux_n_robot_closer = mux;
};

void setMuxNRobotControlledBS()
{
    uint16_t mux = 0;

    mux |= fe2be_msg.status_control_robot[0] * 0b00001;
    mux |= fe2be_msg.status_control_robot[1] * 0b00010;
    mux |= fe2be_msg.status_control_robot[2] * 0b00100;
    mux |= fe2be_msg.status_control_robot[3] * 0b01000;
    mux |= fe2be_msg.status_control_robot[4] * 0b10000;

    cllction_data.mux_bs_control_robot = mux;
};

// void setObsGroupOnly()
// {
//     std::vector<int> obs_x;
//     std::vector<int> obs_y;

//     for (uint8_t i = 0; i < N_ROBOT; i++)
//     {
//         int len_obs = pc2bs_msg[i].obs_length;
//         if (len_obs == pc2bs_msg[i].obs_index.size() && len_obs == pc2bs_msg[i].obs_dist.size())
//         {
//             ROS_INFO("len obs: %d\n", len_obs);
//             for (uint8_t j = 0; j < len_obs; j++)
//             {
//                 int dist = pc2bs_msg[i].obs_dist[j];
//                 int angle = pc2bs_msg[i].obs_index[j] * 2.5;

//                 int obs_x_temp = getAngleToPosX(i, angle, dist);
//                 int obs_y_temp = getAngleToPosY(i, angle, dist);
//                 if (!(obs_x_temp == pc2bs_msg[i].pos_x && obs_y_temp == pc2bs_msg[i].pos_y))
//                 {
//                     obs_x.push_back(getAngleToPosX(i, angle, dist));
//                     obs_y.push_back(getAngleToPosY(i, angle, dist));
//                 }
//             }

//             switch (i)
//             {
//             case 0:
//                 entity_robot.group_obs_x_r1 = obs_x;
//                 entity_robot.group_obs_y_r1 = obs_y;
//                 break;
//             case 1:
//                 entity_robot.group_obs_x_r2 = obs_x;
//                 entity_robot.group_obs_y_r2 = obs_y;
//                 break;
//             case 2:
//                 entity_robot.group_obs_x_r3 = obs_x;
//                 entity_robot.group_obs_y_r3 = obs_y;
//                 break;
//             case 3:
//                 entity_robot.group_obs_x_r4 = obs_x;
//                 entity_robot.group_obs_y_r4 = obs_y;
//                 break;
//             case 4:
//                 entity_robot.group_obs_x_r5 = obs_x;
//                 entity_robot.group_obs_y_r5 = obs_y;
//                 break;
//             }
//         }

//         obs_x.clear();
//         obs_y.clear();
//     }
// }

void setObsGroupOnly()
{
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        // std::vector<int> obs_x;
        // std::vector<int> obs_y;
        // // int len_obs = pc2bs_msg[i].obs_length;
        // int len_obs = pc2bs_msg[i].pos_obs_length;
        // // if (len_obs == pc2bs_msg[i].obs_index.size() && len_obs == pc2bs_msg[i].obs_dist.size())
        // if (len_obs == pc2bs_msg[i].pos_obs_x.size() && len_obs == pc2bs_msg[i].pos_obs_x.size())
        // {
        // for (uint8_t j = 0; j < len_obs; j++)
        // {
        //     int dist = pc2bs_msg[i].obs_dist[j];
        //     int angle = pc2bs_msg[i].obs_index[j] * 2.5;

        //     int obs_x_temp = getAngleToPosX(i, angle, dist);
        //     int obs_y_temp = getAngleToPosY(i, angle, dist);
        //     if (!(obs_x_temp == pc2bs_msg[i].pos_x && obs_y_temp == pc2bs_msg[i].pos_y))
        //     {
        //         obs_x.push_back(getAngleToPosX(i, angle, dist));
        //         obs_y.push_back(getAngleToPosY(i, angle, dist));
        //     }
        // }

        switch (i)
        {
        case 0:
            entity_robot.group_obs_x_r1 = pc2bs_msg[i].pos_obs_x;
            entity_robot.group_obs_y_r1 = pc2bs_msg[i].pos_obs_y;
            break;
        case 1:
            entity_robot.group_obs_x_r2 = pc2bs_msg[i].pos_obs_x;
            entity_robot.group_obs_y_r2 = pc2bs_msg[i].pos_obs_y;
            break;
        case 2:
            entity_robot.group_obs_x_r3 = pc2bs_msg[i].pos_obs_x;
            entity_robot.group_obs_y_r3 = pc2bs_msg[i].pos_obs_y;
            break;
        case 3:
            entity_robot.group_obs_x_r4 = pc2bs_msg[i].pos_obs_x;
            entity_robot.group_obs_y_r4 = pc2bs_msg[i].pos_obs_y;
            break;
        case 4:
            entity_robot.group_obs_x_r5 = pc2bs_msg[i].pos_obs_x;
            entity_robot.group_obs_y_r5 = pc2bs_msg[i].pos_obs_y;
            break;
        }
    }
    // obs_x.clear();
    // obs_y.clear();
    // }
}

void setObs()
{
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        uint8_t LEN_OBS = pc2bs_msg[i].obs_length;
        std::vector<int16_t> obs_x;
        std::vector<int16_t> obs_y;

        if (pc2bs_msg[i].obs_dist.size() == LEN_OBS && pc2bs_msg[i].obs_index.size() == LEN_OBS)
            for (uint8_t j = 0; j < LEN_OBS; j++)
            {
                uint16_t dist = pc2bs_msg[i].obs_dist[j];
                uint16_t angle = pc2bs_msg[i].obs_index[j] * 2.5;

                obs_x.push_back(getAngleToPosX(i, angle, dist));
                obs_y.push_back(getAngleToPosY(i, angle, dist));
            }

        switch (i)
        {
        case 0:
            entity_robot.obs_x_r1 = obs_x;
            entity_robot.obs_y_r1 = obs_y;
            break;
        case 1:
            entity_robot.obs_x_r2 = obs_x;
            entity_robot.obs_y_r2 = obs_y;
            break;
        case 2:
            entity_robot.obs_x_r3 = obs_x;
            entity_robot.obs_y_r3 = obs_y;
            break;
        case 3:
            entity_robot.obs_x_r4 = obs_x;
            entity_robot.obs_y_r4 = obs_y;
            break;
        case 4:
            entity_robot.obs_x_r5 = obs_x;
            entity_robot.obs_y_r5 = obs_y;
            break;
        }

        obs_x.clear();
        obs_y.clear();
    }
};

void setCounterPass()
{
    CounterPass *counter = new CounterPass();
    uint8_t state = cllction_data.n_robot_dapat_bola > 0;

    if (counter->prev_state == 1 && state == 0)
    {
        counter->t1 = std::time(0);
    }
    else if (counter->prev_state == 0 && state == 1)
    {
        counter->t2 = std::time(0);
    }

    std::time_t current_time = std::time(0);

    if (counter->t2 - counter->t1 < counter->threshold_time_umpan && counter->prev_state == 0 && state == 1)
    {
        cllction_data.pass_counter++;
        counter->sudah_umpan = true;
    }
    else if (current_time - counter->t2 > counter->threshold_time_lepas)
    {
        cllction_data.pass_counter = 0;
        counter->sudah_umpan = false;
    }

    cllction_data.pass_counter > 222 ? 222 : cllction_data.pass_counter;

    counter->prev_state = state;
};

void setBS2PC()
{
    uint8_t bs_manual = fe2be_msg.header_manual ? 1 : 0;
    uint8_t auto_call = fe2be_msg.auto_kalibrasi ? 1 : 0;

    bs2pc_msg.header_manual_and_calibration = bs_manual;
    bs2pc_msg.header_manual_and_calibration |= (auto_call << 1);

    bs2pc_msg.command = fe2be_msg.command;
    bs2pc_msg.style = fe2be_msg.style;
    bs2pc_msg.ball_x_in_field = cllction_data.bola_x_pada_lapangan;
    bs2pc_msg.ball_y_in_field = cllction_data.bola_y_pada_lapangan;
    bs2pc_msg.target_manual_x = fe2be_msg.target_manual_x;
    bs2pc_msg.target_manual_y = fe2be_msg.target_manual_y;
    bs2pc_msg.target_manual_theta = fe2be_msg.target_manual_theta;
    bs2pc_msg.offset_robot_x = fe2be_msg.odometry_offset_robot_x;
    bs2pc_msg.offset_robot_y = fe2be_msg.odometry_offset_robot_y;
    bs2pc_msg.mux_bs_control = cllction_data.mux_bs_control_robot;
    bs2pc_msg.passing_counter = cllction_data.pass_counter;
    bs2pc_msg.mux1 = cllction_data.mux1;
    bs2pc_msg.mux2 = cllction_data.mux2;
    bs2pc_msg.offset_robot_theta = fe2be_msg.odometry_offset_robot_theta;
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.control_v_linear[i] = fe2be_msg.trim_kecepatan_robot[i];
    }
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.control_v_angular[i] = fe2be_msg.trim_kecepatan_sudut_robot[i];
    }
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.control_power_kicker[i] = fe2be_msg.trim_penendang_robot[i];
    }
    for (uint8_t i = 0; i < 6; i++)
    {
        // ROS_INFO("pos obs - %d: %d", i, fe2be_msg.pos_obs[i]);
        bs2pc_msg.pos_obs[i] = fe2be_msg.pos_obs[i];
    }
    for (int i = 0; i < 3; i++){
        bs2pc_msg.index_obs[i] = fe2be_msg.index_obs[i];
    }

    // self data for unicast
    for (int i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.status_active[i] = entity_robot.is_active[i];
    }

    for (int i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.pos_x[i] = pc2bs_msg[i].pos_x;
    }

    for (int i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.pos_y[i] = pc2bs_msg[i].pos_y;
    }

    for (int i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.theta[i] = pc2bs_msg[i].theta;
    }

    for (int i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.robot_condition[i] = pc2bs_msg[i].robot_condition;
    }

    for (int i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.robot_condition[i] = pc2bs_msg[i].robot_condition;
    }

    for (int i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.pass_target_x[i] = pc2bs_msg[i].pass_target_x;
    }

    for (int i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.pass_target_y[i] = pc2bs_msg[i].pass_target_y;
    }

    for (int i = 0; i < N_ROBOT; i++)
    {
        bs2pc_msg.target_umpan[i] = pc2bs_msg[i].target_umpan;
    }

    //obs

};

/* Process Data which need sub function */
void setNRobotFriend(uint8_t robot_ind)
{
    uint8_t n_robot_friend = 0;
    if (cllction_data.n_robot_ready <= 1 || !isRobotReady(robot_ind))
    {
        entity_robot.n_robot_teman[robot_ind] = 0;
    }
    else
    {
        entity_robot.n_robot_teman[robot_ind] = getNRobotCloser(robot_ind);
    }
}

void setGoalKeeper()
{
    int16_t pos_x[N_ROBOT];
    int16_t pos_y[N_ROBOT];
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        pos_x[i] = pc2bs_msg[i].goalkeeper_field_x;
        pos_y[i] = pc2bs_msg[i].goalkeeper_field_y;
    }

    int16_t goalkeeper_x = pos_x[0];
    int16_t goalkeeper_y = pos_y[0];

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        if (abs(1200 - pos_y[i]) < abs(1200 - goalkeeper_y))
        {
            goalkeeper_y = pos_y[i];
            if (abs(400 - pos_x[i]) < abs(400 - goalkeeper_x))
            {
                goalkeeper_x = pos_x[i];
            }
        }
    }

    cllction_data.goalkeeper_field_x = goalkeeper_x;
    cllction_data.goalkeeper_field_y = goalkeeper_y;
}

// void setObsGlobal()
// {

//     // make temporary array to store obs_global_x and obs_global_y

//     std::vector<int> obs_region_x;
//     std::vector<int> obs_region_y;
//     std::vector<int> obs_global_x_temp;
//     std::vector<int> obs_global_y_temp;
//     for (uint8_t i = 0; i < N_ROBOT; i++)
//     {
//         if (isRobotReady(i))
//         {
//             switch (i)
//             {
//             case 0:
//                 obs_region_x = entity_robot.group_obs_x_r1;
//                 obs_region_y = entity_robot.group_obs_y_r1;
//                 break;
//             case 1:
//                 obs_region_x = entity_robot.group_obs_x_r2;
//                 obs_region_y = entity_robot.group_obs_y_r2;
//                 break;
//             case 2:
//                 obs_region_x = entity_robot.group_obs_x_r3;
//                 obs_region_y = entity_robot.group_obs_y_r3;
//                 break;
//             case 3:

//                 obs_region_x = entity_robot.group_obs_x_r4;
//                 obs_region_y = entity_robot.group_obs_y_r4;
//                 break;
//             case 4:
//                 obs_region_x = entity_robot.group_obs_x_r5;
//                 obs_region_y = entity_robot.group_obs_y_r5;
//                 break;
//             }

//             int len_obs = obs_region_x.size();

//             for (int16_t j = 0; j < len_obs; j++)
//             {
//                 obs_global_x_temp.push_back(obs_region_x[j]);
//                 obs_global_y_temp.push_back(obs_region_y[j]);
//             }
//         }
//     }
//     cllction_data.obs_x_global.clear();
//     cllction_data.obs_y_global.clear();

//     cllction_data.obs_x_global = obs_global_x_temp;
//     cllction_data.obs_y_global = obs_global_y_temp;
// }

/* GETTER function */
uint8_t getNRobotClosestBall()
{
    uint8_t n_closest_ball = 0;
    int distance_closest = INT_MAX;
    for (uint8_t i = 1; i < N_ROBOT; i++)
    {
        if (isRobotReady(i) && pc2bs_msg[i].status_bola == 1)
        {
            int delta_distance = pythagoras(pc2bs_msg[i].bola_x, pc2bs_msg[i].bola_y, pc2bs_msg[i].pos_x, pc2bs_msg[i].pos_y);

            if (delta_distance < distance_closest)
            {
                distance_closest = delta_distance;
                n_closest_ball = i + 1;
            }
        }
    }

    // if there is no ball detected in robot main, then check keeper
    // if (n_closest_ball == 0 && isRobotReady(0) && pc2bs_msg[0].status_bola == 1)
    // {
    //     n_closest_ball = 1;
    // }

    return n_closest_ball;
}

uint8_t *getRobotTarget()
{
    /*
        n_target[0] -> shooter
        n_target[1] -> target
    */

    static uint8_t n_target[2] = {0, 0};
    for (uint8_t i = 1; i < N_ROBOT; i++)
    {
        if (pc2bs_msg[i].target_umpan > 0 && pc2bs_msg[i].target_umpan < 6)
        {
            n_target[0] = i + 1;
            n_target[1] = pc2bs_msg[i].target_umpan;
            break;
        }
    }
    return n_target;
}

uint8_t getNRobotCloser(uint8_t robot_ind)
{
    uint8_t n_robot_closer = 0;
    int distance_closer = INT_MAX;

    for (uint8_t i = 1; i < N_ROBOT; i++)
    {
        if (i != robot_ind && isRobotReady(i))
        {
            int distance = pythagoras(pc2bs_msg[robot_ind].pos_x, pc2bs_msg[robot_ind].pos_y, pc2bs_msg[i].pos_x, pc2bs_msg[i].pos_y);
            if (distance < distance_closer)
            {
                distance_closer = distance;
                n_robot_closer = i;
            }
        }
    }

    return n_robot_closer + 1;
}

// void setObsGroup()
// {
//     std::vector<int16_t> obs_angle_result;
//     std::vector<int16_t> obs_dist_result;
//     uint8_t dist_max = 100;
//     float_t angle_max = 6;
//     uint8_t counter_offset = 3;
//     int dist_back = 25;

//     for (uint8_t i = 0; i < N_ROBOT; i++)
//     {
//         std::vector<int16_t> obs_dist_dummy;
//         std::vector<int16_t> obs_angle_dummy;
//         std::vector<int16_t> obs_dist_temp;
//         std::vector<int16_t> obs_angle_temp;

//         // assign obs_dist & obs_angle
//         obs_dist_dummy = pc2bs_msg[i].obs_dist;

//         for (uint8_t j = 0; j < pc2bs_msg[i].obs_length; j++)
//         {
//             uint16_t angle = ceil(pc2bs_msg[i].obs_index[j] * 2.5);
//             obs_angle_dummy.push_back(angle);
//         }

//         // abs angle
//         for (int16_t &value : obs_angle_dummy)
//         {
//             if (value < 0)
//             {
//                 value = 360 + value;
//             }
//         }

//         std::vector<std::tuple<int16_t, int16_t, int16_t>> arr_temp;
//         std::vector<int16_t> sort_arr;

//         // sorted arr based on angle
//         for (uint8_t j = 0; j < obs_angle_dummy.size(); j++)
//         {
//             arr_temp.push_back(std::make_tuple(obs_angle_dummy[j], j, obs_dist_dummy[j]));
//         }
//         sort(arr_temp.begin(), arr_temp.end());

//         obs_dist_dummy.clear();
//         obs_angle_dummy.clear();
//         obs_angle_result.clear();
//         obs_dist_result.clear();
//         obs_dist_temp.clear();
//         obs_angle_temp.clear();

//         // update obs_dist_dummy based on sorted arr
//         for (auto &item : arr_temp)
//         {
//             obs_angle_dummy.push_back(std::get<0>(item));
//             obs_dist_dummy.push_back(std::get<2>(item));
//         }

//         // jika langsung assign error
//         int16_t prev_dist = 0;
//         int16_t prev_angle = 0;

//         if (!obs_dist_dummy.empty() && !obs_angle_dummy.empty())
//         {
//             prev_dist = obs_dist_dummy[0];
//             prev_angle = obs_angle_dummy[0];
//         }

//         int16_t dist;
//         int16_t angle;

//         bool status = false;
//         bool prev_status = false;
//         std::vector<int16_t> obs_status;

//         uint8_t start = 0;
//         uint8_t stop = 0;
//         uint8_t counter = 0;
//         uint8_t len_obs = pc2bs_msg[i].obs_length;

//         if (!obs_dist_dummy.empty() && !obs_angle_dummy.empty())
//         {
//             // diskontinu: arr[0] & arr[last]
//             int16_t dist_diff = abs(obs_dist_dummy[0] - obs_dist_dummy[obs_dist_dummy.size() - 1]);
//             int16_t angle_diff = abs(obs_angle_dummy[0] - (360 - obs_angle_dummy[obs_angle_dummy.size() - 1]));
//             angle_diff = angle_diff < 0 ? angle_diff * (-1) : angle_diff;

//             if ((dist_diff <= dist_max) && (angle_diff <= angle_max))
//             {
//                 uint8_t counter_up = 0;
//                 uint8_t counter_down = 0;
//                 uint8_t counter_mean = 0;
//                 uint8_t index = 0;

//                 // depan
//                 for (uint8_t i = 0; i < len_obs - 1; i++)
//                 {
//                     if (
//                         (abs(obs_dist_dummy[i] - obs_dist_dummy[i + 1]) > dist_max) ||
//                         (abs(obs_angle_dummy[i] - obs_angle_dummy[i + 1]) > angle_max))
//                     {
//                         counter_up++;
//                         break;
//                     }
//                     counter_up++;
//                 }

//                 // belakang
//                 if (counter_up < len_obs - 1)
//                 {
//                     for (uint8_t i = len_obs - 1; i >= 1; i--)
//                     {
//                         if (
//                             (abs(obs_dist_dummy[i] - obs_dist_dummy[i - 1]) > dist_max) ||
//                             (abs(obs_angle_dummy[i] - obs_angle_dummy[i - 1]) > angle_max))
//                         {
//                             counter_down++;
//                             break;
//                         }
//                         counter_down++;
//                     }
//                 }

//                 counter_mean = ceil((counter_up + counter_down) / 2);

//                 if (counter_mean > counter_up)
//                 {
//                     index = len_obs - (counter_mean - counter_up);
//                 }
//                 else
//                 {
//                     index = counter_up - counter_mean;
//                 }

//                 obs_angle_result.push_back(obs_angle_dummy[index]);
//                 obs_dist_result.push_back(obs_dist_dummy[index] + dist_back);

//                 // assign 9999
//                 for (uint8_t i = 0; i <= counter_up - 1; i++)
//                 {
//                     obs_angle_dummy[i] = 9999;
//                     obs_dist_dummy[i] = 9999;
//                 }

//                 for (uint8_t i = len_obs - 1; i >= len_obs - counter_down; i--)
//                 {
//                     for (uint8_t j = i; j < len_obs - 1; j++)
//                     {
//                         obs_angle_dummy[j] = obs_angle_dummy[j + 1];
//                         obs_dist_dummy[j] = obs_dist_dummy[j + 1];
//                     }
//                     obs_angle_dummy[len_obs - 1] = 9999;
//                     obs_dist_dummy[len_obs - 1] = 9999;
//                 }

//                 float temp_angle, temp_dist;

//                 for (uint8_t i = 0; i < len_obs - 1; i++)
//                 {
//                     for (uint8_t j = i + 1; j < len_obs; j++)
//                     {
//                         if (obs_angle_dummy[j] < obs_angle_dummy[i])
//                         {
//                             // Tukar nilai obs_angle_dummy
//                             temp_angle = obs_angle_dummy[i];
//                             obs_angle_dummy[i] = obs_angle_dummy[j];
//                             obs_angle_dummy[j] = temp_angle;

//                             // Tukar nilai obs_dist_dummy
//                             temp_dist = obs_dist_dummy[i];
//                             obs_dist_dummy[i] = obs_dist_dummy[j];
//                             obs_dist_dummy[j] = temp_dist;
//                         }
//                     }
//                 }
//             }

//             // angle & dist difference
//             for (uint8_t i = 1; i < obs_angle_dummy.size(); i++)
//             {
//                 if (obs_angle_dummy[i] != 9999 && obs_dist_dummy[i] != 9999)
//                 {
//                     prev_angle = obs_angle_dummy[i - 1];
//                     prev_dist = obs_dist_dummy[i - 1];
//                     angle = obs_angle_dummy[i];
//                     dist = obs_dist_dummy[i];
//                     obs_angle_temp.push_back(abs(angle - prev_angle));
//                     obs_dist_temp.push_back(abs(dist - prev_dist));
//                     prev_angle = obs_angle_dummy[i];
//                     prev_dist = obs_dist_dummy[i];
//                 }
//             }

//             // assign to temp when != 9999
//             if (
//                 obs_angle_dummy[0] != 9999 &&
//                 obs_angle_dummy[obs_angle_dummy.size() - 1] != 9999 &&
//                 obs_dist_dummy[0] != 9999 &&
//                 obs_dist_dummy[obs_dist_dummy.size() - 1] != 9999)
//             {
//                 obs_angle_temp.push_back(
//                     abs(
//                         obs_angle_dummy[0] - obs_angle_dummy[obs_angle_dummy.size() - 1]));
//                 obs_dist_temp.push_back(
//                     abs(
//                         obs_dist_dummy[0] - obs_dist_dummy[obs_dist_dummy.size() - 1]));
//             }

//             // check distance between
//             for (uint8_t i = 0; i < obs_angle_temp.size(); i++)
//             {
//                 prev_status = status;

//                 if (obs_angle_temp[i] <= angle_max && obs_dist_temp[i] <= dist_max)
//                 {
//                     obs_status.push_back(true);
//                     status = true;
//                 }
//                 else
//                 {
//                     obs_status.push_back(false);
//                     status = false;
//                 }

//                 if (!prev_status && status)
//                 {
//                     start = i;
//                 }
//                 if (prev_status && !status)
//                 {
//                     // ex: 0 1 1 1 -> start = 1; stop = 3;
//                     stop = i + 1;
//                 }

//                 if (i == obs_angle_temp.size() - 1)
//                 {
//                     if (prev_status == status)
//                     {
//                         stop = i + 1;
//                         status = false;
//                     }
//                 }

//                 if (prev_status && !status)
//                 {
//                     if (stop > start)
//                     {
//                         counter = stop - start;
//                         if (counter >= counter_offset)
//                         {
//                             int16_t dist_mean = 0;
//                             int16_t angle_mean = 0;
//                             for (uint8_t j = start; j <= stop; j++)
//                             {
//                                 if (j == stop)
//                                 {
//                                     dist_mean += obs_dist_dummy[0];
//                                     angle_mean += obs_angle_dummy[0];
//                                     continue;
//                                 }
//                                 dist_mean += obs_dist_dummy[j];
//                                 angle_mean += obs_angle_dummy[j];
//                             }
//                             dist_mean /= counter + 1;
//                             angle_mean /= counter + 1;

//                             obs_dist_result.push_back(dist_mean + dist_back);
//                             obs_angle_result.push_back(angle_mean);
//                         }
//                     }
//                 }
//             }

//             std::vector<int> obs_x;
//             std::vector<int> obs_y;

//             for (uint8_t j = 0; j < obs_angle_result.size(); j++)
//             {
//                 int dist = obs_dist_result[j];
//                 int angle = obs_angle_result[j];
//                 obs_x.push_back(getAngleToPosX(i, angle, dist));
//                 obs_y.push_back(getAngleToPosY(i, angle, dist));
//             }

//             switch (i)
//             {
//             case 0:
//                 entity_robot.group_obs_x_r1 = obs_x;
//                 entity_robot.group_obs_y_r1 = obs_y;
//                 break;
//             case 1:
//                 entity_robot.group_obs_x_r2 = obs_x;
//                 entity_robot.group_obs_y_r2 = obs_y;
//                 break;
//             case 2:
//                 entity_robot.group_obs_x_r3 = obs_x;
//                 entity_robot.group_obs_y_r3 = obs_y;
//                 break;
//             case 3:
//                 entity_robot.group_obs_x_r4 = obs_x;
//                 entity_robot.group_obs_y_r4 = obs_y;
//                 break;
//             case 4:
//                 entity_robot.group_obs_x_r5 = obs_x;
//                 entity_robot.group_obs_y_r5 = obs_y;
//                 break;
//             }
//         }
//     }
// };

int16_t getAngleToPosX(uint8_t robot_ind, int angle, int dist)
{
    return pc2bs_msg[robot_ind].pos_x + dist * cos((angle * M_PI / 180.0));
};

int16_t getAngleToPosY(uint8_t robot_ind, int angle, int dist)
{
    return pc2bs_msg[robot_ind].pos_y + dist * sin((angle * M_PI / 180.0));
};

/* Formula function */
int pythagoras(int x1, int y1, int x2, int y2)
{
    int x = x1 - x2;
    int y = y1 - y2;
    int z = sqrt(pow(x, 2) + pow(y, 2));
    return z;
}

bool isBallAppear()
{
    bool is_ball_appear = false;
    for (uint8_t i = 1; i < N_ROBOT; i++)
    {
        if (pc2bs_msg[i].status_bola == 1 && isRobotReady(i))
        {
            is_ball_appear = true;
        }
    }
    return is_ball_appear;
}

uint8_t *isBallCatched()
{
    static uint8_t is_ball_catched[2] = {0, 0};
    is_ball_catched[0] = 0;
    is_ball_catched[1] = 0;
    for (uint8_t i = 1; i < N_ROBOT; i++)
    {
        if (pc2bs_msg[i].status_bola == 2 && isRobotReady(i))
        {
            is_ball_catched[0] = 1;
            is_ball_catched[1] = i;
        }
    }
    return is_ball_catched;
}

uint8_t isConditionExist(int cond_number)
{
    uint8_t condition = 0;
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        if (pc2bs_msg[i].robot_condition == cond_number)
        {
            condition = 1;
        }
    }
    return condition;
}

uint8_t isRobotReady(uint8_t index_robot)
{
    uint8_t is_robot_ready = 0;
    if (entity_robot.is_active[index_robot] && fe2be_msg.status_control_robot[index_robot])
    // if (entity_robot.is_active[index_robot])
    {
        is_robot_ready = 1;
    }
    return is_robot_ready;
}