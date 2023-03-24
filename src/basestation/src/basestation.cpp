#include "voronoi/voronoi.h"
#include "basestation/basestation.h"

#define N_ROBOT 5
#define DEG2RAD 0.017452925

ros::Subscriber pc2bs[N_ROBOT];
ros::Subscriber fe2be_sub;

ros::Publisher bs2pc_pub;
ros::Publisher cllction_pub;
ros::Publisher entity_robot_pub;

ros::Timer timer_cllbck_bs2pc;
ros::Timer timer_update_data;

communications::BS2PC bs2pc_msg;
communications::PC2BS pc2bs_msg[N_ROBOT];
basestation::FE2BE fe2be_msg;
basestation::Collection cllction_data;
basestation::EntityRobot entity_robot;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "basestation");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);

    fe2be_sub = n.subscribe("ui2server", 1000, cllbckRcvFE2BE);
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i + 1);
        pc2bs[i] = n.subscribe(str_topic, 1000, cllbckRcvPC2BS);
    }
    bs2pc_pub = n.advertise<communications::BS2PC>("bs2pc", 1000);
    entity_robot_pub = n.advertise<basestation::EntityRobot>("entity_robot", 1000);
    cllction_pub = n.advertise<basestation::Collection>("collection", 1000);

    timer_cllbck_bs2pc = n.createTimer(ros::Duration(0.025), cllbckSndBS2PC);
    timer_update_data = n.createTimer(ros::Duration(0.001), cllbckUpdateData);

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
    setMuxNRobotCloser();
    setMuxNRobotControlledBS();
    setObs();
    getObsGroup();
    setCounterPass();
    setVoronoi();
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

    entity_robot.is_active[robot_ind] = true;
    std::chrono::seconds time_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
    entity_robot.time_coming[robot_ind] = time_now.count();

    //--->Voronoi Pose
    voronoi_robot_pos_x[robot_ind] = msg->pos_x;
    voronoi_robot_pos_y[robot_ind] = msg->pos_y;
    voronoi_is_ready[robot_ind] = entity_robot.is_active[robot_ind];
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        voronoi_status_control_robot[i] = status_control_robot[i];
    }
}

void cllbckRcvFE2BE(const basestation::FE2BE::ConstPtr &msg)
{
    fe2be_msg.header_manual = msg->header_manual;
    fe2be_msg.command = msg->command;
    fe2be_msg.style = msg->style;
    fe2be_msg.connect_refbox = msg->connect_refbox;
    fe2be_msg.n_robot_manual = msg->n_robot_manual;
    fe2be_msg.target_manual_x = msg->target_manual_x;
    fe2be_msg.target_manual_y = msg->target_manual_y;
    fe2be_msg.target_manual_theta = msg->target_manual_theta;
    fe2be_msg.odometry_offset_robot_x = msg->odometry_offset_robot_x;
    fe2be_msg.odometry_offset_robot_y = msg->odometry_offset_robot_y;
    fe2be_msg.odometry_offset_robot_theta = msg->odometry_offset_robot_theta;
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        fe2be_msg.trim_kecepatan_robot[i] = msg->trim_kecepatan_robot[i];
    }
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        fe2be_msg.trim_kecepatan_sudut_robot[i] = msg->trim_kecepatan_sudut_robot[i];
    }
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        fe2be_msg.trim_penendang_robot[i] = msg->trim_penendang_robot[i];
    }
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        fe2be_msg.status_control_robot[i] = msg->status_control_robot[i];
        status_control_robot[i] = msg->status_control_robot[i];
    }
}

void cllbckSndBS2PC(const ros::TimerEvent &event)
{
    entity_robot_pub.publish(entity_robot);
    cllction_pub.publish(cllction_data);
    bs2pc_pub.publish(bs2pc_msg);
}

/* Update/setter Data Global */

void setNRobotData()
{
    std::chrono::seconds time_now =
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
    uint8_t timeout = 2;

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
                                            pc2bs_msg[0].pos_x,
                                            pc2bs_msg[0].pos_y);
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

    setSortingBallDistance();
};

void setRole()
{
    // 0 Goal Keeper
    // 1 attacker
    // 2 defender 1
    // 3 assist
    // 4 defender 2

    if (!isCondition20Exist())
    {
        uint8_t LEN_ARR_ROBOT_DEKAT_BOLA = sizeof(cllction_data.n_array_robot_dekat_bola) /
                                           sizeof(cllction_data.n_array_robot_dekat_bola[0]);
        uint8_t counter_role = 1;

        for (uint8_t i = 0; i < LEN_ARR_ROBOT_DEKAT_BOLA; i++)
        {
            if (cllction_data.n_array_robot_dekat_bola[i] != 0)
            {
                int8_t INDEX_ROBOT = cllction_data.n_array_robot_dekat_bola[i];
                switch (counter_role)
                {
                case 1:
                    entity_robot.role[INDEX_ROBOT] = 1;
                    break;
                case 2:
                    entity_robot.role[INDEX_ROBOT] = 3;
                    break;
                case 3:
                    entity_robot.role[INDEX_ROBOT] = 2;
                    break;
                case 4:
                    entity_robot.role[INDEX_ROBOT] = 4;
                    break;
                }
                counter_role++;
            }
            if (!isRobotReady(i))
            {
                entity_robot.role[i] = 0;
            }
        }
        entity_robot.role[0] = 0;
    }
};

void setMux1()
{
    uint8_t CONVERSION = 6;
    uint16_t mux = 0;

    mux += cllction_data.n_robot_ready;
    mux += cllction_data.n_robot_dekat_bola * CONVERSION;
    mux += cllction_data.n_robot_dapat_bola * CONVERSION * CONVERSION;
    mux += cllction_data.n_robot_umpan * CONVERSION * CONVERSION * CONVERSION;
    mux += cllction_data.n_robot_terima * CONVERSION * CONVERSION * CONVERSION * CONVERSION;

    cllction_data.mux1 = mux;
};

void setMux2()
{
    uint8_t CONVERSION = 6;
    uint16_t mux = 0;

    mux += entity_robot.role[0];
    mux += entity_robot.role[1] * CONVERSION;
    mux += entity_robot.role[2] * CONVERSION * CONVERSION;
    mux += entity_robot.role[3] * CONVERSION * CONVERSION * CONVERSION;
    mux += entity_robot.role[4] * CONVERSION * CONVERSION * CONVERSION * CONVERSION;

    cllction_data.mux2 = mux;
};

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

void setObs()
{
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        uint8_t LEN_OBS = pc2bs_msg[i].obs_length;
        std::vector<int16_t> obs_x;
        std::vector<int16_t> obs_y;
        obs_x.resize(LEN_OBS);
        obs_y.resize(LEN_OBS);

        for (uint8_t j = 0; j < LEN_OBS; j++)
        {
            uint16_t dist = pc2bs_msg[i].obs_dist[j];
            uint16_t angle = pc2bs_msg[i].obs_index[j] * 2.5;
            int16_t x = dist * cos(((angle - 90) * M_PI) / 180);
            int16_t y = dist * sin(((angle - 90) * M_PI) / 180);
            obs_x[j] = x;
            obs_y[j] = y;
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
    /*
        commented variable has been proceeding before
    */
    // bs2pc_msg.ball_x_in_field;
    // bs2pc_msg.ball_y_in_field;
    // bs2pc_msg.target_manual_x;
    // bs2pc_msg.target_manual_y;
    // bs2pc_msg.target_manual_theta;
    // bs2pc_msg.mux1;
    // bs2pc_msg.mux2;
    // bs2pc_msg.mux_bs_control;
    // bs2pc_msg.passing_counter;

    bs2pc_msg.header_manual_and_calibration = 10;
    bs2pc_msg.command = fe2be_msg.command;
    bs2pc_msg.style = fe2be_msg.style;
    bs2pc_msg.offset_robot_x = fe2be_msg.odometry_offset_robot_x;
    bs2pc_msg.offset_robot_y = fe2be_msg.odometry_offset_robot_y;
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

void setVoronoi()
{
    try
    {
        ProcessVoronoiDiagrams();
        if (!output.empty())
        {
            cllction_data.voronoi_start_points_x.resize(output.size());
            cllction_data.voronoi_start_points_y.resize(output.size());
            cllction_data.voronoi_end_points_x.resize(output.size());
            cllction_data.voronoi_end_points_y.resize(output.size());
            for (int i = 0; i < output.size(); i++)
            {
                point p0 = output[i]->start;
                point p1 = output[i]->end;
                if (p0.x == p0.x && p0.y == p0.y && p1.x == p1.x && p1.y == p1.y && p0.x != INFINITY && p0.y != INFINITY && p1.x != INFINITY && p1.y != INFINITY && abs(p0.x - p1.x) < 1200 && abs(p0.y - p1.y) < 1200)
                {
                    cllction_data.voronoi_start_points_x[i] = (int)p0.x;
                    cllction_data.voronoi_start_points_y[i] = (int)p0.y;
                    cllction_data.voronoi_end_points_x[i] = (int)p1.x;
                    cllction_data.voronoi_end_points_y[i] = (int)p1.y;
                }
            }
        }
        ClearOutput();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Voronoi Exception: %s", e.what());
    }
    catch (ros::Exception &e)
    {
        ROS_ERROR("ROS Exception In Voronoi: %s", e.what());
    }
    catch (...)
    {
        ROS_ERROR("Voronoi Exception: Unknown");
    }
}

/* GETTER function */
uint8_t getNRobotClosestBall()
{
    uint8_t n_closest_ball = 0;
    int distance_closest = INT_MAX;
    for (uint8_t i = 0; i < N_ROBOT; i++)
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
    if (n_closest_ball == 0 && isRobotReady(0) && pc2bs_msg[0].status_bola == 1)
    {
        n_closest_ball = 1;
    }

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

void setObsGroup()
{
    std::vector<int16_t> obs_angle_result;
    std::vector<int16_t> obs_dist_result;
    uint8_t dist_max = 100;
    float_t angle_max = 2.5;
    uint8_t counter_offset = 2;

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        uint8_t len_obs;
        len_obs = pc2bs_msg[i].obs_length;
        std::vector<int16_t> obs_dist_dummy;
        std::vector<uint8_t> obs_angle_dummy;

        obs_dist_dummy.clear();
        obs_angle_dummy.clear();

        static uint8_t prev_obs_state;
        static uint8_t obs_state;
        static uint8_t obs_start_idx;
        static uint8_t obs_end_idx;
        static float obs_angle;
        static float obs_x_field_buffer[144];
        static float obs_y_field_buffer[144];
        uint8_t obs_cnt_idx;
        uint8_t obs_cnt;
        uint8_t idx;
        float obs_x_buffer;
        float obs_y_buffer;
        std::vector<float> obs_dist_buffer;
        std::vector<uint8_t> obs_idx_buffer;

        obs_dist_dummy = pc2bs_msg[i].obs_dist;
        obs_angle_dummy = pc2bs_msg[i].obs_index;

        prev_obs_state = 0;
        obs_cnt_idx = 0;
        obs_cnt = 0;

        for (int16_t j = 0; j < len_obs; j++)
        {
            idx = j + (j < 0) * len_obs - (j >= len_obs) * len_obs;
            obs_state = (obs_dist_dummy[idx] <= dist_max && i != len_obs && obs_dist_dummy[idx] > 0);

            if (prev_obs_state == 0 && obs_state == 1)
            {
                obs_start_idx = idx; // get start index

                // record current val
                obs_x_field_buffer[obs_cnt_idx] = pc2bs_msg->pos_x + obs_dist_dummy[idx] * cos(idx * 2.5 * DEG2RAD);
                obs_y_field_buffer[obs_cnt_idx] = pc2bs_msg->pos_y + obs_dist_dummy[idx] * sin(idx * 2.5 * DEG2RAD);

                obs_cnt_idx++;
            }
            else if (prev_obs_state == 1 && obs_state == 1 && obs_cnt_idx > 0)
            {
                // record current val
                obs_x_field_buffer[obs_cnt_idx] = pc2bs_msg->pos_x + obs_dist_dummy[idx] * cos(idx * 2.5 * DEG2RAD);
                obs_y_field_buffer[obs_cnt_idx] = pc2bs_msg->pos_y + obs_dist_dummy[idx] * sin(idx * 2.5 * DEG2RAD);

                obs_cnt_idx++;
            }
            else if (prev_obs_state == 1 && obs_state == 0 && obs_cnt_idx > 0)
            {

                obs_end_idx = idx - 1;                            // get end idx
                obs_angle = (obs_end_idx + obs_start_idx) * 1.25; // Get angle (degrees)

                // Get mean
                for (uint8_t j = 0; j < obs_cnt_idx; j++)
                {
                    obs_x_buffer += (int)obs_x_field_buffer[j];
                    obs_y_buffer += (int)obs_y_field_buffer[j];
                }
                obs_x_buffer /= obs_cnt_idx;
                obs_y_buffer /= obs_cnt_idx;

                // Get center of obs
                obs_x_buffer += 25 * cos(obs_angle * DEG2RAD);
                obs_y_buffer += 25 * sin(obs_angle * DEG2RAD);

                // Save to buffer
                obs_dist_buffer.push_back(pythagoras(pc2bs_msg->bola_x, pc2bs_msg->bola_y, obs_x_buffer, obs_y_buffer));
                obs_idx_buffer.push_back((obs_end_idx + obs_start_idx) * 0.5);

                obs_cnt_idx = 0; // Reset counter
                obs_cnt++;       // Update total obs
            }
        }
    }
}

void getObsGroup()
{
    std::vector<int16_t> obs_angle_result;
    std::vector<int16_t> obs_dist_result;
    uint8_t dist_max = 100;
    float_t angle_max = 3;
    uint8_t counter_offset = 3;

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        std::vector<int16_t> obs_dist_dummy;
        std::vector<int16_t> obs_angle_dummy;
        std::vector<int16_t> obs_dist_temp;
        std::vector<int16_t> obs_angle_temp;

        obs_dist_dummy.clear();
        obs_angle_dummy.clear();

        // assign obs_dist & obs_angle
        obs_dist_dummy = pc2bs_msg[i].obs_dist;
        obs_angle_dummy.resize(pc2bs_msg[i].obs_length);

        for (uint8_t j = 0; j < pc2bs_msg[i].obs_length; j++)
        {
            uint16_t angle = ceil(pc2bs_msg[i].obs_index[j] * 2.5);
            obs_angle_dummy[j] = angle;
        }

        // abs angle
        for (int16_t &value : obs_angle_dummy)
        {
            if (value < 0)
            {
                value = 360 + value;
            }
        }

        std::vector<std::tuple<int16_t, int16_t, int16_t>> arr_temp;
        std::vector<int16_t> sort_arr;

        // sorted arr based on angle
        for (uint8_t j = 0; j < obs_angle_dummy.size(); j++)
        {
            arr_temp.push_back(std::make_tuple(obs_angle_dummy[j], j, obs_dist_dummy[j]));
        }
        sort(arr_temp.begin(), arr_temp.end());

        obs_dist_dummy.clear();
        obs_angle_dummy.clear();
        obs_angle_result.clear();
        obs_dist_result.clear();
        obs_dist_temp.clear();
        obs_angle_temp.clear();

        // update obs_dist_dummy based on sorted arr
        for (auto &item : arr_temp)
        {
            obs_angle_dummy.push_back(std::get<0>(item));
            obs_dist_dummy.push_back(std::get<2>(item));
        }

        // jika langsung assign error
        int16_t prev_dist = 0;
        int16_t prev_angle = 0;

        if (!obs_dist_dummy.empty() && !obs_angle_dummy.empty())
        {
            prev_dist = obs_dist_dummy[0];
            prev_angle = obs_angle_dummy[0];
        }

        int16_t dist;
        int16_t angle;

        bool status = false;
        bool prev_status = false;
        std::vector<int16_t> obs_status;

        uint8_t start = 0;
        uint8_t stop = 0;
        uint8_t counter = 0;
        uint8_t len_obs = pc2bs_msg[i].obs_length;

        if (!obs_dist_dummy.empty() && !obs_angle_dummy.empty())
        {
            // diskontinu: arr[0] & arr[last]
            int16_t dist_diff = abs(obs_dist_dummy[0] - obs_dist_dummy[obs_dist_dummy.size() - 1]);
            int16_t angle_diff = abs(obs_angle_dummy[0] - (360 - obs_angle_dummy[obs_angle_dummy.size() - 1]));
            angle_diff < 0 ? angle_diff * (-1) : angle_diff;

            if ((dist_diff <= dist_max) && (angle_diff <= angle_max))
            {
                uint8_t counter_up = 0;
                uint8_t counter_down = 0;
                uint8_t counter_mean = 0;
                uint8_t index = 0;

                // depan
                for (uint8_t i = 0; i < len_obs - 1; i++)
                {
                    if (
                        (abs(obs_dist_dummy[i] - obs_dist_dummy[i + 1]) > dist_max) ||
                        (abs(obs_angle_dummy[i] - obs_angle_dummy[i + 1]) > angle_max))
                    {
                        counter_up++;
                        break;
                    }
                    counter_up++;
                }

                // belakang
                if (counter_up < len_obs - 1)
                {
                    for (uint8_t i = len_obs - 1; i >= 1; i--)
                    {
                        if (
                            (abs(obs_dist_dummy[i] - obs_dist_dummy[i - 1]) > dist_max) ||
                            (abs(obs_angle_dummy[i] - obs_angle_dummy[i - 1]) > angle_max))
                        {
                            counter_down++;
                            break;
                        }
                        counter_down++;
                    }
                }

                counter_mean = ceil((counter_up + counter_down) / 2);

                if (counter_mean > counter_up)
                {
                    index = len_obs - (counter_mean - counter_up);
                }
                else
                {
                    index = counter_up - counter_mean;
                }

                obs_angle_result.push_back(obs_angle_dummy[index]);
                obs_dist_result.push_back(obs_dist_dummy[index] - 15);

                // assign 9999
                for (uint8_t i = 0; i <= counter_up - 1; i++)
                {
                    obs_angle_dummy[i] = 9999;
                    obs_dist_dummy[i] = 9999;
                }

                for (uint8_t i = len_obs - 1; i >= len_obs - counter_down; i--)
                {
                    for (uint8_t j = i; j < len_obs - 1; j++)
                    {
                        obs_angle_dummy[j] = obs_angle_dummy[j + 1];
                        obs_dist_dummy[j] = obs_dist_dummy[j + 1];
                    }
                    obs_angle_dummy[len_obs - 1] = 9999;
                    obs_dist_dummy[len_obs - 1] = 9999;
                }
            }

            float temp_angle, temp_dist;

            for (uint8_t i = 0; i < len_obs - 1; i++)
            {
                for (uint8_t j = i + 1; j < len_obs; j++)
                {
                    if (obs_angle_dummy[j] < obs_angle_dummy[i])
                    {
                        // Tukar nilai obs_angle_dummy
                        temp_angle = obs_angle_dummy[i];
                        obs_angle_dummy[i] = obs_angle_dummy[j];
                        obs_angle_dummy[j] = temp_angle;

                        // Tukar nilai obs_dist_dummy
                        temp_dist = obs_dist_dummy[i];
                        obs_dist_dummy[i] = obs_dist_dummy[j];
                        obs_dist_dummy[j] = temp_dist;
                    }
                }
            }

            // angle & dist difference
            for (uint8_t i = 1; i < obs_angle_dummy.size(); i++)
            {
                if (obs_angle_dummy[i] != 9999 && obs_dist_dummy[i] != 9999)
                {
                    prev_angle = obs_angle_dummy[i - 1];
                    prev_dist = obs_dist_dummy[i - 1];
                    angle = obs_angle_dummy[i];
                    dist = obs_dist_dummy[i];
                    obs_angle_temp.push_back(abs(angle - prev_angle));
                    obs_dist_temp.push_back(abs(dist - prev_dist));
                    prev_angle = obs_angle_dummy[i];
                    prev_dist = obs_dist_dummy[i];
                }
            }

            // assign to temp when != 9999
            if (
                obs_angle_dummy[0] != 9999 &&
                obs_angle_dummy[obs_angle_dummy.size() - 1] != 9999 &&
                obs_dist_dummy[0] != 9999 &&
                obs_dist_dummy[obs_dist_dummy.size() - 1] != 9999)
            {
                obs_angle_temp.push_back(
                    abs(
                        obs_angle_dummy[0] - obs_angle_dummy[obs_angle_dummy.size() - 1]));
                obs_dist_temp.push_back(
                    abs(
                        obs_dist_dummy[0] - obs_dist_dummy[obs_dist_dummy.size() - 1]));
            }

            // check distance between
            for (uint8_t i = 0; i < obs_angle_temp.size(); i++)
            {
                prev_status = status;

                if (obs_angle_temp[i] <= angle_max && obs_dist_temp[i] <= dist_max)
                {
                    obs_status.push_back(true);
                    status = true;
                }
                else
                {
                    obs_status.push_back(false);
                    status = false;
                }

                if (!prev_status && status)
                {
                    start = i;
                }
                if (prev_status && !status)
                {
                    // ex: 0 1 1 1 -> start = 1; stop = 3;
                    stop = i + 1;
                }

                if (i == obs_angle_temp.size() - 1)
                {
                    if (prev_status == status)
                    {
                        stop = i + 1;
                        status = false;
                    }
                }

                if (prev_status && !status)
                {
                    if (stop > start)
                    {
                        counter = stop - start;
                        if (counter >= counter_offset)
                        {
                            int16_t dist_mean = 0;
                            int16_t angle_mean = 0;
                            for (uint8_t j = start; j < stop; j++)
                            {
                                dist_mean += obs_dist_dummy[j];
                                angle_mean += obs_angle_dummy[j];
                            }
                            dist_mean /= counter;
                            angle_mean /= counter;

                            obs_dist_result.push_back(dist_mean + 5);
                            obs_angle_result.push_back(angle_mean);
                        }
                    }
                }
            }

            std::vector<int16_t> obs_x;
            std::vector<int16_t> obs_y;
            obs_x.clear();
            obs_y.clear();

            for (uint8_t j = 0; j < obs_angle_result.size(); j++)
            {
                int16_t dist = obs_dist_result[j];
                int16_t angle = obs_angle_result[j];
                int16_t x = round((dist * cos((angle - 90) * M_PI / 180.0)) * 100) / 100.0;
                int16_t y = round((dist * sin((angle - 90) * M_PI / 180.0)) * 100) / 100.0;
                obs_x.push_back(x);
                obs_y.push_back(y);
            }

            switch (i)
            {
            case 0:
                entity_robot.group_obs_x_r1 = obs_x;
                entity_robot.group_obs_y_r1 = obs_y;
                break;
            case 1:
                entity_robot.group_obs_x_r2 = obs_x;
                entity_robot.group_obs_y_r2 = obs_y;
                break;
            case 2:
                entity_robot.group_obs_x_r3 = obs_x;
                entity_robot.group_obs_y_r3 = obs_y;
                break;
            case 3:
                entity_robot.group_obs_x_r4 = obs_x;
                entity_robot.group_obs_y_r4 = obs_y;
                break;
            case 4:
                entity_robot.group_obs_x_r5 = obs_x;
                entity_robot.group_obs_y_r5 = obs_y;
                break;
            }
        }
    }
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
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        if (pc2bs_msg[i].status_bola == 1)
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
        if (pc2bs_msg[i].status_bola == 2)
        {
            is_ball_catched[0] = 1;
            is_ball_catched[1] = i;
        }
    }
    return is_ball_catched;
}

uint8_t isCondition20Exist()
{
    uint8_t is_condition_20_exist = 0;
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        if (pc2bs_msg[i].robot_condition == 20)
        {
            is_condition_20_exist = 1;
        }
    }
    return is_condition_20_exist;
}

uint8_t isRobotReady(uint8_t index_robot)
{
    uint8_t is_robot_ready = 0;
    if (entity_robot.is_active[index_robot] && fe2be_msg.status_control_robot[index_robot])
    {
        is_robot_ready = 1;
    }
    return is_robot_ready;
}