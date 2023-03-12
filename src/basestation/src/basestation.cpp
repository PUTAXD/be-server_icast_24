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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "basestation");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i + 1);
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

void cllbckSndBS2PC(const ros::TimerEvent &event)
{
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

    entity_robot.is_active[robot_ind] = 1;
    std::chrono::seconds time_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
    entity_robot.time_coming[robot_ind] = time_now.count();
}

/* Update/setter Data Global */

void setNRobotData()
{
    std::chrono::seconds time_now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
    uint8_t timeout = 2;

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        if (entity_robot.is_active[i])
        {
            if (time_now.count() - entity_robot.time_coming[i] > timeout)
            {
                entity_robot.is_active[i] = 0;
            }
            else
            {
                setNRobotFriend(i);
            }
        }
    }

    uint8_t n_robot_active = 0;
    for (uint8_t i = 1; i < N_ROBOT; i++)
    {
        if (entity_robot.is_active[i])
        {
            n_robot_active++;
        }
    }

    cllction_data.n_robot_aktif = n_robot_active;
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
    }
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
        uint8_t LEN_ARR_ROBOT_DEKAT_BOLA = sizeof(cllction_data.n_array_robot_dekat_bola) / sizeof(cllction_data.n_array_robot_dekat_bola[0]);
        uint8_t counter_role = 1;

        for (uint8_t i = 0; i < LEN_ARR_ROBOT_DEKAT_BOLA; i++)
        {
            int8_t INDEX_ROBOT = cllction_data.n_array_robot_dekat_bola[i] - 1;

            if (!isRobotReady(i))
            {
                entity_robot.role[i] = 0;
            }

            if (INDEX_ROBOT > 0 && isRobotReady(INDEX_ROBOT))
            {
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
            entity_robot.role[0] = 0;
        }
    }
};

void setMux1()
{
    uint8_t CONVERSION = 6;
    uint16_t mux = 0;

    mux += cllction_data.n_robot_aktif;
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

    mux |= fe2bs_msg.status_control_robot[0] * 0b00001;
    mux |= fe2bs_msg.status_control_robot[1] * 0b00010;
    mux |= fe2bs_msg.status_control_robot[2] * 0b00100;
    mux |= fe2bs_msg.status_control_robot[3] * 0b01000;
    mux |= fe2bs_msg.status_control_robot[4] * 0b10000;

    cllction_data.mux_bs_control_robot = mux;
};

void setObs()
{
    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        uint8_t LEN_OBS = pc2bs_msg->obs_length;
        std::vector<int16_t> obs_x;
        std::vector<int16_t> obs_y;
        obs_x.resize(LEN_OBS);
        obs_y.resize(LEN_OBS);

        for (uint8_t j = 0; j < LEN_OBS; j++)
        {
            uint16_t dist = pc2bs_msg->obs_dist[j];
            uint16_t angle = pc2bs_msg->obs_index[j] * 2.5;
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

/* Process Data which need sub function */
void setNRobotFriend(uint8_t robot_ind)
{
    uint8_t n_robot_friend = 0;
    if (cllction_data.n_robot_aktif <= 1)
    {
        entity_robot.n_robot_teman[robot_ind] = 0;
    }
    else
    {
        entity_robot.n_robot_teman[robot_ind] = getNRobotCloser(robot_ind);
    }
}

/* GETTER function */
uint8_t getNRobotCloser(uint8_t robot_ind)
{
    uint8_t n_robot_closer = 0;
    int distance_closer = INT_MAX;

    for (uint8_t i = 1; i < N_ROBOT; i++)
    {
        if (i != robot_ind && entity_robot.is_active[i])
        {
            int distance = pythagoras(pc2bs_msg[robot_ind].pos_x, pc2bs_msg[robot_ind].pos_y, pc2bs_msg[i].pos_x, pc2bs_msg[i].pos_y);
            if (distance < distance_closer)
            {
                distance_closer = distance;
                n_robot_closer = i;
            }
        }
    }

    return n_robot_closer;
}

void getObsGroup()
{
    std::vector<int16_t> obs_angle_result;
    std::vector<int16_t> obs_dist_result;
    uint8_t dist_max = 100;
    float_t angle_max = 2.5;
    uint8_t counter_offset = 2;

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        std::vector<int16_t> obs_dist_dummy;
        std::vector<int16_t> obs_angle_dummy;
        std::vector<int16_t> obs_dist_temp;
        std::vector<int16_t> obs_angle_temp;

        obs_dist_dummy.clear();
        obs_angle_dummy.clear();

        // assign obs_dist & obs_angle
        obs_dist_dummy = pc2bs_msg->obs_dist;
        obs_angle_dummy.resize(pc2bs_msg->obs_length);

        for (uint8_t j = 0; j < pc2bs_msg->obs_length; j++)
        {
            uint16_t angle = pc2bs_msg->obs_index[j] * 2.5;
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

        // diskontinu: arr[0] & arr[last]
        if (!obs_dist_dummy.empty() && !obs_angle_dummy.empty())
        {
            if (abs(obs_angle_dummy[0] - obs_angle_dummy[obs_angle_dummy.size() - 1]) <= angle_max &&
                abs(obs_dist_dummy[0] - obs_dist_dummy[obs_dist_dummy.size() - 1]) <= dist_max)
            {
                uint8_t counter_up = 0;
                uint8_t counter_down = 0;
                uint8_t counter_mean = 0;
                uint8_t index = 0;
                uint8_t len_obs = pc2bs_msg->obs_length;

                // // depan
                for (uint8_t i = 0; i < len_obs; i++)
                {
                    if (
                        abs(obs_angle_dummy[i] - obs_angle_dummy[i + 1]) > angle_max &&
                        abs(obs_dist_dummy[i] - obs_dist_dummy[i + 1]) > dist_max)
                    {
                        counter_up++;
                        break;
                    }
                    counter_up++;
                }

                // // belakang
                if (counter_up < len_obs - 1)
                {
                    for (uint8_t i = len_obs - 1; i >= 1; i--)
                    {
                        if (abs(obs_angle_dummy[i] - obs_angle_dummy[i - 1]) > angle_max &&
                            abs(obs_dist_dummy[i] - obs_dist_dummy[i - 1]) > dist_max)
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
                obs_dist_result.push_back(obs_dist_dummy[index] - 20);

                for (uint8_t i = 0; i <= counter_up - 1; i++)
                {
                    obs_angle_dummy[i] = 9999;
                    obs_dist_dummy[i] = 9999;
                }

                for (uint8_t i = len_obs - 1; i >= len_obs - counter_down; i--)
                {
                    obs_angle_dummy[i] = 9999;
                    obs_dist_dummy[i] = 9999;
                }
            }

            // angle & dist difference
            for (uint8_t i = 1; i < obs_angle_dummy.size(); i++)
            {
                if (obs_angle_dummy[i] != 9999 && obs_dist_dummy[i] != 9999)
                {
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
                    stop = i;
                }

                if (i == obs_angle_temp.size() - 1)
                {
                    bool flag = true;
                    for (uint8_t k = 0; k < obs_status.size(); k++)
                    {
                        if (obs_status[k] == false)
                        {
                            flag = false;
                        }
                    }

                    if (flag)
                    {
                        status = false;
                        stop = i;
                    }
                }

                if (prev_status && !status)
                {
                    if (stop > start)
                    {
                        counter = stop - start;
                        if (counter >= counter_offset)
                        {
                            uint8_t dist_mean = 0;
                            uint8_t angle_mean = 0;
                            for (uint8_t j = start; j <= stop; j++)
                            {
                                j =
                                    j +
                                    (j < 0) * obs_angle_dummy.size() -
                                    (j >= obs_angle_dummy.size()) * obs_angle_dummy.size();
                                dist_mean += obs_dist_dummy[j];
                                angle_mean += obs_angle_dummy[j];
                            }
                            dist_mean /= counter + 1;
                            angle_mean /= counter + 1;

                            obs_dist_result.push_back(dist_mean + 10);
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
                // double dist_rad = dist * cos((angle - 90) * M_PI / 180.0);
                // int x = round(dist_rad * 100) / 100.0;
                int16_t x = round((dist * cos((angle - 90) * M_PI / 180.0)) * 100) / 100.0;
                int16_t y = round((dist * sin((angle - 90) * M_PI / 180.0)) * 100) / 100.0;
                obs_x.push_back(x);
                obs_y.push_back(y);
            }

            switch (i)
            {
            case 0:
                entity_robot.group_obs_x_1 = obs_x;
                entity_robot.group_obs_y_1 = obs_y;
                break;
            case 1:
                entity_robot.group_obs_x_2 = obs_x;
                entity_robot.group_obs_y_2 = obs_y;
                break;
            case 2:
                entity_robot.group_obs_x_3 = obs_x;
                entity_robot.group_obs_y_3 = obs_y;
                break;
            case 3:
                entity_robot.group_obs_x_4 = obs_x;
                entity_robot.group_obs_y_4 = obs_y;
                break;
            case 4:
                entity_robot.group_obs_x_5 = obs_x;
                entity_robot.group_obs_y_5 = obs_y;
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
    if (entity_robot.is_active[index_robot] && fe2bs_msg.status_control_robot[index_robot])
    {
        is_robot_ready = 1;
    }
    return is_robot_ready;
}