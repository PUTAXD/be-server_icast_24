#include "communications/multiboost.h"
#include "communications/PC2BS.h"
#include "communications/BS2PC.h"
#include "basestation/EntityRobot.h"
#include "basestation/FE2BE.h"

#define N_ROBOT 5
#define LEN_MSG 256

communications::PC2BS pc2bs_msg;
basestation::EntityRobot entity_msg;
basestation::FE2BE fe2be_msg;

ros::Publisher pc2bs_pub[N_ROBOT];
ros::Subscriber bs2pc_sub;
ros::Subscriber entity_sub;
ros::Subscriber fe2be_sub;

ros::Timer timer_ack;

receiver *rcvr;
sender *sndr[N_ROBOT];

double time_kirim;
double time_terima;

int main(int argc, char *argv[])
{
    loadConfig();
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(4);

    ros::Timer timer_cllbck_rcv;
    ros::Timer timer_cllbck_snd;

    bs2pc_sub = n.subscribe("bs2pc", 1, cllbckSndMtcast);
    entity_sub = n.subscribe<basestation::EntityRobot>("entity_robot", 1,
                                                       [&](const typename basestation::EntityRobot::ConstPtr &msg)
                                                       {
                                                           entity_msg = *msg;
                                                       });
    fe2be_sub = n.subscribe<basestation::FE2BE>("fe2be", 1,
                                                [&](const typename basestation::FE2BE::ConstPtr &msg)
                                                {
                                                    fe2be_msg = *msg;
                                                });

    for (int i = 0; i < N_ROBOT; i++)
    {
        char str_topic[100];
        sprintf(str_topic, "pc2bs_r%d", i + 1);
        pc2bs_pub[i] = n.advertise<communications::PC2BS>(str_topic, 1);
    }

    boost::asio::io_context io_context;
    rcvr = new receiver(io_context);
    std::thread io_thread([&io_context]()
                          { io_context.run(); });

    if (is_multicast)
    {
        sndr[0] = new sender(io_context, boost::asio::ip::make_address(mtcast_address), multicast_port);
    }
    else
    {
        for (int i = 0; i < N_ROBOT; i++)
        {
            sndr[i] = new sender(io_context, boost::asio::ip::make_address(ip_robot[i]), unicast_port);
        }
        timer_ack = n.createTimer(ros::Duration(1), [&](const ros::TimerEvent &)
                                  {
                                      for (int i = 0; i < N_ROBOT; i++)
                                      {
                                          if (!entity_msg.is_active[i])
                                          {
                                              sndr[i]->sendACK();
                                          }
                                      } });
    }

    spinner.spin();

    io_context.stop();
    io_thread.join();

    return 0;
}

void loadConfig()
{
    // declare variable for config file
    char config_file[100];

    // get current dir
    std::string current_dir = ros::package::getPath("communications");
    sprintf(config_file, "%s/../config/setup.yaml", current_dir.c_str());

    // load config file
    YAML::Node config = YAML::LoadFile(config_file);

    char temp_ip_mtcast[100], temp_ip_robot[5][100];

    printf("\n========= LOADING CONFIG FILE ========\n\n");
    strcpy(temp_ip_mtcast, config["multicast_group"].as<std::string>().c_str());
    strcpy(temp_ip_robot[0], config["ip_robot_1"].as<std::string>().c_str());
    strcpy(temp_ip_robot[1], config["ip_robot_2"].as<std::string>().c_str());
    strcpy(temp_ip_robot[2], config["ip_robot_3"].as<std::string>().c_str());
    strcpy(temp_ip_robot[3], config["ip_robot_4"].as<std::string>().c_str());
    strcpy(temp_ip_robot[4], config["ip_robot_5"].as<std::string>().c_str());

    is_multicast = config["is_multicast"].as<bool>();
    mtcast_address = temp_ip_mtcast;
    multicast_port = config["multicast_port"].as<short>();
    unicast_port = config["unicast_port"].as<int>();
    for (int i = 0; i < 5; i++)
    {
        ip_robot[i] = temp_ip_robot[i];
    }

    // print config
    printf("is_multicast: %d\n", is_multicast);
    printf("port_mtcast: %d\n", multicast_port);
    printf("port_unicast: %d\n", unicast_port);
    printf("ip_mtcast: %s\n", mtcast_address.c_str());
    printf("ip_robot1: %s\n", ip_robot[0].c_str());
    printf("ip_robot2: %s\n", ip_robot[1].c_str());
    printf("ip_robot3: %s\n", ip_robot[2].c_str());
    printf("ip_robot4: %s\n", ip_robot[3].c_str());
    printf("ip_robot5: %s\n", ip_robot[4].c_str());
    printf("\n========= FINISH LOAD CONFIG =========\n\n");
}

void cllbckRcvMtcast(char *recv_buf)
{
    if ((recv_buf[3] > '0' && recv_buf[3] <= '5') && (recv_buf[0] == 'i' && recv_buf[1] == 't' && recv_buf[2] == 's'))
    {
        time_terima = ros::Time::now().toSec();

        uint8_t n_robot = recv_buf[3] - '0';

        int counter = 4;
        int data_size = 0;

        pc2bs_msg.mcast_delay = (uint64_t)((time_terima - time_kirim) * 1000000);

        data_size = sizeof(int64_t);
        memcpy(&pc2bs_msg.epoch, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.pos_x, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.pos_y, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.theta, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(uint8_t);
        memcpy(&pc2bs_msg.status_bola, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.bola_x, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.bola_y, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.robot_condition, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(uint8_t);
        memcpy(&pc2bs_msg.target_umpan, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(uint8_t);
        memcpy(&pc2bs_msg.index_point, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(uint8_t);
        memcpy(&pc2bs_msg.obs_length, recv_buf + counter, data_size);
        counter += data_size;

        int16_t obs_dist = 0;
        int16_t obs_index = 0;
        pc2bs_msg.obs_dist.clear();
        pc2bs_msg.obs_index.clear();

        uint8_t obs_length = pc2bs_msg.obs_length;
        for (int i = 0; i < obs_length; i++)
        {
            data_size = sizeof(int16_t);
            memcpy(&obs_dist, recv_buf + counter, data_size);
            counter += data_size;

            data_size = sizeof(int16_t);
            memcpy(&obs_index, recv_buf + counter, data_size);
            counter += data_size;

            pc2bs_msg.obs_dist.push_back(obs_dist);
            pc2bs_msg.obs_index.push_back(obs_index);
        }

        data_size = sizeof(float_t);
        memcpy(&pc2bs_msg.battery_health, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.goalkeeper_field_x, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.goalkeeper_field_y, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.bola_x_next, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.bola_y_next, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.pos_x_next, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.pos_y_next, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.pass_target_x, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(int16_t);
        memcpy(&pc2bs_msg.pass_target_y, recv_buf + counter, data_size);
        counter += data_size;

        data_size = sizeof(uint8_t);
        memcpy(&pc2bs_msg.pos_obs_length, recv_buf + counter, data_size);
        counter += data_size;

        int16_t pos_obs_x = 0;
        int16_t pos_obs_y = 0;
        pc2bs_msg.pos_obs_x.clear();
        pc2bs_msg.pos_obs_y.clear();

        uint8_t pos_obs_length = pc2bs_msg.pos_obs_length;
        for (int i = 0; i < pos_obs_length; i++)
        {
            data_size = sizeof(int16_t);
            memcpy(&pos_obs_x, recv_buf + counter, data_size);
            counter += data_size;

            data_size = sizeof(int16_t);
            memcpy(&pos_obs_y, recv_buf + counter, data_size);
            counter += data_size;

            pc2bs_msg.pos_obs_x.push_back(pos_obs_x);
            pc2bs_msg.pos_obs_y.push_back(pos_obs_y);
        }

        pc2bs_msg.n_robot = n_robot;

        pc2bs_pub[n_robot - 1].publish(pc2bs_msg);
    }
}

void cllbckSndMtcast(const communications::BS2PC::ConstPtr &msg)
{
    int counter = 0;
    int data_size = 0;

    char send_buf[LEN_MSG];

    data_size = 4;
    memcpy(send_buf, "its0", data_size);
    counter += data_size;

    data_size = sizeof(int8_t);
    memcpy(send_buf + counter, &msg->header_manual_and_calibration, data_size);
    counter += data_size;

    data_size = sizeof(int8_t);
    memcpy(send_buf + counter, &msg->command, data_size);
    counter += data_size;

    data_size = sizeof(int8_t);
    memcpy(send_buf + counter, &msg->style, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->ball_x_in_field, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->ball_y_in_field, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->target_manual_x, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->target_manual_y, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->target_manual_theta, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->offset_robot_x, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->offset_robot_y, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->offset_robot_theta, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->mux1, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->mux2, data_size);
    counter += data_size;

    data_size = sizeof(int16_t);
    memcpy(send_buf + counter, &msg->mux_bs_control, data_size);
    counter += data_size;

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        data_size = sizeof(uint8_t);
        memcpy(send_buf + counter, &msg->control_v_linear[i], data_size);
        counter += data_size;
    }

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        data_size = sizeof(uint8_t);
        memcpy(send_buf + counter, &msg->control_v_angular[i], data_size);
        counter += data_size;
    }

    for (uint8_t i = 0; i < N_ROBOT; i++)
    {
        data_size = sizeof(uint8_t);
        memcpy(send_buf + counter, &msg->control_power_kicker[i], data_size);
        counter += data_size;
    }

    data_size = sizeof(uint8_t);
    memcpy(send_buf + counter, &msg->passing_counter, data_size);
    counter += data_size;

    // for (uint8_t i = 0; i < 6; i++)
    // {
    //     data_size = sizeof(int16_t);
    //     memcpy(send_buf + counter, &msg->pos_obs[i], data_size);
    //     counter += data_size;
    // }

    send_buf[counter] = msg->index_obs[0];
    counter++;
    send_buf[counter] = msg->index_obs[1];
    counter++;
    send_buf[counter] = msg->index_obs[2];
    counter++;


    if (is_multicast)
    {
        sndr[0]->do_send(send_buf, counter);
        time_kirim = ros::Time::now().toSec();
    }
    else
    {
        // assign unicast data

        // status active
        for (int i = 0; i < N_ROBOT; i++)
        {
            data_size = sizeof(uint8_t);
            memcpy(send_buf + counter, &msg->status_active[i], data_size);
            counter += data_size;
        }

        // pos x
        for (int i = 0; i < N_ROBOT; i++)
        {
            data_size = sizeof(int16_t);
            memcpy(send_buf + counter, &msg->pos_x[i], data_size);
            counter += data_size;
        }

        // pos y
        for (int i = 0; i < N_ROBOT; i++)
        {
            data_size = sizeof(int16_t);
            memcpy(send_buf + counter, &msg->pos_y[i], data_size);
            counter += data_size;
        }

        // theta
        for (int i = 0; i < N_ROBOT; i++)
        {
            data_size = sizeof(int16_t);
            memcpy(send_buf + counter, &msg->theta[i], data_size);
            counter += data_size;
        }

        // robot condition
        for (int i = 0; i < N_ROBOT; i++)
        {
            data_size = sizeof(uint8_t);
            memcpy(send_buf + counter, &msg->robot_condition[i], data_size);
            counter += data_size;
        }

        // passing point x
        for (int i = 0; i < N_ROBOT; i++)
        {
            data_size = sizeof(uint8_t);
            memcpy(send_buf + counter, &msg->pass_target_x[i], data_size);
            counter += data_size;
        }

        // passing point y
        for (int i = 0; i < N_ROBOT; i++)
        {
            data_size = sizeof(uint8_t);
            memcpy(send_buf + counter, &msg->pass_target_y[i], data_size);
            counter += data_size;
        }

        // target field
        for (int i = 0; i < N_ROBOT; i++)
        {
            data_size = sizeof(uint8_t);
            memcpy(send_buf + counter, &msg->target_umpan[i], data_size);
            counter += data_size;
        }

        // send the data
        for (uint8_t i = 0; i < N_ROBOT; i++)
        {
            if (entity_msg.is_active[i] && fe2be_msg.status_control_robot[i])
            {
                sndr[i]->do_send(send_buf, counter);
            }
        }
    }
}