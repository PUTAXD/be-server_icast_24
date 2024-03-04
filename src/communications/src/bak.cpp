#include "communications/comm_multicast.hpp"

Icast* icast = Icast::getInstance();
// Dictionary* dc = Dictionary::getInstance();
// Multicast_3* mc = Multicast_3::getInstance();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "comm_multicast");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);

    icast->init();

    if (!icast->mc.initialized()) {
        std::cout << "Multicast not ready" << std::endl;
        ros::shutdown();
        return 0;
    }

    pub_server2bs = nh.advertise<communications::server2bs>("/server2bs", 1);
    sub_bs2server = nh.subscribe("/bs2server", 1, bsToServerCallback);
    tim_routine = nh.createTimer(ros::Duration(1), routineCallback);

    spinner.spin();

    return 0;
}
s void bsToServerCallback(const communications::bs2server::ConstPtr& msg)
{
    command = msg->command;
}

void routineCallback(const ros::TimerEvent& event)
{
    // if (kbhit() > 0) {
    //     char key = std::cin.get();

    //     if (key == 's') {
    //         setDataToBeSend(dc);
    //     }
    // }

    printf("Routine\n");

    icast->update();

    size_t offset, size;
    icast->dc.getOffsetSize(3, "pos", offset, size);
    memcpy(&robot_pose, icast->dc.dictionary_data_.data() + offset, size);
    std::cout << "Robot Pose: " << robot_pose.x << " " << robot_pose.y << " " << robot_pose.theta << std::endl;
    icast->dc.getOffsetSize(3, "vel", offset, size);
    memcpy(&robot_vel, icast->dc.dictionary_data_.data() + offset, size);
    std::cout << "Robot Vel: " << robot_vel.x << " " << robot_vel.y << " " << robot_vel.theta << std::endl;
    icast->dc.getOffsetSize(3, "ball", offset, size);
    memcpy(&ball, icast->dc.dictionary_data_.data() + offset, size);
    std::cout << "Ball: " << ball.is_caught << " " << ball.is_visible << " " << ball.x << " " << ball.y << " " << ball.vx << " " << ball.vy << std::endl;
    pub_server2bs.publish(msg_server2bs);
}

void setDataToBeSend(Dictionary* dc_ptr)
{
    size_t offset, size;
    dc_ptr->getOffsetSize(1, "pos", offset, size);
    // std::cout << "Offset: " << offset << " Size: " << size << std::endl;

    struct data_tag {
        float pose[3] = { 1.1, 2098.2, 0.00 };
    } data_agent;

    std::memcpy(dc_ptr->dictionary_data_.data() + offset, &data_agent, size);

    dc_ptr->setResetUpdate(1, "pos", false, true);
}