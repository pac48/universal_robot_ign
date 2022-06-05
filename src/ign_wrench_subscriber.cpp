/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include "universal_robot_ign/ign_wrench_subscriber.hpp"

using namespace std;
using namespace universal_robot_ign;

IGNWrenchSubscriber::IGNWrenchSubscriber(const std::string& ign_topic){
    ign_topic_ = ign_topic;
}

void IGNWrenchSubscriber::start(){
    // Ignition node
//    ign_node_ = std::make_shared<ignition::transport::Node>();
    ignition::transport::Node ign_node_;
    //create ignition sub
    ign_node_.Subscribe(ign_topic_, &IGNWrenchSubscriber::ignWrenchCb, this);

    current_wrench_msg_.force.z = 0;
    current_wrench_msg_.force.x = 0;
    current_wrench_msg_.force.y = 0;
    current_wrench_msg_.torque.x = 0;
    current_wrench_msg_.torque.y = 0;
    current_wrench_msg_.torque.z = 0;

    ignition::transport::waitForShutdown();

}

geometry_msgs::msg::Wrench IGNWrenchSubscriber::getWrenchMsg()
{
    ignition::msgs::Wrench wrench_msg;
    {
        std::lock_guard<std::mutex> lock(current_wrench_msg_mut_);
        wrench_msg = current_ign_joint_msg_;
    }

    current_wrench_msg_.force.x = wrench_msg.force().x();
    current_wrench_msg_.force.y = wrench_msg.force().y();
    current_wrench_msg_.force.z = wrench_msg.force().z();

    current_wrench_msg_.torque.x = wrench_msg.torque().x();
    current_wrench_msg_.torque.y = wrench_msg.torque().y();
    current_wrench_msg_.torque.z = wrench_msg.torque().z();

    return current_wrench_msg_;
}

void IGNWrenchSubscriber::ignWrenchCb(const ignition::msgs::Wrench& msg)
{
    std::lock_guard<std::mutex> lock(current_wrench_msg_mut_);
    current_ign_joint_msg_ = msg;
}