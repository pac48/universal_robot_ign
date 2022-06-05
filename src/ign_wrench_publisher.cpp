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
#include "universal_robot_ign/ign_wrench_publisher.hpp"

using namespace std;
using namespace universal_robot_ign;

IGNWrenchPublisher::IGNWrenchPublisher(const std::string& ign_topic)
{
    // Ignition node
    ign_node_ = std::make_shared<ignition::transport::Node>();

    //create ignition pub
    ign_cmd_wrench_pub_ = std::make_shared<ignition::transport::Node::Publisher>(
                ign_node_->Advertise<ignition::msgs::Wrench>(ign_topic));

}

void IGNWrenchPublisher::setWrenchCb(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    ignition::msgs::Wrench wrench_msg;
    auto force = new ignition::msgs::Vector3d();
    auto torque = new ignition::msgs::Vector3d();

    force->set_x(msg->force.x);
    force->set_y(msg->force.y);
    force->set_z(msg->force.z);

    torque->set_x(msg->force.x);
    torque->set_y(msg->force.y);
    torque->set_z(msg->force.z);

    wrench_msg.set_allocated_force(force);
    wrench_msg.set_allocated_torque(torque);

    ign_cmd_wrench_pub_->Publish(wrench_msg);

}
