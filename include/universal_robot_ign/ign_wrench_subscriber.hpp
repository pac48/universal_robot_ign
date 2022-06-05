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
#ifndef UNIVERSAL_ROBOT_IGN_FORCE_STATE_PUBLISHER_H
#define UNIVERSAL_ROBOT_IGN_FORCE_STATE_PUBLISHER_H

#include <ignition/transport/Node.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <map>

namespace universal_robot_ign {

class IGNWrenchSubscriber {
public:
    IGNWrenchSubscriber(const std::string& ign_topic);
    ~IGNWrenchSubscriber() {};
    void start();

    geometry_msgs::msg::Wrench getWrenchMsg();

private:
    //callback for Ignition
    void ignWrenchCb(const ignition::msgs::Wrench& msg);

private:
//    rclcpp::Node::SharedPtr nh_;
//    std::shared_ptr<ignition::transport::Node> ign_node_;
    // ros pub and sub
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr ros_wrench_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    // joint names and map
    std::string ign_topic_;
    //Wrench info received form Ignition
    ignition::msgs::Wrench current_ign_joint_msg_;
    geometry_msgs::msg::Wrench current_wrench_msg_;
    std::mutex current_wrench_msg_mut_;
};

}

#endif //UNIVERSAL_ROBOT_IGN_FORCE_STATE_PUBLISHER_H