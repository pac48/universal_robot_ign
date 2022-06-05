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
#ifndef UNIVERSAL_ROBOT_IGN_WRENCH_PUBLISHER_H
#define UNIVERSAL_ROBOT_IGN_WRENCH_PUBLISHER_H

#include <ignition/transport/Node.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <unordered_map>

namespace universal_robot_ign {

class IGNWrenchPublisher {
public:
    IGNWrenchPublisher(const std::string& ign_topic);
    ~IGNWrenchPublisher() {};

    void setWrenchCb(const geometry_msgs::msg::Wrench::SharedPtr msg);

private:
    std::shared_ptr<ignition::transport::Node> ign_node_;
    // ros pub and sub
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr ros_cmd_joint_state_sub_;
    //ignition pub
    std::shared_ptr<ignition::transport::Node::Publisher> ign_cmd_wrench_pub_;
    // joint names and map
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, int> joint_names_map_;
};
}
#endif //UNIVERSAL_ROBOT_IGN_WRENCH_PUBLISHER_H