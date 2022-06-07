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
#include "universal_robot_ign/ign_joint_subscriber.hpp"

using namespace std;
using namespace universal_robot_ign;

IGNJointSubscriber::IGNJointSubscriber(const std::vector<std::string>& joint_names,
                                       const std::string& ign_topic){
    joint_names_ = joint_names;
    ign_topic_ = ign_topic;
    for (size_t i = 0; i < joint_names_.size(); i++) {
        joint_names_map_[joint_names_[i]] = i;
    }
}

void IGNJointSubscriber::start(){
    // Ignition node
//    ign_node_ = std::make_shared<ignition::transport::Node>();
    ignition::transport::Node ign_node_;
    //create ignition sub
    ign_node_.Subscribe(ign_topic_, &IGNJointSubscriber::ignJointStateCb, this);

    //init current_wrench_msg_
    for (auto i = 0u; i < joint_names_.size(); ++i) {
        current_joint_msg_.name.push_back(joint_names_[i]);
        current_joint_msg_.position.push_back(0);
        current_joint_msg_.velocity.push_back(0);
        current_joint_msg_.effort.push_back(0);
    }

    ignition::transport::waitForShutdown();

}

sensor_msgs::msg::JointState IGNJointSubscriber::getJointStateMsg()
{
    ignition::msgs::Model model_msg;
    {
        std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
        model_msg=current_ign_joint_msg_;
    }

    //create  JointState msg
    current_joint_msg_.header.stamp = rclcpp::Clock().now();
    current_joint_msg_.header.frame_id = model_msg.name();
    for(int i = 0; i < model_msg.joint_size() ; ++i){
        if (joint_names_map_.find(model_msg.joint(i).name()) != joint_names_map_.end()) {
            int idx=joint_names_map_[model_msg.joint(i).name()];
            current_joint_msg_.position[idx]=model_msg.joint(i).axis1().position();
            current_joint_msg_.velocity[idx]=model_msg.joint(i).axis1().velocity();
            current_joint_msg_.effort[idx]=model_msg.joint(i).axis1().force();
        }
    }

    return current_joint_msg_;
}

void IGNJointSubscriber::ignJointStateCb(const ignition::msgs::Model& msg)
{
    std::lock_guard<std::mutex> lock(current_joint_msg_mut_);
    current_ign_joint_msg_ = msg;
}