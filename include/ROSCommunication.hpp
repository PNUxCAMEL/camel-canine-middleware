//
// Created by jh on 25. 2. 3.
//

#ifndef ROSCOMMUNICATION_HPP
#define ROSCOMMUNICATION_HPP

#include <Eigen/Dense>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <canine_msgs/msg/canine_state.hpp>
#include <canine_msgs/msg/canine_command.hpp>
#include "SharedMemory.hpp"

class ROSCommunication : public rclcpp::Node
{
public:
  ROSCommunication();
  void timer_callback();
  void package_canine_state_msg(canine_msgs::msg::CANINEState& msg);
  void topic_callback_canine_command(const canine_msgs::msg::CANINECommand::SharedPtr msg) const;

private:
  SharedMemory* sharedMemory;
  rclcpp::TimerBase::SharedPtr timer_canine_states;
  rclcpp::Publisher<canine_msgs::msg::CANINEState>::SharedPtr publisher_canine_states;
  rclcpp::Subscription<canine_msgs::msg::CANINECommand>::SharedPtr subscription_canine_command;


};



#endif //ROSCOMMUNICATION_HPP

