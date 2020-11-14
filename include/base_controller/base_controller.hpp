// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "serial/serial.h"
#include "pigeon/pigeon.h"
#include <base_controller/diff_drive.hpp>

using pigeon::Pigeon;
using motion::DiffDrive;

class BaseController: public rclcpp::Node {

private:
  Pigeon pg_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; 
  std::shared_ptr<serial::Serial> serial_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string port_;
  int baud_;
  DiffDrive drive_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void log_pigeon_callback(const uint8_t *data, uint16_t length);
  void wheels_counter_callback(const uint8_t *data, uint16_t length);
  void create_parameter();
  // bool handle_parameter(rclcpp::Parameter const &param);
  rcl_interfaces::msg::SetParametersResult update_callback(const std::vector<rclcpp::Parameter>& parameters);
public:
  BaseController(const std::string& name, const rclcpp::NodeOptions& options);
  ~BaseController();
};