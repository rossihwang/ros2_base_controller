// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include <tuple>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace motion {

class Kinematic {
 public:
  Kinematic() = default;
  ~Kinematic() = default;
  virtual std::unique_ptr<std::vector<int32_t>> from_twist(const geometry_msgs::msg::Twist::SharedPtr msg) = 0;
  virtual nav_msgs::msg::Odometry::UniquePtr to_odometry(const std::vector<int32_t>& counts) = 0;
};

}  // namespace motion
