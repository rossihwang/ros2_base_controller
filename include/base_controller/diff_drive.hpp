// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include <base_controller/kinematic.hpp>

namespace motion {

class DiffDrive: public Kinematic {
  constexpr static float kCountsPerRound = 1560;
  constexpr static float kDistanceOfWheels = 0.15;  //m
  constexpr static float kControlPeriod = 0.1;  // s
  constexpr static float kWheelRadius = 0.0325;  // m
  int32_t max_target_;
  float meter_per_counts_;
  float wheel_perimeter_;
  float distance_to_icc_;
  int32_t right_target_;
  int32_t left_target_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  float theta_;
  
 public:
  DiffDrive();
  ~DiffDrive() = default;
  std::unique_ptr<std::vector<int32_t>> from_twist(const geometry_msgs::msg::Twist::SharedPtr msg);
  nav_msgs::msg::Odometry::SharedPtr to_odometry(const std::vector<int32_t>& counts);
  std::tuple<int32_t, int32_t> get_targets() {
    return std::make_tuple(right_target_, left_target_);
  }
};

}  // namespace motion