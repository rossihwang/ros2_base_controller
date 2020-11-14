// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <cmath>
#include <algorithm>
#include <base_controller/diff_drive.hpp>

namespace motion {

DiffDrive::DiffDrive() 
  : distance_to_icc_(0) {
  wheel_perimeter_ = M_PI * kWheelRadius;
  meter_per_counts_ = wheel_perimeter_ / kCountsPerRound;
  max_target_ = 1.2 * kControlPeriod / meter_per_counts_;  // maximum velocity 1.2m/s
}

std::unique_ptr<std::vector<int32_t>> DiffDrive::from_twist(const geometry_msgs::msg::Twist::SharedPtr msg) {
  int32_t right_target, left_target;
  if (distance_to_icc_ == 0) {
    right_target = (msg->linear.x + (msg->angular.z * kDistanceOfWheels * 0.5)) * kControlPeriod / meter_per_counts_;
    left_target = (msg->linear.x - (msg->angular.z * kDistanceOfWheels * 0.5)) * kControlPeriod / meter_per_counts_;
  } else {
    // TODO
    right_target = 0;
    left_target = 0;
  }

  std::vector<int32_t> targets;

  targets.push_back(std::clamp(right_target, -max_target_, max_target_));
  targets.push_back(std::clamp(left_target, -max_target_, max_target_));

  return std::make_unique<std::vector<int32_t>>(targets);
}

nav_msgs::msg::Odometry::UniquePtr DiffDrive::to_odometry(const std::vector<int32_t>& counts) {

  nav_msgs::msg::Odometry::UniquePtr odom;
  // TODO
  (void)counts;
  return odom;
}

}  // namespace motion