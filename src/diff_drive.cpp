// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <cmath>
#include <algorithm>
#include <base_controller/diff_drive.hpp>
#include <Eigen/Geometry>

namespace motion {

DiffDrive::DiffDrive() 
  : distance_to_icc_(0),
    theta_(0) {
  wheel_perimeter_ = M_PI * kWheelRadius;
  meter_per_counts_ = wheel_perimeter_ / kCountsPerRound;
  max_target_ = 1.2 * kControlPeriod / meter_per_counts_;  // maximum velocity 1.2m/s

  odom_ = std::make_shared<nav_msgs::msg::Odometry>();
  odom_->pose.pose.position.x = 0;
  odom_->pose.pose.position.y = 0;
  odom_->pose.pose.position.z = 0;
  odom_->pose.pose.orientation.x = 0;
  odom_->pose.pose.orientation.y = 0;
  odom_->pose.pose.orientation.z = 0;
  odom_->pose.pose.orientation.w = 0;
}

std::unique_ptr<std::vector<int32_t>> DiffDrive::from_twist(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (distance_to_icc_ == 0) {
    right_target_ = (msg->linear.x + (msg->angular.z * kDistanceOfWheels * 0.5)) * kControlPeriod / meter_per_counts_;
    left_target_ = (msg->linear.x - (msg->angular.z * kDistanceOfWheels * 0.5)) * kControlPeriod / meter_per_counts_;
  } else {
    // TODO
    right_target_ = 0;
    left_target_ = 0;
  }

  std::vector<int32_t> targets;

  right_target_ = std::clamp(right_target_, -max_target_, max_target_);
  left_target_ = std::clamp(left_target_, -max_target_, max_target_);

  targets.push_back(right_target_);
  targets.push_back(left_target_);

  return std::make_unique<std::vector<int32_t>>(targets);
}

nav_msgs::msg::Odometry::SharedPtr DiffDrive::to_odometry(const std::vector<int32_t>& counts) {
  using namespace Eigen;

  float vel_r = (counts[0] * meter_per_counts_) / kControlPeriod;
  float vel_l = (counts[1] * meter_per_counts_) / kControlPeriod;
  float w = 0;

  if (std::abs(vel_r - vel_l) < 0.01) {
    // linear velocity only
    float vel = (vel_r + vel_l) / 2;
    odom_->twist.twist.linear.x = vel * std::cos(theta_);
    odom_->twist.twist.linear.y = vel * std::sin(theta_);
    
    odom_->twist.twist.angular.x = 0; 
    odom_->twist.twist.angular.y = 0;
    odom_->twist.twist.angular.z = 0;
  } else {
    // angular only
    float radius = (kDistanceOfWheels * (vel_l + vel_r)) / (2 * (vel_r - vel_l));
    w = (vel_r - vel_l) / kDistanceOfWheels;
    
    odom_->twist.twist.angular.x = 0;
    odom_->twist.twist.angular.y = 0;
    odom_->twist.twist.angular.z = w;

    float v = w * radius;
    odom_->twist.twist.linear.x =  v * std::cos(theta_);
    odom_->twist.twist.linear.y =  v * std::sin(theta_);
  }
  odom_->pose.pose.position.x += odom_->twist.twist.linear.x * kControlPeriod;
  odom_->pose.pose.position.y += odom_->twist.twist.linear.y * kControlPeriod;

  Quaterniond q;
  // theta_ += std::clamp(w * kControlPeriod, -M_PI, M_PI);
  theta_ += w * kControlPeriod;
  if (M_PI <= theta_) {
    theta_ -= M_PI;
  } else if (theta_ < -M_PI){
    theta_ += M_PI;
  }
  q = AngleAxisd(theta_, Vector3d::UnitZ());
  odom_->pose.pose.orientation.x = q.x();
  odom_->pose.pose.orientation.y = q.y();
  odom_->pose.pose.orientation.z = q.z();
  odom_->pose.pose.orientation.w = q.w();

  return odom_;
}

}  // namespace motion