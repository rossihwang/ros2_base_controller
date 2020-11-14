
// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <base_controller/base_controller.hpp>

#include <sstream>
#include <cstdlib>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

BaseController::BaseController(const std::string& name, const rclcpp::NodeOptions& options)
  : Node(name, options) {
  
  create_parameter();

  serial_ptr_ = std::make_shared<serial::Serial>(port_, baud_, serial::Timeout::simpleTimeout(3000));
  if (!serial_ptr_->isOpen()) {
    RCLCPP_ERROR(get_logger(), "failed to open %s:%d", port_.c_str(), baud_);
    std::exit(EXIT_FAILURE);
  }
  pg_.create_subscriber(MessageId::LOG, std::bind(&BaseController::log_pigeon_callback, this, _1, _2));
  pg_.create_subscriber(MessageId::WHL_CNTR, std::bind(&BaseController::wheels_counter_callback, this, _1, _2));

  pg_.register_read(std::bind(static_cast<size_t (serial::Serial::*)(uint8_t*, size_t)>(&serial::Serial::read), serial_ptr_, _1, _2));
  pg_.register_write(std::bind(static_cast<size_t (serial::Serial::*)(const uint8_t*, size_t)>(&serial::Serial::write), serial_ptr_, _1, _2));

  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 
    10, 
    std::bind(&BaseController::twist_callback, this, _1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  
  timer_ = this->create_wall_timer(10ms, [&](){this->pg_.poll();});
}

BaseController::~BaseController() = default;

void BaseController::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  WheelsCounter message = WheelsCounter_init_zero;
  
  auto targets = drive_.from_twist(msg);
  message.right = (*targets)[0];
  message.left = (*targets)[1];
  pg_.publish<WheelsCounter>(MessageId::WHL_CNTR, message);
}

void BaseController::log_pigeon_callback(const uint8_t *data, uint16_t length) {
  Log message = Log_init_zero;
  static std::array<std::string, 4> level_string = {"DEBUG", "INFO", "WARN", "ERROR"};

  bool status;
  pb_istream_t stream = pb_istream_from_buffer(data, length);
  status = pb_decode(&stream, Log_fields, &message);
  if (status) {
    std::string level = level_string[message.level];
    std::stringstream ss;
    ss << level << ": " << std::string(message.log_message);
    RCLCPP_INFO(get_logger(), ss.str());
  } else {
    RCLCPP_WARN(get_logger(), "log decode failed");
  }
}

void BaseController::wheels_counter_callback(const uint8_t *data, uint16_t length) {
  WheelsCounter message = WheelsCounter_init_zero;

  bool status;
  pb_istream_t stream = pb_istream_from_buffer(data, length);
  status = pb_decode(&stream, WheelsCounter_fields, &message);
  if (status) {
    RCLCPP_INFO(get_logger(), "right: %d, left: %d", message.right, message.left);
  } else {
    RCLCPP_WARN(get_logger(), "Encoder Decode failed");
  }

  nav_msgs::msg::Odometry::UniquePtr odom;
  std::vector counts = {message.right, message.left};
  odom = drive_.to_odometry(counts);
  odom_pub_->publish(*odom);
}

void BaseController::create_parameter() {
  port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
  baud_ = declare_parameter<int>("baud", 115200);

  // set_on_parameters_set_callback (
  //   [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
  //     auto result = rcl_interfaces::msg::SetParametersResult();
  //     result.successful = true;
  //     for (auto const &p : parameters) {
  //       result.successful &= handle_parameter(p);
  //     }
  //     return result;
  //   });
  param_cb_ = add_on_set_parameters_callback(std::bind(&BaseController::update_callback, this, _1));
}

// bool BaseController::handle_parameter(rclcpp::Parameter const &param) {
//   if (param.get_name() == "port") {
//     port_ = param.as_string();
//   } else if (param.get_name() == "baud") {
//     baud_ = param.as_int();
//   } else {
//     return false;
//   }
//   return true;
// }

rcl_interfaces::msg::SetParametersResult BaseController::update_callback(const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter& param: parameters) {
    if (param.get_name() == "port") {
      port_ = param.as_string();
    } else if (param.get_name() == "baud") {
      baud_ = param.as_int();
    }
  }
  return result;
}