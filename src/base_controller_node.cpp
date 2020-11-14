#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <base_controller/base_controller.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<BaseController>("base_controller_node", rclcpp::NodeOptions());

  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;
  
  return 0;
}
