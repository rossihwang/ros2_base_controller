cmake_minimum_required(VERSION 3.5)
project(base_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(NANO_PB_SRC 3rdparty/nanopb/src/pb_common.c 3rdparty/nanopb/src/pb_decode.c 3rdparty/nanopb/src/pb_encode.c)
set(MSG_SRC shared/msg/wheels_counter.pb.c 
            shared/msg/imu.pb.c 
            shared/msg/logging.pb.c 
            shared/msg/pwm_ctrl.pb.c 
            shared/msg/twist.pb.c 
            shared/msg/Vector3.pb.c
            shared/pigeon/msg_def.cpp)
add_executable(base_controller_node src/base_controller.cpp src/base_controller_node.cpp src/diff_drive.cpp ${NANO_PB_SRC} ${MSG_SRC})
target_include_directories(base_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  shared
  3rdparty/nanopb/include)

ament_target_dependencies(base_controller_node
  rclcpp
  serial
  geometry_msgs
  nav_msgs)

install(TARGETS base_controller_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
