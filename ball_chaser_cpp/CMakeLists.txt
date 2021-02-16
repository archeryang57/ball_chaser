cmake_minimum_required(VERSION 3.5)
project(ball_chaser_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# added by manual
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(drive_bot src/drive_bot.cpp)
ament_target_dependencies(drive_bot rclcpp geometry_msgs robot_interfaces)

add_executable(estimate_color src/estimate_color.cpp)
ament_target_dependencies(estimate_color rclcpp sensor_msgs robot_interfaces)

add_executable(process_image src/process_image.cpp)
ament_target_dependencies(process_image rclcpp sensor_msgs robot_interfaces)

install(TARGETS
  drive_bot
  estimate_color
  process_image
  DESTINATION lib/${PROJECT_NAME}
)
# finished add

ament_package()
