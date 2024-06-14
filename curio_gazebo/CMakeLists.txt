cmake_minimum_required(VERSION 3.8)
project(curio_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(curio_description REQUIRED)
find_package(control_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(curio_controller src/curio_controller.cpp)
ament_target_dependencies(curio_controller
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_geometry_msgs
  trajectory_msgs
  control_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

install(
  TARGETS
    curio_controller
  DESTINATION
    lib/${PROJECT_NAME}
)


ament_package()
