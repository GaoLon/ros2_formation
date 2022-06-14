cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(fake_drone)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)

include_directories( 
    SYSTEM 
    include 
    ${rclcpp_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

add_executable (fake_drone src/poscmd_2_odom.cpp)
ament_target_dependencies(fake_drone
  rclcpp
  rclpy
  std_msgs
  visualization_msgs
  nav_msgs
  quadrotor_msgs
)

install(TARGETS
  fake_drone
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
