cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
# ADD_COMPILE_OPTIONS(-std=c++11 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(quadrotor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(traj_opt REQUIRED)
find_package(traj_utils REQUIRED)
# find_package(cv_bridge REQUIRED)

include_directories( 
  SYSTEM 
  include 
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
ament_target_dependencies(ego_planner_node
  PUBLIC
  rclcpp
  rclpy
  std_msgs
  PCL
  pcl_conversions
  visualization_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  # cv_bridge
)

add_executable(traj_server src/traj_server.cpp)
ament_target_dependencies(traj_server
  PUBLIC
  rclcpp
  rclpy
  std_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  traj_utils
  traj_opt
)

install(TARGETS
  ego_planner_node
  traj_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
