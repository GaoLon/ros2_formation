cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)

set(msg_files
  "msg/DataDisp.msg"
  "msg/PolyTraj.msg"
  "msg/Assignment.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories( 
    ${rclcpp_INCLUDE_DIRS}
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS} 
)

add_library(Traj_utils
  src/planning_visualization.cpp 
)

ament_target_dependencies(Traj_utils
    PUBLIC
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
) 

ament_export_targets(Traj_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS Traj_utils
  EXPORT Traj_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()