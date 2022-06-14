cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(traj_opt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)

find_package(plan_env REQUIRED)
find_package(traj_utils REQUIRED)
find_package(swarm_graph REQUIRED)
find_package(path_searching REQUIRED)


include_directories( 
    SYSTEM 
    include 
    ${rclcpp_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(Traj_opt 
    src/poly_traj_optimizer.cpp
    )

ament_target_dependencies(Traj_opt
    PUBLIC
    rclcpp
    std_msgs
    PCL
    pcl_conversions
    plan_env
    path_searching
    swarm_graph
    traj_utils
) 

ament_export_targets(Traj_opt HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    std_msgs
    PCL
    pcl_conversions
    plan_env
    path_searching
    swarm_graph
    traj_utils
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS Traj_opt
  EXPORT Traj_opt
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
