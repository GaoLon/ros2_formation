cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(swarm_graph)

# there are some bugs if use release
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories( 
    ${rclcpp_INCLUDE_DIRS}
    SYSTEM 
    include 
    ${Eigen3_INCLUDE_DIRS} 
)

add_library(Swarm_graph
  src/swarm_graph.cpp
)

ament_target_dependencies(Swarm_graph
    PUBLIC
    rclcpp
    std_msgs
) 

ament_export_targets(Swarm_graph HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS Swarm_graph
  EXPORT Swarm_graph
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
