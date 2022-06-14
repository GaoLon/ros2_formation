cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
# ADD_COMPILE_OPTIONS(-std=c++11 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories( 
  SYSTEM 
  include 
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable (random_forest src/random_forest_sensing.cpp )

ament_target_dependencies(random_forest
  PUBLIC
  rclcpp
  rclpy
  std_msgs
  geometry_msgs
  PCL
  pcl_conversions
)

install(TARGETS
  random_forest
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()