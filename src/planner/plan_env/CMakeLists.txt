cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
# ADD_COMPILE_OPTIONS(-std=c++11 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)

include_directories( 
    SYSTEM 
    include 
    ${rclcpp_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(Plan_env 
    src/grid_map.cpp 
    src/raycast.cpp
    )

ament_target_dependencies(Plan_env
    PUBLIC
    rclcpp
    std_msgs
    OpenCV
    cv_bridge
    PCL
    pcl_conversions
) 

ament_export_targets(Plan_env HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    std_msgs
    OpenCV
    cv_bridge
    PCL
    pcl_conversions
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS Plan_env
  EXPORT Plan_env
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()