cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
# ADD_COMPILE_OPTIONS(-std=c++11 )
# ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)

include_directories( 
    SYSTEM 
    include 
    ${rclcpp_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library(Path_searching 
    src/dyn_a_star.cpp
    )

ament_target_dependencies(Path_searching
    PUBLIC
    plan_env
) 

ament_export_targets(Path_searching HAS_LIBRARY_TARGET)
ament_export_dependencies(
    plan_env
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS Path_searching
  EXPORT Path_searching
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
