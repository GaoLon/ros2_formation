# ROS2 Formation

This repository contains simulation for swarm formation in ROS2

## How to build and RUN
Please follow the steps:
```
git clone https://github.com/GaoLon/ros2_formation.git
cd ros2_formation
colcon build --symlink-install 
ros2 launch ego_planner run_all.py
```

Then you can use `2DGoalPose` plugin test the planner

## TODO List
Add it into MBZIRC simulator

## NOTE
If when you build and get linker error as `LZ4....`:

You need add "/usr/lib/x86_64-linux-gnu/liblz4.so" to:
`ros2_formation/build/map_generator/CMakeFiles/random_forest.dir/link.txt` end but before `-ldl`
and
`ros2_formation/build/local_sensing/CMakeFiles/pcl_render_node.dir/link.txt` end but before `-ldl`
