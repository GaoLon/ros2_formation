from click import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ego_planner = get_package_share_directory('ego_planner')
    launch_list = list()
    # init_x = ['-26.0', '-23.4', '-26.0', '-28.6', '-28.6', '-26.0', '-23.4']
    # init_y = ['0.0', '-1.5', '-3.0', '-1.5', '1.5', '3.0', '1.5']
    # init_x = ['-26.0', '-23.4']
    # init_y = ['0.0', '-1.5']
    init_x = ['-26.0']
    init_y = ['0.0']

    # Map
    launch_list.append(Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        parameters=[
            {'map/x_size': 34.0},
            {'map/y_size': 15.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},
            {'map/obs_num': 60},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.5},
            {'ObstacleShape/lower_hei': 2.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 20},
            {'ObstacleShape/radius_l': 1.0},
            {'ObstacleShape/radius_h': 1.2},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 3.0},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 0.8},
        ],
        output='screen'
    ))
    # RViz
    launch_list.append(DeclareLaunchArgument('rviz', 
        default_value='true', description='Open RViz.'))
    launch_list.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(ego_planner, 'rviz', 'default.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    ))
    # Drones
    for i in range(len(init_x)):
        launch_list.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ego_planner'), 'launch'),
                '/run_one.py']),
            launch_arguments={
                'drone_id': 'drone_'+str(i),
                'init_x': init_x[i],
                'init_y': init_y[i],
                'init_z': '0.5',
                }.items(),))

    return LaunchDescription(launch_list)