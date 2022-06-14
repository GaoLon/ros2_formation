from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    drone_id = DeclareLaunchArgument('drone_id', default_value='drone_0')
    init_x = DeclareLaunchArgument('init_x', default_value='-26.0')
    init_y = DeclareLaunchArgument('init_y', default_value='0.0')
    init_z = DeclareLaunchArgument('init_z', default_value='0.5')
    map_size_x = DeclareLaunchArgument('map_size_x', default_value='70.0')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='30.0')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='3.0')
    return LaunchDescription([
        init_x,
        init_y,
        init_z,
        map_size_x,
        map_size_y,
        map_size_z,
        drone_id,
        # Trajectory Server
        Node(
            package='ego_planner',
            executable='traj_server',
            namespace=LaunchConfiguration('drone_id'),
            name='traj_server',
            parameters=[
                {'traj_server/time_forward': 1.0}
            ],
            output='screen'
        ),
        # Fake UAV
        Node(
            package='fake_drone',
            executable='fake_drone',
            namespace=LaunchConfiguration('drone_id'),
            name='fake_drone',
            parameters=[
                {'init_x': LaunchConfiguration('init_x')},
                {'init_y': LaunchConfiguration('init_y')},
                {'init_z': LaunchConfiguration('init_z')},
            ],
            output='screen'
        ),
        # Fake Camera
        Node(
            package='local_sensing',
            executable='pcl_render_node',
            namespace=LaunchConfiguration('drone_id'),
            name='pcl_render_node',
            remappings=[
                ('global_map', '/map_generator/global_cloud'),
            ],
            parameters=[
                {'map/x_size': LaunchConfiguration('map_size_x')},
                {'map/y_size': LaunchConfiguration('map_size_y')},
                {'map/z_size': LaunchConfiguration('map_size_z')},
            ],
            output='screen'
        ),
        # Planner
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ego_planner'), 'launch'),
                '/run_planner.py']),
            launch_arguments={
                'drone_id': LaunchConfiguration('drone_id'),
                'map_size_x': LaunchConfiguration('map_size_x'),
                'map_size_y': LaunchConfiguration('map_size_y'),
                'map_size_z': LaunchConfiguration('map_size_z'),
                }.items(),)
    ])