from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
import launch_yaml

def generate_launch_description():
    drone_id = DeclareLaunchArgument('drone_id', default_value='drone_0')
    map_size_x = DeclareLaunchArgument('map_size_x', default_value='70.0')
    map_size_y = DeclareLaunchArgument('map_size_y', default_value='30.0')
    map_size_z = DeclareLaunchArgument('map_size_z', default_value='3.0')
    max_vel = DeclareLaunchArgument('max_vel', default_value='1.5')
    max_acc = DeclareLaunchArgument('max_acc', default_value='8.0')
    planning_horizon = DeclareLaunchArgument('planning_horizon', default_value='7.5')
    return LaunchDescription([
        map_size_x,
        map_size_y,
        map_size_z,
        drone_id,
        max_vel,
        max_acc,
        planning_horizon,
        # Planner Node
        Node(
            package='ego_planner',
            executable='ego_planner_node',
            namespace=LaunchConfiguration('drone_id'),
            name='ego_planner_node',
            parameters=[
                # fsm
                {'fsm/flight_type': 3},
                {'fsm/thresh_replan_time': 1.0},
                {'fsm/thresh_no_replan_meter': 1.0},
                {'fsm/planning_horizon': LaunchConfiguration('planning_horizon')},
                {'fsm/planning_horizen_time': 3.0},
                {'fsm/replan_trajectory_time': 0.1},
                {'fsm/emergency_time': 1.0},
                {'fsm/realworld_experiment': False},
                {'fsm/fail_safe': True},

                # grid map
                {'grid_map/resolution': 0.1},
                {'grid_map/map_size_x': LaunchConfiguration('map_size_x')},
                {'grid_map/map_size_y': LaunchConfiguration('map_size_y')},
                {'grid_map/map_size_z': LaunchConfiguration('map_size_z')},
                {'grid_map/local_update_range_x': 5.5},
                {'grid_map/local_update_range_y': 5.5},
                {'grid_map/local_update_range_z': 4.5},
                {'grid_map/obstacles_inflation': 0.1},
                {'grid_map/local_map_margin': 10.0},
                {'grid_map/ground_height': -0.01},
                {'grid_map/virtual_ceil_height': 3.0},
                {'grid_map/visualization_truncate_height': 2.8},
                {'grid_map/show_occ_time': False},
                {'grid_map/pose_type': 1},
                {'grid_map/frame_id': 'world'},
                {'grid_map/local_bound_inflate': 0.0},
                {'grid_map/show_esdf_time': False},
                {'grid_map/esdf_slice_height': 0.3},

                # manager
                {'manager/max_vel': LaunchConfiguration('max_vel')},
                {'manager/max_acc': LaunchConfiguration('max_acc')},
                {'manager/control_points_distance': 0.4},
                {'manager/polyTraj_piece_length': 2.0},
                {'manager/feasibility_tolerance': 0.05},
                {'manager/planning_horizon': LaunchConfiguration('planning_horizon')},
                {'manager/use_distinctive_trajs': False},
                {'manager/drone_id': LaunchConfiguration('drone_id')},
                {'global_goal/swarm_scale': 2.0},
                {'global_goal/relative_pos_0': [0.0, 0.0, 0.0]},
                {'global_goal/relative_pos_1': [1.7321, -1.0, 0.0]},
                {'global_goal/relative_pos_2': [0.0, -2.0, 0.0]},
                {'global_goal/relative_pos_3': [-1.7321, -1.0, 0.0]},
                {'global_goal/relative_pos_4': [-1.7321, 1.0, 0.0]},
                {'global_goal/relative_pos_5': [0.0, 2.0, 0.0]},
                {'global_goal/relative_pos_6': [1.7321, 1.0, 0.0]},

                # optimization
                {'optimization/constrain_points_perPiece': 3},
                {'optimization/max_vel': LaunchConfiguration('max_vel')},
                {'optimization/max_acc': LaunchConfiguration('max_acc')},
                {'optimization/record_opt': True},
                {'optimization/formation_type': 1},
                {'optimization/weight_obstacle': 50000.0},
                {'optimization/weight_swarm': 50000.0},
                {'optimization/weight_feasibility': 10000.0},
                {'optimization/weight_sqrvariance': 10000.0},
                {'optimization/weight_time': 80.0},
                {'optimization/weight_formation': 15000.0},
                {'optimization/obstacle_clearance': 0.5},
                {'optimization/swarm_clearance': 0.5},
                
            ],
            remappings=[
                ('planning/broadcast_traj_send', '/broadcast_traj_planner'),
                # ('planning/broadcast_traj_send', '/broadcast_traj_from_planner'),
                ('planning/broadcast_traj_recv', '/broadcast_traj_planner'),
                # ('planning/broadcast_traj_recv', '/broadcast_traj_to_planner'),
                ('grid_map/odom', 'odometry'),
                ('grid_map/cloud', 'pcl_render_node/cloud'),
            ],
            output='screen',
            respawn=True
        )
    ])