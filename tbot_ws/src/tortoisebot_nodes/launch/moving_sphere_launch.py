from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    # argument for using sim time for gazebo
    use_sim_time=LaunchConfiguration('use_sim_time', default='true')
    # argument for using other URDF
    urdf_file=LaunchConfiguration('urdf_file', default='tortoisebot_Task2.urdf')
    # argument for using other URDF path
    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', urdf_file]))
    # world_file = dynamic world name to load which is already in install folder
    world_file=LaunchConfiguration('world_file',default='slow_moving_sphere.sdf')
    # world_path = dynamic path to the world file as per task
    world_path=LaunchConfiguration('world_path',default=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'worlds', world_file]))
    # world_location=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'moving_square.sdf'])
    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description').find('tortoisebot_description'),'config', 'rviz.rviz']))
    # filter_parameter file path for filtering using laser_filters
    filter_params=os.path.join(FindPackageShare('tortoisebot_filters').find('tortoisebot_filters'),'config','median_filter_config.yaml')
    # bridge parameter file path for filtering using laser_filters
    moving_sphere_bridge_parms=os.path.join(FindPackageShare('tortoisebot_gazebo').find('tortoisebot_gazebo'),'config','moving_env_bridge.yaml')

    # Start Gazebo using ros_gz_sim launch
    start_gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args':['-r ',world_path],
            'on_exit_shutdown':'true'
            }.items()
            )
    # Spawn bot urdf
    create_bot_entity =Node(
        package='ros_gz_sim',
        executable='create',
        name='spawnentity',
        arguments=[
            '-name', 'tortoisebot',
            #'-t','robot_description',
            '-file', urdf_path,
            '-x','3', # spawn coordinates
            '-y','4',
            '-z', '2'
            ],
            output='screen'
            )

#     start_follow_script =TimerAction(
#         period=1.0,
#         actions=[Node(
#         package='tortoisebot_nodes',
# 		executable='follow_the_closest_object',
# 		arguments=[],
# 	output='screen'
#   )])
    # Start node to follow the closest object
    start_follow_script=Node(
        package='tortoisebot_nodes',
		executable='follow_the_closest_object',
		arguments=[],
	output='screen'
  )
    
    # start laser_filters node to filter scans
    start_median_filter =Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[filter_params]
        )
    # gazebo node for briding topics
    gazebo_ros_bridge=Node(
        package='ros_gz_bridge',
		executable='parameter_bridge',
		arguments=[
            '--ros-args','-p', f'config_file:={moving_sphere_bridge_parms}'
    ],
	output='screen'
    )
    
    # start rviz launch file from
    rviz_launch_file=IncludeLaunchDescription(
        PathJoinSubstitution([
			FindPackageShare('tortoisebot_description'),
			'launch',
			'rviz_launch.py',
			]),launch_arguments={'use_sim_time':use_sim_time,'urdf_path':urdf_path,'config_file':rviz_config}.items())

    return LaunchDescription([
        
        start_gazebo_world,
        gazebo_ros_bridge,
        create_bot_entity,
        rviz_launch_file,
        start_median_filter,
        start_follow_script,

    ])