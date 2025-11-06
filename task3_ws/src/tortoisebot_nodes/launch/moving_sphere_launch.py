from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time=LaunchConfiguration('use_sim_time', default='true')
    
    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'tortoisebot_Task2.urdf']))
    urdf_content=FileContent(urdf_path)

    # sphere_path=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'moving_sphere.sdf'])
    
    
    world_location=LaunchConfiguration('world_path',default=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'worlds', 'slow_moving_sphere.sdf']))
    # world_location=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'moving_square.sdf'])
    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description').find('tortoisebot_description'),'config', 'rviz.rviz']))
    
    filter_params=os.path.join(FindPackageShare('tortoisebot_filters').find('tortoisebot_filters'),'config','median_filter_config.yaml')
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
            'gz_args':['-r ',world_location],
            'on_exit_shutdown':'true'
            }.items()
    )
    create_bot_entity =Node(
        package='ros_gz_sim',
        executable='create',
        name='spawnentity',
        arguments=[
            '-name', 'tortoisebot',
            #'-t','robot_description',
            '-file', urdf_path,
            '-x','3',
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
    start_follow_script=Node(
        package='tortoisebot_nodes',
		executable='follow_the_closest_object',
		arguments=[],
	output='screen'
  )
    
    
    start_median_filter =Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[filter_params]
        )
    filter_params
    
    gazebo_ros_bridge=Node(
        package='ros_gz_bridge',
		executable='parameter_bridge',
		arguments=[
            '--ros-args','-p', f'config_file:={moving_sphere_bridge_parms}'
    ],
	output='screen'
  )
    

    rviz_launch_file=IncludeLaunchDescription(
        PathJoinSubstitution([
			FindPackageShare('tortoisebot_description'),
			'launch',
			'rviz_launch.py',
			]),launch_arguments={'use_sim_time':use_sim_time,'urdf_path':urdf_content,'config_file':rviz_config}.items())

    return LaunchDescription([
        
        start_gazebo_world,
        gazebo_ros_bridge,
        create_bot_entity,
        rviz_launch_file,
        start_median_filter,
        start_follow_script,

    ])