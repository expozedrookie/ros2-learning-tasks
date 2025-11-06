from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time=LaunchConfiguration('use_sim_time', default='true')
    
    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'tortoisebot_Task2.urdf']))
    urdf_content=FileContent(urdf_path)
    
    world_path=LaunchConfiguration('world_path', default=PathJoinSubstitution([
        FindPackageShare('tortoisebot_gazebo'), 'worlds', 'empty_world.sdf'
    ]))
    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description').find('tortoisebot_description'),'config', 'rviz.rviz']))
    
    bridge_params=os.path.join(FindPackageShare('tortoisebot_gazebo').find('tortoisebot_gazebo'),'config','bridge_parameters.yaml')
    

    

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

    create_entity =Node(
        package='ros_gz_sim',
        executable='create',
        name='spawnentity',
        arguments=[
            '-name', 'tortoisebot',
            #'-t','robot_description',
            '-file', urdf_path,
            '-z', '2'
            ],
            output='screen'
            )
    
    gazebo_ros_bridge=Node(
        package='ros_gz_bridge',
		executable='parameter_bridge',
		arguments=[
            '--ros-args','-p', f'config_file:={bridge_params}'
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
        create_entity,
        rviz_launch_file,
        gazebo_ros_bridge,

    ])