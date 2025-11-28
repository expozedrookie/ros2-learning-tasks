from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch.conditions import UnlessCondition,IfCondition




def generate_launch_description():
    # argument to choose to use ROS laser filters or Custom Filters
    use_laser_filters=LaunchConfiguration('use_laser_filters', default='false')
    # argument for name to the urdf file
    urdf_file=LaunchConfiguration('urdf_file', default='tortoisebot_Task2.urdf')
    # argument for path to the urdf file
    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', urdf_file]))
    # argument for name to the world file
    world_file=LaunchConfiguration('world_file', default='square_world.sdf') 
    # argument for path to the world file
    world_path=LaunchConfiguration('world_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'worlds', world_file]))
    # rviz_file = argument for name to the rviz configuration file
    rviz_file=LaunchConfiguration('rviz_file', default='rviz_task2.rviz')
    # rviz_config = argument for path to the rviz configuration file
    rviz_config=LaunchConfiguration('rviz_file_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description').find('tortoisebot_description'),'config', rviz_file]))
    
    # Start Gazebo using ros_gz_sim launch
    gazebo_rviz_launch_file=IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'launch','gazebo_launch.py']),launch_arguments={'urdf_path':urdf_path,'world_path':world_path,'rviz_file':rviz_config}.items())
    
    # Start scan_to_scan_filter_chain from laser_filters package
    laserFilterNode=Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("tortoisebot_filters").find('tortoisebot_filters'),
                    "config", "filter_config.yaml",
                ])],
		condition=IfCondition(use_laser_filters), # Run only if use_laser_filters = true
        )
    
    # Start custom filter node from current package and find closest distance
    closestDistanceFinderNode=TimerAction(
        period=1.0,
        actions=[

            Node(
        package='tortoisebot_nodes', # changed the node package to new structure 
        name='lidar_closest_distance',
        executable='find_closest_object_distance',
        output='screen',
		condition=UnlessCondition(use_laser_filters), # Run only if use_laser_filters = false
    )

        ]
    )
    # Start  node which uses scan after filtering using laser_filters from current package and find closest distance
    laserFilterClosestDistanceFinderNode=TimerAction(
        period=1.0,
        actions=[

            Node(
        package='tortoisebot_nodes', # changed the node package to new structure 
        name='lidar_closest_distance',
        executable='find_distance_using_laser_filters',
        output='screen',
		condition=IfCondition(use_laser_filters),# Run only if use_laser_filters = true
    )

        ]
    )
    
    return LaunchDescription([
        
        gazebo_rviz_launch_file,
        closestDistanceFinderNode,
        laserFilterNode,
        laserFilterClosestDistanceFinderNode,

    ])