from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch.conditions import UnlessCondition,IfCondition




def generate_launch_description():
    use_laser_filters=LaunchConfiguration('use_laser_filters', default='false')

    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'tortoisebot_Task2.urdf']))

    world_file=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'worlds', 'square_world.sdf'])

    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description').find('tortoisebot_description'),'config', 'rviz_task2.rviz']))
    
    gazebo_rviz_launch_file=IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'launch','gazebo_launch.py']),launch_arguments={'urdf_path':urdf_path,'world_path':world_file,'rviz_file':rviz_config}.items())
    
    laserFilterNode=Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("tortoisebot_filters").find('tortoisebot_filters'),
                    "config", "filter_config.yaml",
                ])],
		condition=IfCondition(use_laser_filters),
        )
    closestDistanceFinderNode=TimerAction(
        period=1.0,
        actions=[

            Node(
        package='tortoisebot_nodes',
        name='lidar_closest_distance',
        executable='find_closest_object_distance',
        output='screen',
		condition=UnlessCondition(use_laser_filters),
    )

        ]
    )
    laserFilterClosestDistanceFinderNode=TimerAction(
        period=1.0,
        actions=[

            Node(
        package='tortoisebot_nodes',
        name='lidar_closest_distance',
        executable='find_distance_using_laser_filters',
        output='screen',
		condition=IfCondition(use_laser_filters),
    )

        ]
    )
    
    return LaunchDescription([
        
        gazebo_rviz_launch_file,
        closestDistanceFinderNode,
        laserFilterNode,
        laserFilterClosestDistanceFinderNode,

    ])