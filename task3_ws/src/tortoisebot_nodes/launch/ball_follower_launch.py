from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node





def generate_launch_description():

    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'tortoisebot_Task2.urdf']))

    world_file=LaunchConfiguration('world_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'worlds', 'empty_world.sdf']))

    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description').find('tortoisebot_description'),'config', 'rviz_task2.rviz']))
    
    gazebo_rviz_launch_file=IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'launch','gazebo_launch.py']),launch_arguments={'urdf_path':urdf_path,'world_path':world_file,'rviz_file':rviz_config}.items())
    
    laserFilterNode=Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("tortoisebot_filters").find('tortoisebot_filters'),
                    "config", "median_filter_config.yaml",
                ])],
        )
    
    ballFollowerNode=TimerAction(
        period=1.0,
        actions=[

            Node(
        package='tortoisebot_nodes',
        name='following_closest_object',
        executable='follow_the_closest_object',
        output='screen',
        
    )

        ]
    )
    
    return LaunchDescription([
        
        gazebo_rviz_launch_file,
        laserFilterNode,
        ballFollowerNode,
        

    ])