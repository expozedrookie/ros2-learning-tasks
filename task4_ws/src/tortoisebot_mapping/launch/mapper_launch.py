from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo,TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FileContent
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch.conditions import UnlessCondition,IfCondition




def generate_launch_description():
    use_laser_filters=LaunchConfiguration('use_laser_filters', default='false')

    urdf_path=LaunchConfiguration('urdf_path', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'tortoisebot_Task4.urdf']))
    
    slam_params_file=LaunchConfiguration('slam_param_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_mapping').find('tortoisebot_mapping'),'config', 'mapping_config.yaml']))
    
    bridge_file_path=''#PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo').find('tortoisebot_gazebo'),'config', 'mapping_bridge.yaml'])

    world_file=LaunchConfiguration('world_file', default='custom_world.world.sdf')
    
    world_path=PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'worlds', world_file])

    rviz_config=LaunchConfiguration('rviz_file', default=PathJoinSubstitution([FindPackageShare('tortoisebot_description').find('tortoisebot_description'),'config', 'rviz_task4.rviz']))
    
    gazebo_rviz_launch_file=IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('tortoisebot_gazebo'),'launch','gazebo_launch.py']),launch_arguments={'urdf_path':urdf_path,'world_path':world_path,'rviz_file':rviz_config}.items())

    slam_launch = IncludeLaunchDescription( PathJoinSubstitution([FindPackageShare('slam_toolbox'),'launch','online_async_launch.py']),launch_arguments={'slam_params_file':slam_params_file,'use_sim_time':'true'}.items())
    
    
    return LaunchDescription([
        
        gazebo_rviz_launch_file,
        #slam_launch,
        

    ])