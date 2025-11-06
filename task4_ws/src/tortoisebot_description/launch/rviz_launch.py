from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,FileContent,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition
import os

def generate_launch_description():
	use_sim_time=LaunchConfiguration('use_sim_time',default='false')

	urdf_content=LaunchConfiguration('urdf_path', default=FileContent(PathJoinSubstitution([FindPackageShare('tortoisebot_description'),'models', 'tortoisebot.urdf'])))

	rviz_config_location=LaunchConfiguration('config_file', default=PathJoinSubstitution([
        FindPackageShare('tortoisebot_description'),'config','rviz.rviz']))
	
	
	# rviz_config_location=PathJoinSubstitution([
    #     FindPackageShare('tortoisebot_description'),'config','rviz.rviz'])
	

	start_rviz=Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		arguments=['-d',rviz_config_location],
		output='screen'

	)
	start_rsp=Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher',
		parameters=[{'use_sim_time':use_sim_time,'robot_description':urdf_content}],
		output='screen'
	)
	start_jsp=Node(
        package='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
		condition=UnlessCondition(use_sim_time),
    )
	
	
	return LaunchDescription([
			start_rsp,
			start_jsp,
			start_rviz,
			])