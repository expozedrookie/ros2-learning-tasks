# tortoisebot-sim-tasks
The goal of this project is to build a solid understanding of core ROS 2 principles (nodes, topics, launch files, custom packages) and their application in a realistic, simulated environment.

**Common Setup**

- Go to tbot_ws directory -``cd tbot_ws``
- Build the workspace             - ``colcon build``
- Source the workspace            - ``source install/setup.bash``
- ``ros2 launch tortoisebot_bringup task.launch.py task_id:=<task_number>``

**Task 1 - Simulate tortoisebot.urdf in Gazebo and run it using Teleop**
- RViz only launch   - ``ros2 launch tortoisebot_description rviz_launch.py ``
- Gazebo+Rviz launch - ``ros2 launch tortoisebot_gazebo gazebo_launch.py``

**Task 2 - Get nearest object distance using LiDAR and laser filters**
- Closest Distance using custom filter - ``ros2 launch tortoisebot_filters find_closest_distance_launch.py ``
- Closest Distance using laser_filters - ``ros2 launch tortoisebot_filters find_closest_distance_launch.py  use_laser_filters:=true ``

**Task 3 - Follow a Ball Shaped Object**
- Get to the closest object - ``ros2 launch tortoisebot_nodes ball_follower_launch.py `` (Initial Version)
- Follow the ball - ``ros2 launch tortoisebot_nodes moving_sphere_launch.py `` 