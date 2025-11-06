import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random
import time


class MovingSphere(Node):

    def __init__(self):
        super().__init__('ball_follow_node') # node name
        
        self.publisher = self.create_publisher(Twist,'/model/moving_sphere/cmd_vel',10) #publishing to /closest_object_distance
        
        timer_period=0.5 #seconds
        self.timer=self.create_timer(timer_period,self.cmdvel_callback) # if publishing at specific interval
    
    def cmdvel_callback(self):
        
        
        
        outputCmdVel=Twist()        
        outputCmdVel.linear.x=random.choice([random.random(),-random.random()])
        outputCmdVel.linear.y=random.choice([random.random(),-random.random()])
        # outputCmdVel.angular.z=random.choice([random.random(),-random.random()])
        self.publisher.publish(outputCmdVel)
        # time.sleep(1)
        # outputCmdVel.linear.x=0.0
        # # outputCmdVel.linear.y=0.0
        # outputCmdVel.angular.z=random.choice([random.random(),-random.random()])
        self.publisher.publish(outputCmdVel)
        



def main(args=None):
    import subprocess
    cmd = [
    "ros2", "topic", "pub", "--once", "/model/moving_sphere/cmd_vel", "geometry_msgs/msg/Twist",
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ]
    

    rclpy.init(args=args)
    lidarsubb=MovingSphere()
    try:
        rclpy.spin(lidarsubb)
    except KeyboardInterrupt:
        try:
            lidarsubb.destroy_node()
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            print('\n NODE ALREADY SHUTDOWN')
    finally:
        print('STOPPING ROBOT')
        subprocess.run(cmd)
        

if __name__=='__main__':
    main()

