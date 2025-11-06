import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import isinf,isnan,pi,exp



class BallFollowNode(Node):

    def __init__(self):
        super().__init__('ball_follow_node') # node name
        self.subscription=self.create_subscription(
            LaserScan,
            'scan_filtered',
            self.listener_callback,
            10) # subscribing to /scan_filtered
        self.publisher = self.create_publisher(Twist,'cmd_vel',10) #publishing to /cmd_vel

        self.msgData=None
        self.safeDistanceObject=1.05
        
        
        self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] STARTED')
        self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] SUBSCRIBED TO /scan_filtered')
        self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Publishing to /cmd_vel')

    def findClosestObjectDirection(self,listVar,angMin,angMax,angInc):
        closestValue=min(listVar)
        closestIndex=listVar.index(closestValue)
        targetDir=angMin+closestIndex*angInc
        
        # if closestValue==float('inf'):
            # closestValue=self.safeDistanceObject
            
        return [targetDir,closestValue]
    
    
    def moveRobot(self,angularTarget=0.0,LinearTarget=0.0,stopRobot=False):
        (angularVel,linearVel)=(0.0,0.0)
        if stopRobot==True:
            return (angularVel,linearVel)
            
        distDiff=LinearTarget-self.safeDistanceObject
        
        gainLinearVel=2
        gainAngularVel=5

        if angularTarget!=0.0:
            angularVel=(gainAngularVel*angularTarget/pi)
        if distDiff!=0.0 :
            linearVel=gainLinearVel*((2/(1+exp(-4*distDiff)))-1)*(1-abs(angularTarget/pi))
            # linearVel=gainLinearVel*(distDiff)*(1-abs(angularTarget/pi))            
            
        dirValue='Turning Left' if angularVel>0.0 else 'Turning Right'
        speedValue='Going forward' if linearVel>0.0 else 'Going Reverse'
        
        # DEADBAND VELOCITY
        if -0.05<linearVel<0.05:
            linearVel=0.0
        if -0.1<angularVel<0.1:
            angularVel=0.0


        # VELOCITY LIMITER
        linearVel=min(2.0,linearVel)
        linearVel=max(-2.5,linearVel)
        angularVel=min(2.0,angularVel)
        angularVel=max(-2.0,angularVel)
        if linearVel in [2.0,-2.5] or angularVel in [2.0,-2.0]:
            if linearVel in [2.0,-3]:
                limiterName='LINEAR'
            if angularVel in [2.0,-2.0]:
                limiterName='LINEAR'
            self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] LIMITER TRIGGERED : %s'%limiterName)
        self.get_logger().debug('[DEBUG][BALL_FOLLOW_NODE] %s %s' %(dirValue,speedValue))
        
        return (angularVel,linearVel)
    
    
    def stopRobot(self):
        outputCmdVel=Twist()
        outputCmdVel.angular.z=0.0
        outputCmdVel.linear.x=0.0
        self.publisher.publish(outputCmdVel)
            
        
        
    def listener_callback(self,msg):
        # /scan INCOMING DATA
        self.msgData=msg
        distanceList=list(msg.ranges)
        minSensorAngle=msg.angle_min
        maxSensorAngle=msg.angle_max
        angleIncrement=msg.angle_increment
        infCount=distanceList.count(float('inf'))+distanceList.count(float('nan'))

        self.targetAngle=0.0
        self.resultVariable=0.0
        
        self.noObjectFlag=False

        if infCount == len(distanceList):
            self.noObjectFlag=True
            self.stopRobot()
        else:
            objDistDir=self.findClosestObjectDirection(distanceList,minSensorAngle,maxSensorAngle,angleIncrement)
            self.targetAngle=objDistDir[0]
            self.resultVariable=objDistDir[1]
            self.cmdvel_callback()
    
    def cmdvel_callback(self):

        if self.msgData is None:
            self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] NO SCAN DATA RECIEVED')
            return
        outputCmdVel=Twist()
        if self.noObjectFlag==False:
            resultMove=self.moveRobot(self.targetAngle,self.resultVariable)
            self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] NO SCAN DATA RECIEVED %s %s'%(resultMove))
            outputCmdVel.angular.z=resultMove[0]
            outputCmdVel.linear.x=resultMove[1]
            self.publisher.publish(outputCmdVel)
        else:
            outputCmdVel.angular.z=0.0
            outputCmdVel.linear.x=0.0
            self.publisher.publish(outputCmdVel)
            

def main(args=None):
    import subprocess
    cmd = [
    "ros2", "topic", "pub", "--once", "/cmd_vel", "geometry_msgs/msg/Twist",
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    ]
    

    rclpy.init(args=args)
    lidarsubb=BallFollowNode()
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
