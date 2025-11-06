import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import isinf,isnan,pi,degrees,exp



class BallFollowNode(Node):

    def __init__(self):
        super().__init__('ball_follow_node') # node name
        self.subscription=self.create_subscription(
            LaserScan,
            'scan_filtered',
            self.listener_callback,
            10) # subscribing to /scan
        self.publisher = self.create_publisher(Twist,'cmd_vel',10) #publishing to /closest_object_distance

        self.msgData=None
        self.safeDistanceObject=1.05
        self.timerManual=0
        self.has_scan=False
        
        self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] STARTED')
        self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] SUBSCRIBED TO /scan_filtered')
        self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Publishing to /cmd_vel')
        

        
    



    def findClosestObjectDirection(self,listVar,angMin,angMax,angInc):
        closestValue=min(listVar)
        closestIndex=listVar.index(closestValue)
        targetDir=angMin+closestIndex*angInc
        
        if closestValue==float('inf'):
            closestValue=self.safeDistanceObject
            
        return [targetDir,closestValue]
    
    
    def moveRobot(self,angularTarget=0.0,LinearTarget=0.0,stopRobot=False):
        (angularVel,linearVel)=(0.0,0.0)
        if stopRobot==True:
            return (angularVel,linearVel)
        distDiff=LinearTarget-self.safeDistanceObject
        

        if angularTarget!=0.0:
            angularVel=(2.0*angularTarget/pi)
            if distDiff!=0.0:
                linearVel=2*((2/(1+exp(-5*distDiff)))-1)*(1-abs(angularVel))


        # if distDiff>0.05:
        #     linearVel=1.5*((2/(1+exp(-5*distDiff)))-1)
        # elif distDiff<0.0:
        #     linearVel=1.5*((2/(1+exp(-10*distDiff)))-1)
        # else:
        #     linearVel=0.5*distDiff
        
        # if distDiff>0.05:
        #     linearVel=1.0*distDiff*(1-(abs(angularVel)))
        # elif distDiff<0.0:# and -0.25<angularTarget<0.25:
        #     linearVel=distDiff#0.5*(distDiff-1.0)
        #     linearVel=-(distDiff-1.0)**2
        #     self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] VELOCITY %s' %(linearVel))
        # else:
        #     linearVel=-(distDiff-1.0)**2
        
        # linearVel=min(1.5,linearVel)
        # linearVel=max(-2.5,linearVel)
        # angularVel=2*angularVel/pi #since angular target is between -3.14 and 3.14
        
            
        dirValue='Turning Left' if angularVel>0.0 else 'Turning Right'
        speedValue='Going forward' if linearVel>0.0 else 'Going Reverse'
        # VELOCITY LIMITER
        linearVel=min(2.0,linearVel)
        linearVel=max(-2.0,linearVel)
        angularVel=min(2.0,angularVel)
        # self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] %s ' %dirValue)
        # self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] %s ' %speedValue)
        # if self.timerManual%4==0:
        #     self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] %s %s' %(angularVel,linearVel))
        # self.timerManual+=1
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
        self.has_scan=True
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

        if self.has_scan==False:
            self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] NO SCAN DATA RECIEVED')
            self.stopRobot()
            return
        outputCmdVel=Twist()
        resultMove=self.moveRobot(0.0,self.safeDistanceObject,stopRobot=True)
        if self.noObjectFlag==False:
            resultMove=self.moveRobot(self.targetAngle,self.resultVariable)
        print(resultMove)
        outputCmdVel.angular.z=resultMove[0]
        outputCmdVel.linear.x=resultMove[1]
        
        if self.timerManual%4==0:
            self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] %s %s' %(resultMove))
        self.timerManual+=1
        self.publisher.publish(outputCmdVel)
        # self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Dir : %f' %self.targetAngle)
        # self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Dist: %f' %self.resultVariable)



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
