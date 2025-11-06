import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import isinf,isnan,pi,degrees,exp


class CloseObjectFinder(Node):

    def __init__(self):
        super().__init__('closest_object_finder_node') # node name
        self.subscription=self.create_subscription(
            LaserScan,
            'scan_filtered',
            self.listener_callback,
            10) # subscribing to /scan filtered
        self.publisher = self.create_publisher(String,'current_closest_object',10) #publishing to /current_closest_object

        self.msgData=None        
        
        self.get_logger().warn('[DEBUG][CLOSEST_OBJECT_FINDER] STARTED')
        self.get_logger().info('[DEBUG][CLOSEST_OBJECT_FINDER] SUBSCRIBED TO /scan_filtered')
        self.get_logger().info('[DEBUG][CLOSEST_OBJECT_FINDER] Publishing to /current_closest_object')
        

        
    



    def findClosestObjectDirection(self,listVar,angMin,angMax,angInc):
        closestValue=min(listVar)
        closestIndex=listVar.index(closestValue)
        targetDir=angMin+closestIndex*angInc
        
        if closestValue==float('inf'):
            closestValue=self.safeDistanceObject
            
        return [targetDir,closestValue]
    
            
        
        
    def listener_callback(self,msg):
        # /scan INCOMING DATA
        self.msgData=msg
        distanceList=list(msg.ranges)
        minSensorAngle=msg.angle_min
        maxSensorAngle=msg.angle_max
        angleIncrement=msg.angle_increment
        infCount=distanceList.count(float('inf'))+distanceList.count(float('nan'))

        self.targetAngle=0.0
        self.targetDistance=0.0
        self.noObjectFlag=False

        if infCount == len(distanceList):
            self.noObjectFlag=True
        else:
            objDistDir=self.findClosestObjectDirection(distanceList,minSensorAngle,maxSensorAngle,angleIncrement)
            self.targetAngle=objDistDir[0]
            self.targetDistance=objDistDir[1]
        self.cmdvel_callback()
    
    def cmdvel_callback(self):

        if self.msgData is None:
            self.get_logger().warn('[DEBUG][CLOSEST_OBJECT_FINDER] NO SCAN DATA RECIEVED')
            return
        outputCmdVel=String()
        
        finalMessage=f'{self.targetAngle},{self.targetDistance}'
        outputCmdVel.data=finalMessage
        
        
        
        self.publisher.publish(outputCmdVel)
        


def main(args=None):
    rclpy.init(args=args)
    class_subb=CloseObjectFinder()
    rclpy.spin(class_subb)

    class_subb.destroy_node()
    rclpy.shutdown()
    

if __name__=='__main__':
    main()

