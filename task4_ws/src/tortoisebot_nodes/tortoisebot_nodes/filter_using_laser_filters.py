import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from math import isinf,isnan


class LidarFilterNode(Node):

    def __init__(self):
        super().__init__('laser_filter_close_object_node') # node name
        self.subscription=self.create_subscription(
            LaserScan,
            'scan_filtered',
            self.listener_callback,
            10) # subscribing to /scan
        self.publisher = self.create_publisher(Float32,'closest_object_distance',10) #publishing to /closest_object_distance

        self.msgData=None
        self.resultVariable=[]
        self.get_logger().info('[DEBUG][LASER_FILTER_NODE] STARTED ')
        self.get_logger().info('[DEBUG][LASER_FILTER_NODE] SUBSCRIBED TO /scan_filtered')
        self.get_logger().info('[DEBUG][LASER_FILTER_NODE] Publishing to /closest_object_distance')
    
    @staticmethod
    def minimumValidFilter(listVariable,minRange,maxRange):
        listLength=len(listVariable)
        
        minValFilteredList=[]

        for i in range(listLength):
            if minRange<=listVariable[i]<=maxRange and not isinf(listVariable[i]) and not isnan(listVariable[i]):
                minValFilteredList.append(listVariable[i])
        return minValFilteredList
    
    
    
    def listener_callback(self,msg):
        # /scan INCOMING DATA
        self.msgData=msg
        distanceList=list(msg.ranges)
        

        # FILTERING OUT INVALID VALUES - inf,NaN,Zeros
        lidarMinRange=msg.range_min
        lidarMaxRange=msg.range_max
        minValFilteredList=self.minimumValidFilter(distanceList,lidarMinRange,lidarMaxRange)
        #print(minValFilteredList)
        

        try:
            self.resultVariable=min(minValFilteredList)
        except ValueError:
            self.get_logger().warn('[DEBUG][LASER_FILTER_NODE] NO OBJECTS DETECTED')
            self.resultVariable=float('inf')
            

        self.publisher_callback()
    
    def publisher_callback(self):
        if self.msgData is None:
            self.get_logger().warn('[DEBUG][LASER_FILTER_NODE] NO SCAN DATA RECIEVED from /scan_filtered')
            return
        
        rangeFilterOutput=Float32()
        rangeFilterOutput.data=self.resultVariable
        self.publisher.publish(rangeFilterOutput)
        self.get_logger().info('[DEBUG][LASER_FILTER_NODE] Publishing Closest Distance %f' %self.resultVariable)



def main(args=None):
    rclpy.init(args=args)
    lidarsubb=LidarFilterNode()
    rclpy.spin(lidarsubb)

    lidarsubb.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
