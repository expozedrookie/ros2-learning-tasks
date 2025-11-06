import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from math import isinf,isnan,pi


class LidarFilterNode(Node):

    def __init__(self):
        super().__init__('lidar_filtering_node') # node name
        self.subscription=self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10) # subscribing to /scan
        self.publisher = self.create_publisher(Float32,'closest_object_distance',10) #publishing to /closest_object_distance

        self.msgData=None
        self.resultVariable=[]
        self.get_logger().warn('[DEBUG][CUSTOM_FILTER_NODE] STARTED')
        self.get_logger().info('[DEBUG][CUSTOM_FILTER_NODE] SUBSCRIBED TO /scan')
        self.get_logger().info('[DEBUG][CUSTOM_FILTER_NODE] Publishing to /closest_object_distance')


        #timer_period=0.125 #seconds
        #self.timer=self.create_timer(timer_period,self.publisher_callback) # if publishing at specific interval
        

    @staticmethod
    def spatialMedianFilter(listVariable,windowSize=5):
        n=len(listVariable)
        half=windowSize//2
        localFilteredList=[]

        for i in range(n):
            start=max(0,i-half)
            end=min(n,i+half+1)

            window=listVariable[start:end]
            window.sort()
            m=len(window)
            if len(window)%2==1:
                median=window[m//2]
            else:
                median=(window[(m//2)-1]+window[m//2])/2
            localFilteredList.append(median)
            

        return localFilteredList

    
    @staticmethod
    def minimumValidFilter(listVariable,minRange,maxRange):
        listLength=len(listVariable)
        
        minValFilteredList=[]

        for i in range(listLength):
            if minRange<=listVariable[i]<=maxRange and not isinf(listVariable[i]) and not isnan(listVariable[i]):
                minValFilteredList.append(listVariable[i])
        return minValFilteredList
    
    @staticmethod
    def FrontRangeFilter(listVariable,aMin,aInc,minMaxAngle):
        listLength=len(listVariable)
        updatedList=[0]*listLength
        
        cropMin=minMaxAngle[0]
        cropMax=minMaxAngle[1]
        for i in range(len(listVariable)):
            currentAngle=int((aMin+(i*aInc))*180/pi)
            #print(i,' = ',currentAngle,' ',listVariable[i])
            if currentAngle in list(range(cropMin,cropMax+1)):
                updatedList[i]=listVariable[i]
            else:
                updatedList[i]=float('inf')
        
        return updatedList
    
    
    def listener_callback(self,msg):
        # /scan INCOMING DATA
        self.msgData=msg
        distanceList=list(msg.ranges)
        # sampleCount=len(distanceList)
        # requiredAngleDegrees=15 #degrees
        # requiredMaxSample=(int)((sampleCount/360)*requiredAngleDegrees)
        minSensorAngle=float(msg.angle_min)
        maxSensorAngle=float(msg.angle_max)
        minMaxRequiredAngle=(-15,15) # required range tuple
        angleIncrement=msg.angle_increment

        # EXTRACTING DATA ONLY IN -15 TO 15 DEGREES
        angleFilteredScan=self.FrontRangeFilter(distanceList,minSensorAngle,angleIncrement,minMaxRequiredAngle)
        # print(angleFilteredScan)
        # print(len(angleFilteredScan))
        
        # APPLYING MEDIAN FILTER TO MANAGE SPIKES
        medianFilteredScan=self.spatialMedianFilter(angleFilteredScan)
        


        # FILTERING OUT INVALID VALUES - inf,NaN,Zeros
        lidarMinRange=msg.range_min
        lidarMaxRange=msg.range_max
        minValFilteredList=self.minimumValidFilter(medianFilteredScan,lidarMinRange,lidarMaxRange)
        #print(minValFilteredList)
        try:
            self.resultVariable=min(minValFilteredList)
        except ValueError:
            self.get_logger().warn('[DEBUG][CUSTOM_FILTER_NODE] NO OBJECT DETECTED!')
            self.resultVariable=float('inf')

        # self.resultVariable=angleFilteredScan

        
        self.publisher_callback()
    
    def publisher_callback(self):
        if self.msgData is None:
            self.get_logger().warn('[DEBUG][CUSTOM_FILTER_NODE] NO SCAN DATA RECIEVED')
            return
        
        
        # rangeFilterOutput=LaserScan()
        # rangeFilterOutput.header=self.msgData.header
        # rangeFilterOutput.angle_min=self.msgData.angle_min
        # rangeFilterOutput.angle_max=self.msgData.angle_max
        # rangeFilterOutput.angle_increment=self.msgData.angle_increment
        # rangeFilterOutput.time_increment=self.msgData.time_increment
        # rangeFilterOutput.scan_time=self.msgData.scan_time
        # rangeFilterOutput.range_min=self.msgData.range_min
        # rangeFilterOutput.range_max=self.msgData.range_max
        # rangeFilterOutput.ranges=self.resultVariable
        # rangeFilterOutput.intensities=self.msgData.intensities
        rangeFilterOutput=Float32()
        rangeFilterOutput.data=self.resultVariable
        self.publisher.publish(rangeFilterOutput)
        self.get_logger().info('[DEBUG][CUSTOM_FILTER_NODE] Publishing Closest Distance %f' %self.resultVariable)



def main(args=None):
    rclpy.init(args=args)
    lidarsubb=LidarFilterNode()
    rclpy.spin(lidarsubb)

    lidarsubb.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()

