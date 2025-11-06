#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.import time

import time

from tortoisebot_interfaces.action import FollowTheBall
from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class FollowActionServer(Node):

    def __init__(self):
        super().__init__('follow_action_server')
        self._action_server = ActionServer(
            self,
            FollowTheBall,
            'followtheball',
            self.execute_callback)
        self.publisher = self.create_publisher(Twist,'cmd_vel',10) #publishing to /closest_object_distance
        self.msgData=None

    def execute_callback(self, goal_handle):
        
        self.get_logger().info('Executing goal...')

        feedback_msg = FollowTheBall.Feedback()
        # feedback_msg.partial_sequence = [0, 1]
        # feedback_msg.remaining

        # for i in range(1, goal_handle.request.order):
        #     feedback_msg.partial_sequence.append(
        #         feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
        #     self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
        #     goal_handle.publish_feedback(feedback_msg)
        #     time.sleep(1)

        goal_handle.succeed()

        result = FollowTheBall.Result()
        # result.sequence = feedback_msg.partial_sequence
        return result
    

    def moveRobot(self,angularTarget=0.0,LinearTarget=0.0):
        (angularVel,linearVel)=(0.0,0.0)
        distDiff=LinearTarget-self.safeDistanceObject
        if angularTarget!=0.0:
             angularVel=0.5*(angularTarget)
        if distDiff>0.025 and -0.25<angularTarget<0.25:
            linearVel=1*distDiff
        if distDiff<0.0:# and -0.25<angularTarget<0.25:
            linearVel=0.25*distDiff
        if distDiff==0.0:
            linearVel=0.0

            
        # VELOCITY LIMITER
        linearVel=min(0.5,linearVel)
        linearVel=max(-1,linearVel)
        angularVel=min(0.5,angularVel)

        return (angularVel,linearVel)
    
    def cmdvel_callback(self):

        if self.msgData is None:
            self.get_logger().warn('[DEBUG][BALL_FOLLOW_NODE] NO SCAN DATA RECIEVED')
            return
        outputCmdVel=Twist()
        resultMove=self.moveRobot(0.0,self.safeDistanceObject)
        if self.noObjectFlag==False:
            resultMove=self.moveRobot(self.targetAngle,self.resultVariable)
        
        outputCmdVel.angular.z=resultMove[0]
        outputCmdVel.linear.x=resultMove[1]
        
        
        self.publisher.publish(outputCmdVel)
        # self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Dir : %f' %self.targetAngle)
        # self.get_logger().info('[DEBUG][BALL_FOLLOW_NODE] Dist: %f' %self.resultVariable)

def main(args=None):
    rclpy.init(args=args)

    follow_action_server = FollowActionServer()

    try:
        rclpy.spin(follow_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()