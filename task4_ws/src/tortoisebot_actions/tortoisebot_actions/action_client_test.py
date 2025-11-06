from tortoisebot_interfaces.action import FollowTheBall
import rclpy
from std_msgs.msg import String
from rclpy.action import ActionClient
from rclpy.node import Node


class FollowActionClient(Node):

    def __init__(self):
        super().__init__('follow_ball_action_client')
        # Match this name to your server's action name
        self._action_client = ActionClient(self, FollowTheBall, 'follow_the_ball')
        
        # Subscribes to string messages like "angle,distance"
        self.subscription = self.create_subscription(
            String,
            'current_closest_object',
            self.listener_callback,
            10)
        
        self.msgData = None
        self.goal_active = False  # prevent multiple overlapping goals

    def listener_callback(self, msg):
        # Parse the incoming string safely
        try:
            targetData = msg.data.strip().split(',')
            self.targetAngle = float(targetData[0])
            self.targetDistance = float(targetData[1])
        except (ValueError, IndexError):
            self.get_logger().warn("Invalid /current_closest_object format. Expected 'angle,distance'.")
            return
        
        self.msgData = msg
        self.get_logger().info(f"Target angle: {self.targetAngle}, distance: {self.targetDistance}")

        # Only send a new goal if no active goal is running
        if not self.goal_active:
            self.send_goal(self.targetAngle)
        else:
            self.get_logger().info("Goal already active — ignoring new one until finished.")

    def send_goal(self, theta):
        if self.msgData is None:
            self.get_logger().warn("No target data yet.")
            return

        goal_msg = FollowTheBall.Goal()
        goal_msg.theta = theta

        self._action_client.wait_for_server()
        self.get_logger().info("Connected to action server.")

        self.goal_active = True
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.goal_active = False
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # Update this field according to your FollowTheBall.action definition
        self.get_logger().info(f'Result received: {result}')
        self.goal_active = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Update this to your action's actual feedback field
        self.get_logger().info(f'Feedback: {feedback}')


def main(args=None):
    rclpy.init(args=args)
    node = FollowActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
