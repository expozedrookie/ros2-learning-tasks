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
        
        
        self.msgData = None
        self.goal_active = False  # prevent multiple overlapping goals
        
        
    def send_goal(self, theta):
        if theta is None:
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
