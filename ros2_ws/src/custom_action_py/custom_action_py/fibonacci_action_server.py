import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_action_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = []

        for i in range(1, goal_handle.request.order + 1):
            fibonacci_number = self.fibonacci(i)
            feedback_msg.partial_sequence.append(fibonacci_number)
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

    def fibonacci(self, n):
        if n < 2:
            return n
        else:
            return self.fibonacci(n-1) + self.fibonacci(n-2)

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)

if __name__ == '__main__':
    main()