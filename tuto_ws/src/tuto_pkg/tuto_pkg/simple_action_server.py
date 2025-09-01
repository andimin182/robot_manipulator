import rclpy
from rclpy.node import Node
import time
from rclpy.action import ActionServer
from robot_msgs.action import Fibonacci



class SimpleActionServer(Node):
    def __init__(self):
        super().__init__("SimpleActionNode")
        self.action_server_ = ActionServer(
            self, Fibonacci, 'fibonacci',
            self.goal_callback)
        self.get_logger().info("Action server Fibonacci initialized")

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received request with order {goal_request.request.order}")
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_request.request.order):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info(f"Current feedback: {feedback_msg.partial_sequence[1]}")
            goal_request.publish_feedback(feedback_msg)
            time.sleep(2)

        goal_request.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        return result
    
def main():
    rclpy.init()
    simple_action_server = SimpleActionServer()
    rclpy.spin(simple_action_server)
    simple_action_server.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()