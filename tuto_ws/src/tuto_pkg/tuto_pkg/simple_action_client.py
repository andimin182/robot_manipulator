import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robot_msgs.action import Fibonacci

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__("SimpleActionClientNode")

        self.action_client_ = ActionClient(self, Fibonacci, "fibonacci")

        self.action_client_.wait_for_server()

        self.goal = Fibonacci.Goal()
        self.goal.order = 10
        self.future_ = self.action_client_.send_goal_async(self.goal, feedback_callback=self.feedbackCallback)
        self.future_.add_done_callback(self.futureCallback)


    def feedbackCallback(self, feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback.partial_sequence}")

    def futureCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        else:
            self.get_logger().info("Goal accepted")
            self.future_ = goal_handle.get_result_async()
            self.future_.add_done_callback(self.resultCallback)

    def resultCallback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.sequence}")
        rclpy.shutdown()


def main():
    rclpy.init()
    action_client = SimpleActionClient()
    rclpy.spin(action_client)


if __name__=="__main__":
    main()
