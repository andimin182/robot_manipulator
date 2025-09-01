import rclpy
from rclpy.node import Node
from robot_msgs.srv import AddTwoInt
import sys

class SimpleServerClient(Node):
    def __init__(self, a, b):
        super().__init__('simple_server_client')

        self.client_ = self.create_client(AddTwoInt, 'add_two_int')
        self.get_logger().info('Client created')

        while not self.client_.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interruped while waiting for the server.')
                return
            else:
                self.get_logger().info('Server not available, waiting again...')

        self.req_ = AddTwoInt.Request()
        self.req_.a = a
        self.req_.b = b
        self.future_ = self.client_.call_async(self.req_)
        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info('Service response: %d' % future.result().sum)

def main():
    rclpy.init()
    if len(sys.argv) !=3:
        print('2 arguments must be provided')
        return -1
    else:
        simple_client = SimpleServerClient(int(sys.argv[1]), int(sys.argv[2]))
        rclpy.spin(simple_client)
        simple_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()