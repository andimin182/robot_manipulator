import rclpy
from rclpy.node import Node
from robot_msgs.srv import AddTwoInt

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('add_two_int_server')
        self.service_ = self.create_service(AddTwoInt, 'add_two_int', self.serviceCallback)

        self.get_logger().info('Service add_two_int ready')

    def serviceCallback(self, req, res):
        self.get_logger().info('New request received with a: %d, b: %d' % (req.a, req.b))
        res.sum = req.a + req.b
        self.get_logger().info('Returning sum: %d' % (res.sum))
        return res
    

def main():
    rclpy.init()
    service_node = SimpleServiceServer()
    rclpy.spin(service_node)
    service_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()



