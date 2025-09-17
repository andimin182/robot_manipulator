import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")
        self.pub_ = self.create_publisher(String, "chatter", 10)
        self.counter_ = 0
        self.freq_ = 1.0
        self.get_logger().info("Publishing at %d Hz" % self.freq_)
        self.timer_ = self.create_timer(self.freq_, self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data = "Hello: counter %d" % self.counter_
        self.pub_.publish(msg)
        self.counter_ += 1

def main():
    rclpy.init()
    pub = SimplePublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

