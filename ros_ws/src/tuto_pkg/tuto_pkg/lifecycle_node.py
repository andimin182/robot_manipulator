import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String
import time

class SimpleLifecycleNode(Node):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)
        self.get_logger().info("Lifecycle node on configure called")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.sub_)
        self.get_logger().info("Lifecycle node shutdown called")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self.sub_)
        self.get_logger().info("Lifecycle node cleanup called")
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle node activate called")
        time.sleep(5)
        return super().on_activate(state)
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Lifecycle node deactivate called")
        return super().on_deactivate(state)
    
    def msgCallback(self, msg):
        current_state = self._state_machine.current_state
        if current_state[1] == "active":
            self.get_logger().info(f"The received msg is: {msg.data}")
        else:
            pass

def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    node = SimpleLifecycleNode("simple_lifecycle_node")
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executor.ExternalShutdownExeption):
        node.destroy()




if __name__=='__main__':
    main()



