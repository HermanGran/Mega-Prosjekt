import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # Publishes movement commands to the motion planner
        self.command_pub = self.create_publisher(String, '/move_command', 10)

        # Start a background thread to read from terminal
        threading.Thread(target=self.command_input_loop, daemon=True).start()

    def command_input_loop(self):
        while rclpy.ok():
            cmd = input('home').strip()
            if cmd:
                msg = String()
                msg.data = cmd
                self.command_pub.publish(msg)
                self.get_logger().info(f"Sent command: '{cmd}'")

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
