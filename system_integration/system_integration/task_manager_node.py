import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # Lag en serviceklient for /go_to_home
        self.cli = self.create_client(Trigger, '/go_to_home')

        # Vent til tjenesten er tilgjengelig
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Venter på /go_to_home tjeneste...')

        # Kall tjenesten én gang ved oppstart
        self.go_to_home()

    def go_to_home(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)

        def callback(future):
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info('Gikk til hjem-posisjon.')
                else:
                    self.get_logger().error(f"Feil: {future.result().message}")
            else:
                self.get_logger().error("Servicekallet feilet.")

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
