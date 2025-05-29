import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # Opprett klient for /go_to_home
        self.cli = self.create_client(Trigger, '/go_to_home')

        # Vent til tjenesten er tilgjengelig
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Venter på /go_to_home-tjeneste...')

        self.get_logger().info('Tjeneste funnet. Kaller /go_to_home...')
        self.call_go_to_home()

    def call_go_to_home(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)

        def callback(future):
            if future.result() is not None and future.result().success:
                self.get_logger().info('Robot gikk til home-posisjon.')
            else:
                self.get_logger().error('Feil: Kunne ikke sende robot til home.')

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin_once(node, timeout_sec=3.0)  # Kjører én gang
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
