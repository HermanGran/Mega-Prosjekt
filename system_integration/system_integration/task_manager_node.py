import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # Klargjør serviceklient for /go_to_home
        self.cli = self.create_client(Trigger, '/go_to_home')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Venter på /go_to_home tjeneste...')

        self.get_logger().info('Kaller /go_to_home...')
        self.go_to_home()

        # Oppsett av fargerekkefølge
        self.target_colors = ['red', 'yellow', 'blue']
        self.current_target_index = 0

        # Statusvariabler
        self.last_color = None
        self.last_pose = None
        self.waiting_for_pose = False
        self.has_received_color = False
        self.has_received_pose = False
        self.delayed_home_timer = None

        # Abonnementer
        self.create_subscription(String, '/qube_color', self.color_callback, 10)
        self.create_subscription(PoseStamped, '/cube_pose', self.pose_callback, 10)

        # Publisher for å sende sortert pose
        self.pose_order_pub = self.create_publisher(PoseStamped, '/cube_pose_order', 10)

        # Sjekker jevnlig om vi kan sende neste pose
        self.timer = self.create_timer(1.0, self.timer_callback)

    def go_to_home(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)

        def callback(future):
            if future.result() is not None and future.result().success:
                self.get_logger().info('Gikk til hjem-posisjon.')
                self.waiting_for_pose = True
            else:
                self.get_logger().error('Kunne ikke gå til hjem-posisjon.')

        future.add_done_callback(callback)

    def color_callback(self, msg):
        self.get_logger().info(f"Mottatt farge: {msg.data}")
        self.last_color = msg.data.lower()
        self.has_received_color = True

    def pose_callback(self, msg):
        self.get_logger().info(f"Mottatt pose: {msg.pose}")
        self.last_pose = msg
        self.has_received_pose = True

    def timer_callback(self):
        if not self.has_received_color or not self.has_received_pose:
            if not hasattr(self, 'has_logged_waiting') or not self.has_logged_waiting:
                self.get_logger().info("Venter på første farge og pose fra kamera...")
                self.has_logged_waiting = True
            return

        if not self.waiting_for_pose:
            return

        if self.current_target_index >= len(self.target_colors):
            self.get_logger().info("Alle kuber håndtert.")
            return

        current_target_color = self.target_colors[self.current_target_index]

        if self.last_color == current_target_color and self.last_pose is not None:
            self.get_logger().info(f'Fant {current_target_color}-kube. Publiserer til /cube_pose_order.')
            self.pose_order_pub.publish(self.last_pose)

            self.current_target_index += 1
            self.last_pose = None
            self.has_received_color = False
            self.has_received_pose = False
            self.waiting_for_pose = False

            self.get_logger().info("Starter 5 sek forsinkelse før retur til home...")
            self.delayed_home_timer = self.create_timer(5.0, self.delayed_go_to_home)

    def delayed_go_to_home(self):
        if self.delayed_home_timer:
            self.delayed_home_timer.cancel()
            self.delayed_home_timer = None

        self.get_logger().info("Kaller go_to_home etter forsinkelse...")
        self.go_to_home()

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
