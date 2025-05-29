import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        self.cli = self.create_client(Trigger, '/go_to_home')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Venter på /go_to_home tjeneste...')

        self.get_logger().info('Kaller /go_to_home...')
        self.go_to_home()

        self.target_colors = ['red', 'yellow', 'blue']
        self.current_target_index = 0

        self.last_color = None
        self.last_pose = None
        self.waiting_for_pose = False
        self.has_received_color = False
        self.has_received_pose = False

        self.create_subscription(String, '/qube_color', self.color_callback, 10)
        self.create_subscription(PoseStamped, '/cube_pose', self.pose_callback, 10)

        self.pose_order_pub = self.create_publisher(PoseStamped, '/cube_pose_order', 10)

        self.create_service(Trigger, '/cube_pose_done', self.task_done_callback)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def go_to_home(self):
        self.get_logger().info('[go_to_home] Kaller hjem-funksjon...')
        req = Trigger.Request()
        future = self.cli.call_async(req)

        def callback(future):
            if future.result() is not None and future.result().success:
                self.get_logger().info('[go_to_home] Gikk til hjem-posisjon.')
                self.waiting_for_pose = True
            else:
                self.get_logger().error('[go_to_home] Kunne ikke gå til hjem-posisjon.')

        future.add_done_callback(callback)

    def color_callback(self, msg):
        self.last_color = msg.data.lower()
        self.has_received_color = True
        self.get_logger().info(f"[color_callback] Mottatt farge: {self.last_color}")

    def pose_callback(self, msg):
        self.last_pose = msg
        self.has_received_pose = True
        self.get_logger().info(f"[pose_callback] Mottatt pose")

    def timer_callback(self):
        self.get_logger().info("[timer_callback] Sjekker status...")

        if not self.has_received_color:
            self.get_logger().info("[timer_callback] Har ikke mottatt farge ennå.")
            return

        if not self.has_received_pose:
            self.get_logger().info("[timer_callback] Har ikke mottatt pose ennå.")
            return

        if not self.waiting_for_pose:
            self.get_logger().info("[timer_callback] Venter ikke på ny kube (venter på at forrige skal bli ferdig?).")
            return

        if self.current_target_index >= len(self.target_colors):
            self.get_logger().info("[timer_callback] Alle kuber håndtert.")
            return

        current_target_color = self.target_colors[self.current_target_index]
        self.get_logger().info(f"[timer_callback] Venter på farge: {current_target_color}, mottatt: {self.last_color}")

        if self.last_color == current_target_color and self.last_pose is not None:
            self.get_logger().info(f"[timer_callback] Farge matcher ({current_target_color}). Publiserer pose.")
            self.pose_order_pub.publish(self.last_pose)

            self.current_target_index += 1
            self.last_pose = None
            self.waiting_for_pose = False
            self.has_received_color = False
            self.has_received_pose = False
        else:
            self.get_logger().info("[timer_callback] Farge matcher ikke eller pose er None. Venter...")

    def task_done_callback(self, request, response):
        self.get_logger().info("[task_done_callback] Robot har fullført oppgave. Starter ny runde.")
        self.go_to_home()
        response.success = True
        response.message = "Neste kube kan hentes"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
