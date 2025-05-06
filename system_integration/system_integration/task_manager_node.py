import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        self.get_logger().info('Task Manager node started')

        # Eksempel: lyttere og sendere
        self.detection_sub = self.create_subscription(
            PoseStamped,
            '/detected_cube',
            self.cube_callback,
            10
        )

        self.motion_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )

        self.current_color = 'red'
        self.cubes_found = {}

        # Start flow
        self.timer = self.create_timer(1.0, self.start_task_flow)

    def start_task_flow(self):
        self.get_logger().info('Starting cube detection for red cube...')
        # Send kommando til kamera/deteksjon hvis nødvendig

    def cube_callback(self, msg):
        self.get_logger().info(f"Detected cube: {msg}")
        # Her kunne du brukt navnet på fargen eller posisjonen
        self.cubes_found[self.current_color] = msg

        if self.current_color == 'red':
            self.current_color = 'yellow'
        elif self.current_color == 'yellow':
            self.current_color = 'blue'
        elif self.current_color == 'blue':
            self.get_logger().info('Alle kuber funnet.')
            self.timer.cancel()

        # Send pose videre til robotcontroller
        self.motion_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
