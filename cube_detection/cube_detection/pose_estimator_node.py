import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')
        self.subscription_point = self.create_subscription(Point, '/qube_centroid', self.point_callback, 10)
        self.subscription_color = self.create_subscription(String, '/qube_color', self.color_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/cube_pose', 10)

        self.last_color = "unknown"

        # Parametere du kan justere:
        self.camera_height = 0.5  # meter over bakken
        self.scale = 0.002  # meter per piksel (må kalibreres!)
        self.img_width = 640
        self.img_height = 480

    def color_callback(self, msg):
        self.last_color = msg.data

    def point_callback(self, point_msg):
        cx = point_msg.x
        cy = point_msg.y

        # Bildets midtpunkt
        center_x = self.img_width / 2
        center_y = self.img_height / 2

        # Konverter pikselkoordinater til verden (XY) med antatt flat vinkel
        x_world = (cx - center_x) * self.scale
        y_world = (cy - center_y) * self.scale
        z_world = 0.0  # fordi objektet ligger på gulvet

        # Lag og publiser Pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_frame'

        pose_msg.pose.position.x = x_world
        pose_msg.pose.position.y = y_world
        pose_msg.pose.position.z = z_world

        # Sett orientering til nøytral (kan utvides senere)
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0

        self.pose_publisher.publish(pose_msg)

        self.get_logger().info(f"Pose published for {self.last_color} cube: x={x_world:.2f}, y={y_world:.2f},  z={z_world:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
