import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        self.subscription_point = self.create_subscription(Point, '/qube_centroid', self.point_callback, 10)
        self.subscription_color = self.create_subscription(String, '/qube_color', self.color_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/cube_pose', 10)

        self.last_color = "unknown"
        self.scale = 0.036 / 37
        self.img_width = 640
        self.img_height = 480
        self.camera_height = 0.5

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def color_callback(self, msg):
        self.last_color = msg.data

    def point_callback(self, point_msg):
        cx = point_msg.x
        cy = point_msg.y

        # Konverter til posisjon i kamera-frame
        x_cam = (cx - self.img_width / 2) * self.scale
        y_cam = (cy - self.img_height / 2) * self.scale
        z_cam = 0.0  # ligger på bakken fra kameraets perspektiv

        pose_cam = PoseStamped()
        pose_cam.header.stamp = self.get_clock().now().to_msg()
        pose_cam.header.frame_id = "camera_frame"
        pose_cam.pose.position.x = x_cam
        pose_cam.pose.position.y = y_cam
        pose_cam.pose.position.z = z_cam

        # Orientation can be default/identity (doesn’t matter here)
        pose_cam.pose.orientation.w = 1.0

        try:
            transformed_pose = self.tf_buffer.transform(pose_cam, "base_link", timeout=rclpy.duration.Duration(seconds=0.5))

            self.pose_publisher.publish(transformed_pose)
            '''
            self.get_logger().info(
                f"Published transformed pose for {self.last_color} cube: x={transformed_pose.pose.position.x:.2f}, "
                f"y={transformed_pose.pose.position.y:.2f}, z={transformed_pose.pose.position.z:.2f}"
            )
            '''
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

