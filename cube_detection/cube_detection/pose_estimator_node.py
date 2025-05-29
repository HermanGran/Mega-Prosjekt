import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        self.subscription_point = self.create_subscription(Point, '/qube_centroid', self.point_callback, 10)
        self.subscription_color = self.create_subscription(String, '/qube_color', self.color_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/cube_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/cube_markers', 10)

        self.last_color = "unknown"
        self.scale = 0.036 / 37
        self.img_width = 640
        self.img_height = 480
        self.camera_height = 0.5
        self.fixed_cube_height = 0.02  # Fast høyde for kuben (2 cm)
        self.focal_length = 500.0  # Fokal lengde i piksler

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def color_callback(self, msg):
        self.last_color = msg.data

    def point_callback(self, point_msg):
        cx = point_msg.x
        cy = point_msg.y

        # Konverter til posisjon i kamera-frame
        x_normalized = (cx - self.img_width / 2) / self.focal_length
        y_normalized = (self.img_height / 2 - cy) / self.focal_length

        ray_end = PointStamped()
        ray_end.header.stamp = self.get_clock().now().to_msg()
        ray_end.header.frame_id = "camera_frame"
        ray_end.point.x = x_normalized
        ray_end.point.y = y_normalized
        ray_end.point.z = 1.0  # Enhetsvektor i kameraets retning

        try:
            # Transformér posisjon fra camera_frame til base_link
            ray_end_base = self.tf_buffer.transform(
                ray_end,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Hent kameraposisjon i base_link
            tf_camera = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_frame",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Kamera posisjon
            cam_x = tf_camera.transform.translation.x
            cam_y = tf_camera.transform.translation.y
            cam_z = tf_camera.transform.translation.z

            # Retningsvektor fra kamera til strålens ende
            dir_x = ray_end_base.point.x - cam_x
            dir_y = ray_end_base.point.y - cam_y
            dir_z = ray_end_base.point.z - cam_z

            # Beregn skjæring med fast høyde-plan
            t = (self.fixed_cube_height - cam_z) / dir_z
            cube_x = cam_x + t * dir_x
            cube_y = cam_y + t * dir_y
            cube_z = self.fixed_cube_height

            self.get_logger().info(f"Kube posisjon i base_link: x={cube_x:.3f}, y={cube_y:.3f}, z={cube_z:.3f}")

            # Lag en ny PoseStamped i base_link med kubens posisjon
            cube_pose = PoseStamped()
            cube_pose.header.stamp = self.get_clock().now().to_msg()
            cube_pose.header.frame_id = "base_link"
            cube_pose.pose.position.x = cube_x
            cube_pose.pose.position.y = cube_y
            cube_pose.pose.position.z = cube_z
            cube_pose.pose.orientation.w = 1.0  # Ingen rotasjon

            # Publiser kubens posisjon i base_link
            self.pose_publisher.publish(cube_pose)

            # Lag marker for RViz med kubens posisjon i base_link
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cube"
            marker.id = int(time.time() * 1000) % 100000
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = cube_x
            marker.pose.position.y = cube_y
            marker.pose.position.z = cube_z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04

            # Sett farge basert på sist registrerte farge
            color_map = {
                'red': (1.0, 0.0, 0.0),
                'blue': (0.0, 0.0, 1.0),
                'yellow': (1.0, 1.0, 0.0),
            }
            r, g, b = color_map.get(self.last_color, (1.0, 1.0, 1.0))
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            self.marker_pub.publish(marker)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform feilet: {e}")
        except Exception as e:
            self.get_logger().warn(f"Generell feil: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()