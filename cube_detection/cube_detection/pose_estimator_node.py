import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point
import time


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        self.subscription_point = self.create_subscription(Point, '/qube_centroid', self.point_callback, 10)
        self.subscription_color = self.create_subscription(String, '/qube_color', self.color_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/cube_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/cube_markers', 10)

        self.last_color = "unknown"

        # Camera parameters (should be calibrated for your setup)
        self.img_width = 640
        self.img_height = 480
        self.camera_height = 0.5  # Height of camera above ground (meters)
        self.focal_length = 500.0  # Approximate focal length in pixels

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
        z_cam = self.fixed_cube_height  # Bruk fast høyde

        pose_cam = PoseStamped()
        pose_cam.header.stamp = self.get_clock().now().to_msg()
        pose_cam.header.frame_id = "camera_frame"
        pose_cam.pose.position.x = x_cam
        pose_cam.pose.position.y = y_cam
        pose_cam.pose.position.z = z_cam
        pose_cam.pose.orientation.w = 1.0

        try:
            # Transformér direkte til base_link
            transformed_pose = self.tf_buffer.transform(
                pose_cam,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=0.5))

            # Hent TCP-posisjon (camera_frame er montert på tool0)
            tf_tool = self.tf_buffer.lookup_transform(
                "base_link",
                "tool0",  # TCP
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))

            robot_x = tf_tool.transform.translation.x
            robot_y = tf_tool.transform.translation.y
            robot_z = tf_tool.transform.translation.z

            # Beregn korrekt kube-posisjon
            # Merk: transformed_pose er allerede i base_link, så vi må ikke addere robot_x/y direkte
            # Men siden kameraet er på roboten, må vi justere for kameraets offset

            # Finn offset mellom tool0 og camera_frame
            tf_camera_offset = self.tf_buffer.lookup_transform(
                "tool0",
                "camera_frame",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))

            offset_x = tf_camera_offset.transform.translation.x
            offset_y = tf_camera_offset.transform.translation.y
            offset_z = tf_camera_offset.transform.translation.z

            # Korriger for kameraets offset fra tool0
            cube_x = transformed_pose.pose.position.x - offset_x
            cube_y = transformed_pose.pose.position.y - offset_y
            cube_z = transformed_pose.pose.position.z - offset_z
            cube_z = self.fixed_cube_height  # Tving fast høyde

            # Resten av koden for publisering og visualisering...

            # Create PoseStamped in base_link
            cube_pose = PoseStamped()
            cube_pose.header.stamp = self.get_clock().now().to_msg()
            cube_pose.header.frame_id = "base_link"
            cube_pose.pose.position.x = cube_x
            cube_pose.pose.position.y = cube_y
            cube_pose.pose.position.z = cube_z
            cube_pose.pose.orientation.w = 1.0

            # Publish pose
            self.pose_publisher.publish(cube_pose)

            # Create marker for RViz
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cube"
            marker.id = int(time.time() * 1000) % 100000
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = cube_pose.pose
            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04

            # Set color based on detected color
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
            self.get_logger().warn(f"TF transform failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()