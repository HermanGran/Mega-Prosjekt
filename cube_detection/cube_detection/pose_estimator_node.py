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

        # Convert to normalized image coordinates
        x_normalized = (cx - self.img_width / 2) / self.focal_length
        y_normalized = (self.img_height / 2 - cy) / self.focal_length

        # Create ray from camera to point (in camera frame)
        ray_end = PointStamped()
        ray_end.header.stamp = self.get_clock().now().to_msg()
        ray_end.header.frame_id = "camera_frame"
        ray_end.point.x = x_normalized
        ray_end.point.y = y_normalized
        ray_end.point.z = 1.0  # Unit depth

        try:
            # Transform ray direction to base_link frame
            ray_end_base = self.tf_buffer.transform(
                ray_end,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=0.5))

            # Get camera position in base_link
            tf_camera = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_frame",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))

            #Camera position
            cam_x = tf_camera.transform.translation.x
            cam_y = tf_camera.transform.translation.y
            cam_z = tf_camera.transform.translation.z

            # Ray direction vector
            dir_x = ray_end_base.point.x - cam_x
            dir_y = ray_end_base.point.y - cam_y
            dir_z = ray_end_base.point.z - cam_z

            # Calculate intersection with ground plane (z = 0)
            if abs(dir_z) > 0.001:  # Avoid division by zero
                t = -cam_z / dir_z
                cube_x = cam_x + t * dir_x
                cube_y = cam_y + t * dir_y
                cube_z = 0.1
            else:
                self.get_logger().warn("Ray parallel to ground plane")
                return

            self.get_logger().info(f"Cube position: x={cube_x:.3f}, y={cube_y:.3f}")

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