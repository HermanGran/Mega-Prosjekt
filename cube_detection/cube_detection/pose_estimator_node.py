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

        self.subscription_point = self.create_subscription(
            Point, '/qube_centroid', self.point_callback, 10)
        self.subscription_color = self.create_subscription(
            String, '/qube_color', self.color_callback, 10)
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/cube_pose', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/cube_markers', 10)

        self.last_color = "unknown"

        # Camera parameters - these should be calibrated for your specific setup
        self.img_width = 640      # Camera resolution width in pixels
        self.img_height = 480     # Camera resolution height in pixels
        self.camera_height = 0.6  # Height of camera above ground (meters)
        self.focal_length = 530.0 # Focal length in pixels (adjust based on calibration)
        self.fixed_cube_height = 0.02  # Height of cube (meters)

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera offset from tool0 (if known)
        self.camera_offset_x = 0.0  # Adjust if camera is not centered on tool0
        self.camera_offset_y = 0.0
        self.camera_offset_z = 0.0

    def color_callback(self, msg):
        self.last_color = msg.data

    def point_callback(self, point_msg):
        cx = point_msg.x
        cy = point_msg.y

        # Convert from pixel coordinates to camera frame coordinates
        # Using perspective projection (pinhole camera model)
        x_cam = (cx - self.img_width/2) * (self.fixed_cube_height / self.focal_length)
        y_cam = (cy - self.img_height/2) * (self.fixed_cube_height / self.focal_length)
        z_cam = self.fixed_cube_height

        # Create PointStamped in camera frame
        point_cam = PointStamped()
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.header.frame_id = "camera_frame"
        point_cam.point.x = x_cam
        point_cam.point.y = y_cam
        point_cam.point.z = z_cam

        try:
            # First transform to tool0 frame to account for camera mounting
            point_tool0 = self.tf_buffer.transform(
                point_cam,
                "tool0",
                timeout=rclpy.duration.Duration(seconds=0.5))

            # Apply any static camera offset from tool0
            point_tool0.point.x += self.camera_offset_x
            point_tool0.point.y += self.camera_offset_y
            point_tool0.point.z += self.camera_offset_z

            # Then transform to base_link frame
            transformed_point = self.tf_buffer.transform(
                point_tool0,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=0.5))

            # Create PoseStamped in base_link frame
            cube_pose = PoseStamped()
            cube_pose.header.stamp = self.get_clock().now().to_msg()
            cube_pose.header.frame_id = "base_link"
            cube_pose.pose.position.x = transformed_point.point.x
            cube_pose.pose.position.y = transformed_point.point.y
            cube_pose.pose.position.z = self.fixed_cube_height  # Force fixed height
            cube_pose.pose.orientation.w = 1.0  # Neutral orientation

            # Publish pose
            self.pose_publisher.publish(cube_pose)

            # Create marker for visualization
            self.publish_marker(cube_pose)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform failed: {e}")

    def publish_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "cube"
        marker.id = int(time.time() * 1000) % 100000
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose.pose
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


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()