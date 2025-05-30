import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
import time


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')
        # Creating subscribtions
        self.subscription_point = self.create_subscription(Point, '/qube_centroid', self.point_callback, 10)
        self.subscription_color = self.create_subscription(String, '/qube_color', self.color_callback, 10)
        self.subscription_completion = self.create_subscription(Bool, '/check_completion', self.check_completion, 10)

        # Creating Publishers
        self.pose_publisher = self.create_publisher(PoseStamped, '/cube_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/cube_markers', 10)
        self.search_request_pub = self.create_publisher(String, '/request_search', 10)
        
        self.last_color = "unknown"
        self.scale = 0.036 / 37
        self.img_width = 640
        self.img_height = 480
        self.camera_height = 0.6
        self.found_cubes = {'red': None, 'blue': None, 'yellow': None}

        # TF Setup
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
        z_cam = 0.0  # Kuben på bakken

        pose_cam = PoseStamped()
        pose_cam.header.stamp = self.get_clock().now().to_msg()
        pose_cam.header.frame_id = "camera_frame"
        pose_cam.pose.position.x = x_cam
        pose_cam.pose.position.y = y_cam
        pose_cam.pose.position.z = z_cam
        pose_cam.pose.orientation.w = 1.0  # Ingen rotasjon

        try:
            # Transformér posisjon fra camera_frame til base_link
            transformed_pose = self.tf_buffer.transform(
                pose_cam,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Hent TCP-posisjon i base_link
            try:
                tf_tool = self.tf_buffer.lookup_transform(
                    "base_link",
                    "tool0",  # End-effektor (TCP)
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                robot_x = tf_tool.transform.translation.x
                robot_y = tf_tool.transform.translation.y

                # Regn ut kubens posisjon i base_link-koordinater ved å legge på TCP-posisjonen
                cube_x = robot_x + transformed_pose.pose.position.y
                cube_y = robot_y + transformed_pose.pose.position.x
                cube_z = 0.1  # Eventuelt sett til ønsket høyde over bakken

                # self.get_logger().info(f"Kube posisjon i base_link: x={cube_x:.3f}, y={cube_y:.3f}, z={cube_z:.3f}")

                # Lag en ny PoseStamped i base_link med kubens posisjon
                cube_pose = PoseStamped()
                cube_pose.header.stamp = self.get_clock().now().to_msg()
                cube_pose.header.frame_id = "base_link"
                cube_pose.pose.position.x = cube_x
                cube_pose.pose.position.y = cube_y
                cube_pose.pose.position.z = cube_z
                cube_pose.pose.orientation.w = 1.0  # Dummy value, is overwritten in planner_node

                # Store found cube position
                self.found_cubes[self.last_color] = cube_pose

                # Publish marker
                self.publish_marker(cube_pose)

                # Check if all cubes found
            except Exception as e:
                self.get_logger().warn(f"Kunne ikke hente robotposisjon (tool0): {e}")

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF transform feilet: {e}")

    def publish_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "cube"
        color_id_map = {'red': 0, 'blue': 1, 'yellow': 2}
        marker.id = color_id_map.get(self.last_color, 99)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04

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

    def check_completion(self, msg):
        if msg.data:  # Only check when requested
            missing = [color for color, pos in self.found_cubes.items() if pos is None]
            self.get_logger().info(f"Current cubes missing: {missing}")
            
            if not missing:  # Only if ALL cubes are found
                self.get_logger().info("All cubes found! Publishing poses...")
                for color, pose in self.found_cubes.items():
                    if pose is not None:
                        self.pose_publisher.publish(pose)
                        self.get_logger().info(f"Published {color} cube pose")
            else:
                search_msg = String()
                search_msg.data = ",".join(missing)
                self.search_request_pub.publish(search_msg)
                self.get_logger().info(f"Requesting search for missing cubes: {missing}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()