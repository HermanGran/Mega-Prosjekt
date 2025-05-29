import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class CubeDetector(Node):
    def __init__(self):
        super().__init__('cube_detector')
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Point, '/qube_centroid', 10)
        self.color_pub = self.create_publisher(String, '/qube_color', 10)
        self.debug_pub = self.create_publisher(Image, '/debug_image', 10)
        self.enable_sub = self.create_subscription(Bool, '/enable_detection', self.enable_callback, 10)
        self.missing_pub = self.create_publisher(String, '/qube_missing', 10)

        self.bridge = CvBridge()
        self.enabled = False
        self.detection_order = ['red', 'yellow', 'blue' ]
        self.current_index = 0

        self.color_ranges = {
            'red': [
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([160, 100, 100]), np.array([180, 255, 255]))
            ],
            'blue': [(np.array([100, 150, 50]), np.array([140, 255, 255]))],
            'yellow': [(np.array([20, 100, 100]), np.array([40, 255, 255]))]
        }

    def enable_callback(self, msg):
        if msg.data:
            self.current_index = 0  # Start på nytt
            self.enabled = True
            self.get_logger().info("Detection sequence started")

    def image_callback(self, image_msg):
        if not self.enabled or self.current_index >= len(self.detection_order):
            return

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        current_color = self.detection_order[self.current_index]
        ranges = self.color_ranges[current_color]

        mask_total = None
        for lower, upper in ranges:
            mask = cv2.inRange(hsv, lower, upper)
            mask_total = mask if mask_total is None else cv2.bitwise_or(mask_total, mask)

        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    point_msg = Point(x=float(cx), y=float(cy), z=0.0)
                    self.pub.publish(point_msg)

                    color_msg = String()
                    color_msg.data = current_color
                    self.color_pub.publish(color_msg)

                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_image, current_color, (x + 10, y + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    self.get_logger().info(f"{current_color} cube detected at ({cx}, {cy})")
                    found = True
                    break  # Bare detekter én kloss per bilde

        if not found:
            missing_msg = String()
            missing_msg.data = f"{current_color} not found"
            self.missing_pub.publish(missing_msg)
            self.get_logger().warn(f"{current_color} cube not found")

        # Gå videre til neste farge
        self.current_index += 1
        if self.current_index >= len(self.detection_order):
            self.enabled = False
            self.get_logger().info("Finished detection sequence")

        # Publiser debug-bilde
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
