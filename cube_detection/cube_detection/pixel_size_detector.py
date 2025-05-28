import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PixelSizeDetector(Node):
    def __init__(self):
        super().__init__('pixel_size_detector')
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Pixel Size Detector for red cube is running...")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # HSV-område for rød farge (to områder pga. HSV hue wrap)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2  # Kombiner begge røde områder

        # Finn konturer i masken
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Finn største kontur - antatt å være klossen
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Tegn grønn boks rundt klossen
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Vis både originalbilde med boks og masken
            cv2.imshow("Detected Red Cube", frame)
            cv2.imshow("Red Mask", mask)
            cv2.waitKey(1)

            self.get_logger().info(f"Rød kloss funnet: bredde={w} px, høyde={h} px")
        else:
            self.get_logger().info("Ingen rød kloss funnet i bildet")

def main(args=None):
    rclpy.init(args=args)
    node = PixelSizeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
