import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectObjects(Node):
    def __init__(self):
        super().__init__('detect_objects')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/world/pick_world/model/ur/link/wrist_3_link/sensor/wrist_camera/image',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(String, '/detected_objects', 10)

        self.color_ranges = {
            'red': {
                'lower1': (0, 120, 70),
                'upper1': (10, 255, 255),
                'lower2': (170, 120, 70),
                'upper2': (180, 255, 255),
            },
            'blue': {
                'lower1': (100, 120, 70),
                'upper1': (130, 255, 255),
            }
        }

        self.get_logger().info('DetectObjects node started')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for color, ranges in self.color_ranges.items():
            if "lower1" in ranges:
                lower1 = np.array(ranges["lower1"])
            if "upper1" in ranges:
                upper1 = np.array(ranges["upper1"])
            if "lower2" in ranges:
                lower2 = np.array(ranges["lower2"])
            if "upper2" in ranges:
                upper2 = np.array(ranges["upper2"])

            if "lower1" in ranges and "upper1" in ranges:
                mask1 = cv2.inRange(hsv, lower1, upper1)

            if "lower2" in ranges and "upper2" in ranges:
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = mask1

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for c in contours:
                area = cv2.contourArea(c)
                if 100 < area < 15000:
                    M = cv2.moments(c)
                    if M["m00"] == 0:
                        continue
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                    self.pub.publish(String(data=f"{color}:{cx}:{cy}"))

        cv2.imshow('Detection', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectObjects()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
