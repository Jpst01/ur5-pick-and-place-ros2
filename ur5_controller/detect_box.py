import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectBox(Node):
    def __init__(self):
        super().__init__('detect_box')
        
        # Create a CvBridge to convert ROS Image → OpenCV
        self.bridge = CvBridge()
        
        # Subscribe to the camera image topic
        self.sub = self.create_subscription(
            Image,
            '/world/pick_world/model/ur/link/wrist_3_link/sensor/wrist_camera/image',
            self.image_callback,
            10
        )
        
        # Publisher for detected box pixel coordinates
        self.pub = self.create_publisher(Point, '/detected_box', 10)
        
        self.get_logger().info('DetectBox node started')

    def image_callback(self, msg):
        # Step 1: Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Step 2: Convert BGR to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Step 3: Create a mask for red color
        # Red in HSV has two ranges (it wraps around):
        #   Range 1: H=0-10,  S=120-255, V=70-255
        #   Range 2: H=170-180, S=120-255, V=70-255
        mask1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        mask = mask1 | mask2
        
        # Step 4: Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Step 5: Find the largest contour (that's the red box)
        if len(contours) == 0:
            return
        largest = max(contours, key=cv2.contourArea)
        
        # Step 6: Get the centroid using moments
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        
        # Step 7: Publish the pixel coordinates
        # Create a Point msg with x=cx, y=cy, z=0
        point = Point()
        point.x = float(cx)
        point.y = float(cy)
        point.z = 0.0
        self.pub.publish(point)
        self.get_logger().info(f'Read box at pixel ({cx}, {cy})')
        
        # Step 8 (optional): Show the image with detection drawn
        cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
        cv2.imshow('Detection', frame)
        cv2.waitKey(1)
        
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DetectBox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
