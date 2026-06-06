import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import CameraInfo
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class PixelToWorld(Node):
    def __init__(self):
        super().__init__('pixel_to_world')

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        self.table_z = 0.515

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        