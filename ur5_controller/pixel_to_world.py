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

        self.create_subscription(
            CameraInfo,
            '/world/pick_world/model/ur/link/wrist_3_link/sensor/wrist_camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.create_subscription(
            Point,
            '/detected_box',
            self.detected_box_callback,
            10
        )

        self.pub = self.create_publisher(PointStamped, '/box_world_pose', 10)

        self.get_logger().info('PixelToWorld node started')

    def camera_info_callback(self, msg):
        
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        pass

    def detected_box_callback(self, msg):
        if self.fx is None:
            return

        pixel_x = msg.x
        pixel_y = msg.y

        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'wrist_camera_optical_link',
                rclpy.time.Time()
            )
        except tf2_ros.LookupException:
            return

        depth = transform.transform.translation.z - self.table_z
        
        x_cam = (pixel_x - self.cx) * depth / self.fx
        y_cam = (pixel_y - self.cy) * depth / self.fy
        z_cam = depth
        
        point_cam = PointStamped()
        point_cam.header.frame_id = 'wrist_camera_optical_link'
        
        point_cam.point.x = x_cam
        point_cam.point.y = y_cam
        point_cam.point.z = z_cam
        
        point_world = do_transform_point(point_cam, transform)
        
        self.pub.publish(point_world)
        self.get_logger().info(f'Box at world: ({point_world.point.x:.3f}, {point_world.point.y:.3f}, {point_world.point.z:.3f})')

        pass


def main(args=None):
    rclpy.init(args=args)
    node = PixelToWorld()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
