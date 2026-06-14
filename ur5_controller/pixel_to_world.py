import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
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
            String,
            '/detected_objects',
            self.detected_box_callback,
            10
        )

        self.pub = self.create_publisher(String, '/box_world_poses', 10)

        self.get_logger().info('PixelToWorld node started')

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def detected_box_callback(self, msg):
        if self.fx is None:
            return

        parts = msg.data.split(':')
        if len(parts) != 3:
            return
        color = parts[0]
        pixel_x = float(parts[1])
        pixel_y = float(parts[2])

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

        out = String()
        out.data = f"{color}:{point_world.point.x:.4f}:{point_world.point.y:.4f}:{point_world.point.z:.4f}"
        self.pub.publish(out)
        self.get_logger().info(f'{color} box at world: ({point_world.point.x:.3f}, {point_world.point.y:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = PixelToWorld()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
