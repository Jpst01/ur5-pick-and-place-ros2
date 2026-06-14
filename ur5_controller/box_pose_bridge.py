import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage


class BoxPoseBridge(Node):
    def __init__(self):
        super().__init__('box_pose_bridge')
        self.pub = self.create_publisher(Pose, '/red_box_pose', 10)
        self.sub = self.create_subscription(
            TFMessage,
            '/world/pick_world/dynamic_pose/info',
            self.on_pose,
            10
        )
        self.get_logger().info('BoxPoseBridge: listening on /world/pick_world/dynamic_pose/info')

    def on_pose(self, msg):
        for t in msg.transforms:
            pos = t.transform.translation
            if abs(pos.x - 0.4) < 0.3 and pos.z > 0.01:
                pose = Pose()
                pose.position.x = pos.x
                pose.position.y = pos.y
                pose.position.z = pos.z
                pose.orientation.x = t.transform.rotation.x
                pose.orientation.y = t.transform.rotation.y
                pose.orientation.z = t.transform.rotation.z
                pose.orientation.w = t.transform.rotation.w
                self.pub.publish(pose)
                return


def main(args=None):
    rclpy.init(args=args)
    node = BoxPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
