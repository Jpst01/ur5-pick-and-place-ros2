"""Bridges the red_box pose from Gazebo → ROS2.

Subscribes to /world/pick_world/dynamic_pose/info (bridged from Gazebo)
and publishes the red box pose on /red_box_pose for Unity.
"""
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
        # The red_box is the only dynamic model in the scene.
        # Its pose is the first transform with position near z≈0.5 (on the table).
        # If the box is picked up, z will change accordingly.
        for t in msg.transforms:
            pos = t.transform.translation
            # Filter: the red_box starts near (0.4, 0, 0.515)
            # The UR5 robot base is at (0,0,0) — its transforms have x≈0
            # So any transform with x > 0.2 is likely the box
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
