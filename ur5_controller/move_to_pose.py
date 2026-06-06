import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
import time

class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__('move_to_pose')
        self.get_logger().info('Node Started')

        self.moveit = MoveItPy(node_name = 'moveit_py')

        self.arm = self.moveit.get_planning_component('ur_manipulator')

        self.get_logger().info('Connected to MoveIt2')

    def move_to_pose(self, x, y, z):

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'base_link'

        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 1.0
        pose_goal.pose.orientation.z = 0.0
        pose_goal.pose.orientation.w = 0.0

        self.arm.set_goal_state(
            pose_stamped_msg = pose_goal,
            pose_link = 'tool0'
        )

        plan_result = self.arm.plan()

        if plan_result:
            self.get_logger().info('Plan found - executing')
            self.moveit.execute(plan_result.trajectory, controllers=[])
        else:
            self.get_logger().error('Planning Failed')
    
def main(args=None):
    rclpy.init(args=args)
    node = MoveToPoseNode()
    time.sleep(5)
    node.move_to_pose(x=0.3, y=0.2, z=0.5)

    rclpy.shutdown()

if __name__ == '__main__':
    main()