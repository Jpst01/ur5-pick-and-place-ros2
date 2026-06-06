import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.get_logger().info('Pick and Place Node Started')

        self.moveit = MoveItPy(node_name='moveit_py')
        self.arm = self.moveit.get_planning_component('ur_manipulator')
        self.get_logger().info('Connected to MoveIt2')

        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

    def move_to_pose(self, x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'world'
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link='tool0'
        )
        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info(f'Executing move to ({x:.2f}, {y:.2f}, {z:.2f})')
            self.moveit.execute(plan_result.trajectory, controllers=[])
            time.sleep(3)
            return True
        else:
            self.get_logger().error(f'Planning failed for ({x:.2f}, {y:.2f}, {z:.2f})')
            return False

    def gripper_command(self, position, max_effort=50.0):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self.gripper_client.wait_for_server(timeout_sec=5.0)
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        time.sleep(1.0)

    def open_gripper(self):
        self.get_logger().info('Opening Gripper')
        self.gripper_command(0.0)

    def close_gripper(self):
        self.get_logger().info('Closing Gripper')
        self.gripper_command(0.8)

    def run(self):
        BOX_X = 0.4
        BOX_Y = 0.0
        GRASP_Z = 0.68
        PRE_GRASP_Z = 0.72
        LIFT_Z = 0.72
        QX, QY, QZ, QW = 0.0, 1.0, 0.0, 0.0

        self.get_logger().info('Starting Pick and Place')
        self.open_gripper()

        self.get_logger().info('Step 1: Pre-Grasp')
        if not self.move_to_pose(BOX_X, BOX_Y, PRE_GRASP_Z, QX, QY, QZ, QW):
            return

        self.get_logger().info('Step 2: Grasp')
        if not self.move_to_pose(BOX_X, BOX_Y, GRASP_Z, QX, QY, QZ, QW):
            return

        self.close_gripper()

        self.get_logger().info('Step 3: Lift')
        if not self.move_to_pose(BOX_X, BOX_Y, LIFT_Z, QX, QY, QZ, QW):
            return

        self.get_logger().info('Step 4: Move to drop')
        if not self.move_to_pose(0.2, 0.35, LIFT_Z, QX, QY, QZ, QW):
            return

        self.open_gripper()
        # Extra wait for box to fall
        time.sleep(2.0)

        self.get_logger().info('Step 5: Return home')
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='home')
        plan_result = self.arm.plan()
        if plan_result:
            self.moveit.execute(plan_result.trajectory, controllers=[])

        self.get_logger().info('Pick and Place Complete')

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    time.sleep(5)
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()