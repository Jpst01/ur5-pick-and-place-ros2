import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import JointState
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

        self.detected_x = None
        self.detected_y = None
        self.create_subscription(
            PointStamped,
            '/box_world_pose',
            self.detection_callback,
            10
        )

        # Track joint velocities to know when arm has settled
        self.joint_velocities = []
        self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
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
        self.gripper_command(1.0)

    def detection_callback(self, msg):
        self.detected_x = msg.point.x
        self.detected_y = msg.point.y

    def joint_state_callback(self, msg):
        self.joint_velocities = list(msg.velocity) if msg.velocity else []

    def run(self):
        GRASP_Z = 0.68
        PRE_GRASP_Z = 0.72
        LIFT_Z = 0.72
        QX, QY, QZ, QW = 0.0, 1.0, 0.0, 0.0

        self.get_logger().info('Starting Pick and Place')

        # Wait for arm to settle (joint velocities near zero)
        self.get_logger().info('Waiting for arm to settle...')
        for i in range(100):  # 10s max
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_velocities and all(abs(v) < 0.01 for v in self.joint_velocities):
                self.get_logger().info('Arm settled!')
                break
        else:
            self.get_logger().warn('Arm may not be fully settled, proceeding anyway')

        self.open_gripper()

        # Move to home first (joint-space, more robust)
        self.get_logger().info('Step 0: Moving to home position...')
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='home')
        plan_result = self.arm.plan()
        if plan_result:
            self.moveit.execute(plan_result.trajectory, controllers=[])
            time.sleep(5)
        self.get_logger().info('At home position')

        # Now scan for box
        self.get_logger().info('Step 1: Scanning for box...')
        if not self.move_to_pose(0.4, 0.0, PRE_GRASP_Z, QX, QY, QZ, QW):
            return

        # Reset any stale detection
        self.detected_x = None
        self.detected_y = None

        self.get_logger().info('Waiting for box detection...')
        readings_x = []
        readings_y = []
        for i in range(100):  # 10 seconds max
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.detected_x is not None:
                readings_x.append(self.detected_x)
                readings_y.append(self.detected_y)
                self.detected_x = None
                self.detected_y = None
                if len(readings_x) >= 5:  # average 5 readings
                    break

        if len(readings_x) == 0:
            self.get_logger().error('Box not detected')
            return

        BOX_X = sum(readings_x) / len(readings_x)
        BOX_Y = sum(readings_y) / len(readings_y)
        self.get_logger().info(f'Raw detection: ({BOX_X:.3f}, {BOX_Y:.3f}) from {len(readings_x)} readings')

        # Calibration offsets (tune these if gripper misses the box)
        OFFSET_X = 0.005   # positive = move gripper further from robot
        OFFSET_Y = 0.02    # positive = move gripper to the left
        BOX_X += OFFSET_X
        BOX_Y += OFFSET_Y
        self.get_logger().info(f'Adjusted target: ({BOX_X:.3f}, {BOX_Y:.3f})')

        self.get_logger().info('Step 2: Pre-Grasp')
        if not self.move_to_pose(BOX_X, BOX_Y, PRE_GRASP_Z, QX, QY, QZ, QW):
            return

        self.get_logger().info('Step 3: Grasp')
        if not self.move_to_pose(BOX_X, BOX_Y, GRASP_Z, QX, QY, QZ, QW):
            return

        self.close_gripper()

        self.get_logger().info('Step 4: Lift')
        if not self.move_to_pose(BOX_X, BOX_Y, LIFT_Z, QX, QY, QZ, QW):
            return

        self.get_logger().info('Step 5: Move to drop')
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
    time.sleep(10)  # Wait for arm to settle in Gazebo
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()