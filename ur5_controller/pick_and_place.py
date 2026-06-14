import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
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
        self.detected_color = None
        self.create_subscription(
            String,
            '/box_world_poses',
            self.detection_callback,
            10
        )

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
        self.gripper_command(0.95, max_effort=200.0)

    def detection_callback(self, msg):
        parts = msg.data.split(':')
        if len(parts) != 4:
            return
        self.detected_color = parts[0]
        self.detected_x = float(parts[1])
        self.detected_y = float(parts[2])

    def joint_state_callback(self, msg):
        self.joint_velocities = list(msg.velocity) if msg.velocity else []

    def run(self):
        GRASP_Z = 0.68
        PRE_GRASP_Z = 0.72
        LIFT_Z = 0.72
        QX, QY, QZ, QW = 0.0, 1.0, 0.0, 0.0

        self.get_logger().info('Starting Pick and Place')

        self.get_logger().info('Waiting for arm to settle...')
        for i in range(100):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_velocities and all(abs(v) < 0.01 for v in self.joint_velocities):
                self.get_logger().info('Arm settled!')
                break
        else:
            self.get_logger().warn('Arm may not be fully settled, proceeding anyway')

        self.open_gripper()

        self.get_logger().info('Step 0: Moving to home position...')
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='home')
        plan_result = self.arm.plan()
        if plan_result:
            self.moveit.execute(plan_result.trajectory, controllers=[])
            time.sleep(5)
        self.get_logger().info('At home position')

        RED_BIN = (0.2, 0.3)
        BLUE_BIN = (0.2, -0.3)

        OFFSET_X = 0.0
        OFFSET_Y = 0.0

        TABLE_X_MIN, TABLE_X_MAX = 0.25, 0.55
        TABLE_Y_MIN, TABLE_Y_MAX = -0.15, 0.15

        SCAN_POSITIONS = [
            (0.40, 0.0),
            (0.40, 0.08),
            (0.40, -0.08),
            (0.45, 0.0),
            (0.35, 0.0),
            (0.45, 0.08),
            (0.35, -0.08),
            (0.40, 0.04),
            (0.40, -0.04),
            (0.45, -0.08),
        ]
        total_picked = 0
        MAX_ROUNDS = 10
        for scan_round in range(MAX_ROUNDS):
            scan_x, scan_y = SCAN_POSITIONS[scan_round % len(SCAN_POSITIONS)]
            self.get_logger().info(f'=== Scan round {scan_round + 1} ({scan_x:.2f}, {scan_y:.2f}) ===')

            if not self.move_to_pose(scan_x, scan_y, PRE_GRASP_Z, QX, QY, QZ, QW):
                break

            self.detected_x = None
            self.detected_y = None
            self.detected_color = None

            self.get_logger().info('Collecting detections...')
            raw_detections = []
            for i in range(50):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.detected_x is not None:
                    x, y = self.detected_x, self.detected_y
                    if TABLE_X_MIN <= x <= TABLE_X_MAX and TABLE_Y_MIN <= y <= TABLE_Y_MAX:
                        raw_detections.append((self.detected_color, x, y))
                    self.detected_x = None
                    self.detected_y = None
                    self.detected_color = None

            boxes = []
            for color, x, y in raw_detections:
                found = False
                for box in boxes:
                    if box['color'] == color and abs(box['x'] - x) < 0.05 and abs(box['y'] - y) < 0.05:
                        box['readings'].append((x, y))
                        box['x'] = sum(r[0] for r in box['readings']) / len(box['readings'])
                        box['y'] = sum(r[1] for r in box['readings']) / len(box['readings'])
                        found = True
                        break
                if not found:
                    boxes.append({'color': color, 'x': x, 'y': y, 'readings': [(x, y)]})

            if len(boxes) == 0:
                self.get_logger().info('No more boxes found on table')
                break

            boxes = [b for b in boxes if len(b['readings']) >= 3]
            if len(boxes) == 0:
                self.get_logger().info('No reliable detections (too few readings)')
                break

            self.get_logger().info(f'Found {len(boxes)} boxes:')
            for b in boxes:
                self.get_logger().info(f'  {b["color"]} at ({b["x"]:.3f}, {b["y"]:.3f}) [{len(b["readings"])} readings]')

            boxes.sort(key=lambda b: (b['x'] - scan_x)**2 + (b['y'] - scan_y)**2)

            box = boxes[0]
            color = box['color']
            BOX_X = box['x'] + OFFSET_X
            BOX_Y = box['y'] + OFFSET_Y
            bin_pos = RED_BIN if color == 'red' else BLUE_BIN

            self.get_logger().info(f'--- Picking {color} box at ({BOX_X:.3f}, {BOX_Y:.3f}) ---')

            self.open_gripper()

            if not self.move_to_pose(BOX_X, BOX_Y, PRE_GRASP_Z, QX, QY, QZ, QW):
                continue

            self.detected_x = None
            self.detected_y = None
            self.detected_color = None
            re_readings = []
            for i in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.detected_x is not None:
                    if (self.detected_color == color and
                        abs(self.detected_x - BOX_X) < 0.05 and
                        abs(self.detected_y - BOX_Y) < 0.05 and
                        TABLE_X_MIN <= self.detected_x <= TABLE_X_MAX and
                        TABLE_Y_MIN <= self.detected_y <= TABLE_Y_MAX):
                        re_readings.append((self.detected_x, self.detected_y))
                    self.detected_x = None
                    self.detected_y = None
                    self.detected_color = None

            if len(re_readings) >= 3:
                BOX_X = sum(r[0] for r in re_readings) / len(re_readings)
                BOX_Y = sum(r[1] for r in re_readings) / len(re_readings)
                self.get_logger().info(f'Re-detected center: ({BOX_X:.3f}, {BOX_Y:.3f})')
                if not self.move_to_pose(BOX_X, BOX_Y, PRE_GRASP_Z, QX, QY, QZ, QW):
                    continue

            if not self.move_to_pose(BOX_X, BOX_Y, GRASP_Z, QX, QY, QZ, QW):
                continue

            self.close_gripper()

            if not self.move_to_pose(BOX_X, BOX_Y, LIFT_Z, QX, QY, QZ, QW):
                continue

            self.get_logger().info(f'Dropping in {color} bin')
            if not self.move_to_pose(bin_pos[0], bin_pos[1], LIFT_Z, QX, QY, QZ, QW):
                continue

            self.open_gripper()
            time.sleep(1.0)
            total_picked += 1

        self.get_logger().info(f'Done! Picked {total_picked} boxes total')

        self.get_logger().info('Returning home...')
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name='home')
        plan_result = self.arm.plan()
        if plan_result:
            self.moveit.execute(plan_result.trajectory, controllers=[])
        self.get_logger().info('Pick and Sort Complete!')

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    time.sleep(10)
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()