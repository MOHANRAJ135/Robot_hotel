# delivery_robot_simple.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, qos_profile_system_default
from rclpy.duration import Duration

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from enum import Enum

class RobotState(Enum):
    IDLE = 0
    GOING_TO_KITCHEN = 1
    GOING_TO_TABLE = 2
    RETURNING_HOME = 3

class DeliveryRobot(Node):
    def __init__(self):
        super().__init__('delivery_robot_simple')

        # Parameters
        self.declare_parameter('confirmation_timeout', 30.0)  # seconds
        self.confirmation_timeout = self.get_parameter('confirmation_timeout').get_parameter_value().double_value

        # QoS profiles
        map_qos = QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        self.order_subscription = self.create_subscription(
            String,
            'food_order',
            self.order_callback,
            qos_profile_system_default
        )

        # Publishers
        self.robot_state_publisher = self.create_publisher(String, 'robot_state', 10)

        # Variables
        self.robot_state = RobotState.IDLE
        self.current_orders = []  # List of tables for the current delivery
        self.current_goal_table = None
        self.navigation_goal_handle = None
        self.confirmation_timer = None

        # Define locations (replace with your actual coordinates)
        self.home_position = self.create_pose_stamped(0.0, 0.0, 0.0)
        self.kitchen_position = self.create_pose_stamped(5.0, 2.0, 1.57)
        self.customer_table_positions = {}  # To be populated in main

        self.get_logger().info('Simple Delivery Robot Initialized. Waiting for orders...')
        self.publish_robot_state()

    def publish_robot_state(self):
        msg = String()
        msg.data = str(self.robot_state)
        self.robot_state_publisher.publish(msg)

    def create_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        import tf_transformations
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def order_callback(self, msg):
        order_info = msg.data.strip().lower()
        tables = [table.strip() for table in order_info.split(',')]  # Assuming comma-separated tables
        new_orders = [table for table in tables if table not in self.current_orders]

        if new_orders:
            self.get_logger().info(f'Received new order(s): {", ".join(new_orders)}')
            for table in new_orders:
                if table in self.customer_table_positions:
                    self.current_orders.append(table)
                else:
                    self.get_logger().warn(f'Table {table} location not known.')
            if self.robot_state == RobotState.IDLE and self.current_orders:
                self.start_delivery_sequence()
        else:
            self.get_logger().info('Received order for already queued tables.')

    def start_delivery_sequence(self):
        if self.current_orders:
            self.robot_state = RobotState.GOING_TO_KITCHEN
            self.publish_robot_state()
            self.go_to_kitchen()
        else:
            self.robot_state = RobotState.IDLE
            self.publish_robot_state()
            self.get_logger().info('No orders to process.')

    def go_to_kitchen(self):
        self.get_logger().info('Moving to kitchen...')
        self.navigate_to_goal(self.kitchen_position, self.kitchen_reached_callback)

    def kitchen_reached_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation to kitchen was rejected!')
            self.return_home()
            return

        self.get_logger().info('Navigation to kitchen goal accepted.')
        self.navigation_goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.kitchen_navigation_finished_callback)

    def kitchen_navigation_finished_callback(self, future):
        status = future.result().result.nav_result
        if status == NavigateToPose.RESULT_SUCCESS:
            self.get_logger().info('Reached kitchen. Moving to table...')
            self.process_next_table()
        else:
            self.get_logger().error(f'Navigation to kitchen failed with status: {status}')
            self.return_home()

    def process_next_table(self):
        if self.current_orders:
            next_table = self.current_orders.pop(0)  # Get the first table and remove it
            self.current_goal_table = next_table
            self.robot_state = RobotState.GOING_TO_TABLE
            self.publish_robot_state()
            self.go_to_table(self.customer_table_positions[next_table])
        else:
            self.return_home()

    def go_to_table(self, target_pose):
        self.get_logger().info(f'Moving to {self.current_goal_table}...')
        self.navigate_to_goal(target_pose, self.table_reached_callback)

    def table_reached_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Navigation to {self.current_goal_table} was rejected!')
            self.return_home()
            return

        self.get_logger().info(f'Navigation to {self.current_goal_table} goal accepted.')
        self.navigation_goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.table_navigation_finished_callback)

    def table_navigation_finished_callback(self, future):
        status = future.result().result.nav_result
        if status == NavigateToPose.RESULT_SUCCESS:
            self.get_logger().info(f'Reached {self.current_goal_table}. Delivery done.')
            if self.current_orders:
                self.process_next_table()
            else:
                self.return_home()
        else:
            self.get_logger().error(f'Navigation to {self.current_goal_table} failed with status: {status}')
            self.return_home()

    def return_home(self):
        self.current_goal_table = None
        self.robot_state = RobotState.RETURNING_HOME
        self.publish_robot_state()
        self.get_logger().info('Returning to home position...')
        self.navigate_to_goal(self.home_position, self.home_reached_callback)

    def home_reached_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation to home was rejected!')
            return

        self.get_logger().info('Reached home position. Ready for next order.')
        self.robot_state = RobotState.IDLE
        self.publish_robot_state()

    def navigate_to_goal(self, pose, done_callback):
        self.get_logger().info(f'Sending goal to: {pose.pose.position.x}, {pose.pose.position.y}')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(done_callback)

def main(args=None):
    rclpy.init(args=args)
    delivery_robot = DeliveryRobot()

    # Set up customer table locations (replace with actual coordinates)
    delivery_robot.customer_table_positions = {
        "table1": delivery_robot.create_pose_stamped(8.0, 3.0, 0.0),
        "table2": delivery_robot.create_pose_stamped(8.0, -2.0, 0.0),
        "table3": delivery_robot.create_pose_stamped(10.0, 1.0, 0.0),
        # Add more tables as needed
    }

    rclpy.spin(delivery_robot)
    delivery_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
