import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from math import atan2, sqrt, pi
import transforms3d.euler

class RobotController(Node):
    def __init__(self, robot_id, robot_config):
        super().__init__(f'robot_controller_{robot_id}')

        self.robot_id = robot_id
        self.robot_config = robot_config

        # Publishers and subscribers specific to each robot
        self.cmd_vel_pub = self.create_publisher(Twist, f'/robot{self.robot_id}/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, f'/robot{self.robot_id}/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, f'/robot{self.robot_id}/path', self.path_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_velocity = Twist()
        self.current_odom = None
        self.path = []
        self.current_goal_idx = 0
        self.goal_reached = True  # Initially true to wait for a new goal

    def odom_callback(self, msg):
        self.current_odom = msg
        self.get_logger().info(f'Current Position: ({self.current_odom.pose.pose.position.x}, {self.current_odom.pose.pose.position.y})')

    def path_callback(self, msg):
        # Path received from the topic with waypoints
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.current_goal_idx = 0
        self.goal_reached = False
        self.get_logger().info(f"Received new path with {len(self.path)} points")

    def get_yaw(self):
        if self.current_odom is None:
            return 0.0
        orientation_q = self.current_odom.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        if angle > pi:
            angle -= 2.0 * pi
        elif angle < -pi:
            angle += 2.0 * pi
        return angle

    def control_loop(self):
        if self.current_odom is None or not self.path:
            return

        # Check if we have goals and if the robot hasn't reached the final goal
        if self.current_goal_idx < len(self.path):
            goal_x, goal_y = self.path[self.current_goal_idx]
            current_x = self.current_odom.pose.pose.position.x
            current_y = self.current_odom.pose.pose.position.y

            # Calculate distance and angle to the goal
            distance = sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2) + 0.8
            angle_to_goal = atan2(goal_y - current_y, goal_x - current_x)

            # Check if we are close enough to the current goal
            if distance-0.8 < 0.2:
                self.current_goal_idx += 1
                if self.current_goal_idx >= len(self.path):
                    self.stop_robot()
                    self.goal_reached = True
                    self.get_logger().info("All goals reached!")
                    return

            # Control logic: Proportional control for linear and angular velocities
            angular_velocity = 2.0 * self.normalize_angle(angle_to_goal - self.get_yaw())
            self.current_velocity.angular.z = max(min(angular_velocity, 1.0), -1.0)  # Limit angular velocity to [-1, 1]
            self.current_velocity.linear.x = min(1.0, distance)  # Limit linear velocity to [0, 1] and proportional to distance

            # Publish velocity commands
            self.cmd_vel_pub.publish(self.current_velocity)

            self.get_logger().info(f"Moving towards: x={goal_x}, y={goal_y}, distance: {distance}, angle: {angle_to_goal}")
        else:
            self.get_logger().info(f"Waiting for a new path")

    def stop_robot(self):
        self.current_velocity.linear.x = 0.0
        self.current_velocity.angular.z = 0.0
        self.cmd_vel_pub.publish(self.current_velocity)


def main(args=None):
    rclpy.init(args=args)

    robot_configs = [
        {'robot_id': 0},  # Configuration for robot 0
        {'robot_id': 1},  # Configuration for robot 1
    ]

    nodes = []
    executor = MultiThreadedExecutor()

    for config in robot_configs:
        robot_id = config['robot_id']
        node = RobotController(robot_id, config)
        nodes.append(node)
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()