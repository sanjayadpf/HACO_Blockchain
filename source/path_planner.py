import time
import rclpy
from interface import ContractInterface
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import random
from web3 import Web3
import os
from hexbytes import HexBytes


class HybridACOPathPlanner(Node):
    def __init__(self):
        super().__init__('hybrid_aco_path_planner')

        self.owner_address = '0x9FDa315e1b38960f4E2F0942309E5e0e8F11bcF8'
        self.robot_addresses = ['0xEeEC2c4C17dFED2da9aDfb0EC5Affa2a3C2629f9', '0xBA5afc5932470c0D2A72cFabDD25B05238cAbC4B']

        self.pointcloud_sub = self.create_subscription(PointCloud2, '/merged_pointcloud', self.pointcloud_callback, 10)
        self.imu_sub_robot0 = self.create_subscription(Imu, '/robot0/imu', self.imu_callback_robot0, 10)
        self.imu_sub_robot1 = self.create_subscription(Imu, '/robot1/imu', self.imu_callback_robot1, 10)
        self.path_pub_robot0 = self.create_publisher(Path, '/robot0/path', 10)
        self.path_pub_robot1 = self.create_publisher(Path, '/robot1/path', 10)

        self.grid_map = np.zeros((10, 10), dtype=np.int8)
        self.shared_pheromone_map = None
        self.pheromone_map_robot0 = None
        self.pheromone_map_robot1 = None
        self.ants = 5
        self.iterations = 5
        self.alpha = 1.0
        self.beta = 2.0
        self.evaporation_rate = 0.5
        self.initial_pheromone = 1.0
        self.complex_subspaces = []

        self.web3 = Web3(Web3.HTTPProvider('http://127.0.0.1:7545'))
        self.contract_interface = ContractInterface(self.web3, 'SwarmContract', os.path.abspath('../contracts'))
        self.contract_interface.compile_source_files()
        self.reward = self.web3.to_wei(0.03, 'ether')
        self.collateral = self.web3.to_wei(0.02, 'ether')
        self.sub_task_contracts = []

        self.robot_tasks = {}
        self.current_strategy = "cooperative"

        self.mapping_region = {
            'start': (-5, 0),
            'goal': (5, 10)
        }

        self.merged_map = None

        print("HybridACOPathPlanner initialized")
        self.deploy_contract()

    def deploy_contract(self):
        try:
            ownerAcc = self.web3.eth.accounts[0]
            price = self.reward + self.collateral
            print(f"Deploying contract with reward: {self.reward}, collateral: {self.collateral}, total value: {price}")
            self.contract_address = self.contract_interface.deploy_contract(self.reward, self.collateral, deployment_params={'from': ownerAcc, 'value': price})
            self.contract_interface.get_instance()
            balance = self.web3.eth.get_balance(self.contract_address)
            print(f"Contract deployed at {self.contract_address} with balance: {self.web3.from_wei(balance, 'ether')} ether")
        except Exception as e:
            print(f"Error deploying contract: {e}")
            if hasattr(e, 'args') and len(e.args) > 1:
                print(f"Error details: {e.args[1]}")

    def pointcloud_callback(self, pointcloud):
        print("Merged PointCloud2 received")
        self.merged_map = self.convert_pointcloud2_to_array(pointcloud)
        print(f"Converted Merged PointCloud2 to array with {len(self.merged_map)} points")
        self.update_map()

    def convert_pointcloud2_to_array(self, pointcloud2_msg):
        points = []
        for point in pc2.read_points(pointcloud2_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        return np.array(points)

    def filter_obstacle_points(self, points):
        # Filter out obstacle points based on the z-coordinate
        obstacle_points = points[points[:, 2] > 0.2]  # Adjust the height threshold as needed
        return obstacle_points

    def update_map(self):
        if self.merged_map is not None:
            print("Updating map with new pointclouds")
            obstacle_points = self.filter_obstacle_points(self.merged_map)
            self.grid_map = self.create_occupancy_grid(obstacle_points)
            print(f"Grid map updated with shape: {self.grid_map.shape}")
            self.shared_pheromone_map = self.initialize_pheromone_map(self.grid_map.shape)
            self.pheromone_map_robot0 = self.initialize_pheromone_map(self.grid_map.shape)
            self.pheromone_map_robot1 = self.initialize_pheromone_map(self.grid_map.shape)
            self.allocate_tasks_sequentially()

    def imu_callback_robot0(self, imu):
        self.check_subspace_complexity(imu, 'robot0')

    def imu_callback_robot1(self, imu):
        self.check_subspace_complexity(imu, 'robot1')

    def create_occupancy_grid(self, points):
        print(f"Creating occupancy grid with {len(points)} points")

        min_x, max_x = -5, 5
        min_y, max_y = -5, 5

        width, height = max_x - min_x + 1, max_y - min_y + 1
        grid_map = np.zeros((width, height), dtype=np.int8)
        for point in points:
            x, y = int(point[0]) - min_x, int(point[1]) - min_y
            if 0 <= x < width and 0 <= y < height:
                grid_map[x, y] = 100

        print("Occupancy grid created")
        return grid_map

    def initialize_pheromone_map(self, shape):
        print(f"Initializing pheromone map with shape {shape}")
        return np.ones(shape) * self.initial_pheromone

    def allocate_tasks_sequentially(self):
        if self.grid_map is not None:
            sub_areas = self.divide_area_into_sub_areas(self.grid_map, num_sub_areas=100)
            for i, sub_area in enumerate(sub_areas):
                if self.is_task_completed(sub_area):
                    continue  # Skip already covered sub-areas
                robot_id = i % 2
                self.assign_task_to_robot(sub_area, robot_id)
                # Wait for the task to complete before moving to the next sub-area
                while not self.is_task_completed(sub_area):
                    time.sleep(1)  # Adjust the sleep time as needed
                # Update the map with the completed task
                self.update_map()

    def divide_area_into_sub_areas(self, grid_map, num_sub_areas=100):
        height, width = grid_map.shape
        sub_height = height // 10
        sub_width = width // 10
        sub_areas = []

        for i in range(10):
            for j in range(10):
                start_x = i * sub_height
                end_x = start_x + sub_height - 1
                start_y = j * sub_width
                end_y = start_y + sub_width - 1
                sub_areas.append({'start': (start_x, start_y), 'goal': (end_x, end_y)})

        return sub_areas

    def assign_task_to_robot(self, sub_area, robot_id):
        path = self.plan_path_for_sub_area(sub_area, robot_id)
        if robot_id == 0:
            self.publish_path(path, self.path_pub_robot0)
        elif robot_id == 1:
            self.publish_path(path, self.path_pub_robot1)
        contract_address = self.deploy_sub_task_contract()
        self.sub_task_contracts.append(contract_address)
        robot_address = self.robot_addresses[robot_id]
        self.assign_task_to_robot_contract(contract_address, robot_address)
        self.robot_tasks[robot_id] = {
            'path': path,
            'sub_area': sub_area,
            'contract_address': contract_address
        }

    def plan_path_for_sub_area(self, sub_area, robot_id):
        start, goal = sub_area['start'], sub_area['goal']
        path = self.aco(start, goal, cooperative=(self.current_strategy == "cooperative"), robot_id=robot_id)
        print(f"Robot {robot_id} path: {path}")  # Debug print for path
        return path

    def is_task_completed(self, sub_area):
        start, goal = sub_area['start'], sub_area['goal']
        sub_map = self.grid_map[start[0]:goal[0]+1, start[1]:goal[1]+1]
        return np.all(sub_map == 100)

    def check_subspace_complexity(self, imu_data, robot_id):
        # Extract linear acceleration and angular velocity from the IMU message
        accel = np.array([imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z])
        angular_vel = np.array([imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z])
        
        # Calculate Root Mean Square (RMS) for acceleration and angular velocity
        accel_rms = np.sqrt(np.mean(accel**2))
        angular_vel_rms = np.sqrt(np.mean(angular_vel**2))
        
        # Calculate variance for acceleration and angular velocity
        accel_variance = np.var(accel)
        angular_vel_variance = np.var(angular_vel)
        
        # Define complexity thresholds
        accel_rms_threshold = 1.5  # threshold, adjust based on experimentation
        angular_vel_rms_threshold = 0.5  # threshold for angular velocity
        accel_variance_threshold = 0.8  # variance threshold
        angular_vel_variance_threshold = 0.3  # variance threshold

        # Determine complexity based on thresholds
        is_complex = False
        if (accel_rms > accel_rms_threshold or
            angular_vel_rms > angular_vel_rms_threshold or
            accel_variance > accel_variance_threshold or
            angular_vel_variance > angular_vel_variance_threshold):
            
            is_complex = True

        # Print debug information (optional)
        print(f"Robot {robot_id} IMU analysis - Accel RMS: {accel_rms}, Angular RMS: {angular_vel_rms}")
        print(f"Robot {robot_id} IMU analysis - Accel Variance: {accel_variance}, Angular Variance: {angular_vel_variance}")
        print(f"Robot {robot_id} complexity: {is_complex}")
        
        # Return the complexity result
        return is_complex

    def aco(self, start, goal, cooperative=True, robot_id=None):
        best_path = None
        best_length = float('inf')
        for _ in range(self.iterations):
            all_paths = []
            for _ in range(self.ants):
                if cooperative:
                    path, length = self.construct_solution(start, goal, self.shared_pheromone_map)
                else:
                    if robot_id == 0:
                        path, length = self.construct_solution(start, goal, self.pheromone_map_robot0)
                    else:
                        path, length = self.construct_solution(start, goal, self.pheromone_map_robot1)
                all_paths.append((path, length))
                if length < best_length:
                    best_path = path
                    best_length = length
            if cooperative:
                self.update_pheromones(all_paths, self.shared_pheromone_map)
            else:
                if robot_id == 0:
                    self.update_pheromones(all_paths, self.pheromone_map_robot0)
                else:
                    self.update_pheromones(all_paths, self.pheromone_map_robot1)
        return best_path

    def construct_solution(self, start, goal, pheromone_map):
        path = [start]
        current = start
        path_length = 0
        while current != goal:
            neighbors = self.get_neighbors(current)
            neighbors = [n for n in neighbors if self.grid_map[n[0], n[1]] != 100]  # Exclude obstacles
            if not neighbors:
                return path, float('inf')
            probabilities = self.calculate_probabilities(current, neighbors, pheromone_map)
            if np.isnan(probabilities).any() or np.isinf(probabilities).any():
                return path, float('inf')
            next_node = self.choose_next_node(neighbors, probabilities)
            path.append(next_node)
            path_length += 1  # Increment path length by 1
            current = next_node
        return path, path_length

    def calculate_probabilities(self, current, neighbors, pheromone_map):
        pheromones = np.array([pheromone_map[n[0], n[1]] for n in neighbors])
        heuristics = np.array([1.0 / (self.grid_map[n[0], n[1]] + 1) for n in neighbors])
        numerators = (pheromones ** self.alpha) * (heuristics ** self.beta)
        denominator = np.sum(numerators)
        if denominator == 0:
            return np.ones_like(numerators) / len(numerators)
        return numerators / denominator

    def choose_next_node(self, neighbors, probabilities):
        return random.choices(neighbors, probabilities)[0]

    def update_pheromones(self, all_paths, pheromone_map):
        pheromone_map *= (1 - self.evaporation_rate)
        for path, length in all_paths:
            if length == 0:
                continue
            for node in path:
                pheromone_map[node[0], node[1]] += 1.0 / length

    def get_neighbors(self, node):
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        result = []
        for dx, dy in neighbors:
            x2, y2 = node[0] + dx, node[1] + dy
            if 0 <= x2 < self.grid_map.shape[0] and 0 <= y2 < self.grid_map.shape[1]:
                result.append((x2, y2))
        return result

    def publish_path(self, path, publisher):
        ros_path = Path()
        ros_path.header.frame_id = 'map'
        for pose in path:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = float(pose[0])
            pose_stamped.pose.position.y = float(pose[1])
            pose_stamped.pose.position.z = 0.0  # Assuming a 2D path, set z to 0.0
            ros_path.poses.append(pose_stamped)
        publisher.publish(ros_path)
        print(f"Published path: {ros_path}")  # Debug print for path publication

    def assign_task_to_robot_contract(self, contract_address, robot_address):
        try:
            # Ensure the addresses are properly formatted as checksum addresses
            robot_address = self.web3.to_checksum_address(robot_address)
            contract_address = self.web3.to_checksum_address(contract_address)

            # Debugging: Print the addresses and their types
            print(f"Robot address: {robot_address}, type: {type(robot_address)}")
            print(f"Contract address: {contract_address}, type: {type(contract_address)}")

            # Ensure the collateral value is correctly formatted
            collateral_wei = self.collateral if isinstance(self.collateral, int) else self.web3.toWei(self.collateral, 'ether')

            # Debugging: Print the collateral value and its type
            print(f"Collateral (wei): {collateral_wei}, type: {type(collateral_wei)}")

            # Debugging: Print the transaction parameters before sending
            tx_params = {'from': robot_address, 'value': collateral_wei, 'to': contract_address}
            print(f"Transaction parameters: {tx_params}")

            # Send the transaction with the function name and transaction parameters
            self.contract_interface.send('acceptTask', tx_params={'from': robot_address, 'value': collateral_wei, 'to': contract_address})

            # Debugging: Print the transaction hash
            #print(f"Transaction hash: {tx_hash}")

            # Wait for the transaction receipt
            #receipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)
            #print("Transaction receipt received.")

        except Exception as e:
            print(f"Error assigning task to robot contract: {e}")
            if hasattr(e, 'args') and len(e.args) > 1:
                print(f"Error details: {e.args[1]}")

    def deploy_sub_task_contract(self):
        ownerAcc = self.web3.eth.accounts[0]
        price = self.reward + self.collateral
        contract_address = self.contract_interface.deploy_contract(self.reward, self.collateral, deployment_params={'from': ownerAcc, 'value': price})
        self.contract_interface.get_instance()
        return contract_address

    def all_tasks_completed(self):
        for robot_id, task in self.robot_tasks.items():
            if not self.is_task_completed(task['sub_area']):
                return False
        return True

def main(args=None):
    rclpy.init(args=args)
    planner = HybridACOPathPlanner()
    rclpy.spin(planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
