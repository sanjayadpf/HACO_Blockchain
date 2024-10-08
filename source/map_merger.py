import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')
        self.pointcloud_sub_robot0 = self.create_subscription(PointCloud2, '/robot0/point_cloud2', self.pointcloud_callback_robot0, 10)
        self.pointcloud_sub_robot1 = self.create_subscription(PointCloud2, '/robot1/point_cloud2', self.pointcloud_callback_robot1, 10)
        self.merged_pointcloud_pub = self.create_publisher(PointCloud2, '/merged_pointcloud', 10)
        self.pointcloud_robot0 = None
        self.pointcloud_robot1 = None
        self.accumulated_points = np.empty((0, 3), dtype=np.float32)

        # Define the region of interest (e.g., a square region)
        self.min_x, self.max_x = -4.5, 4.5
        self.min_y, self.max_y =0.0, 10.0

    def pointcloud_callback_robot0(self, msg):
        self.pointcloud_robot0 = msg
        self.merge_pointclouds()

    def pointcloud_callback_robot1(self, msg):
        self.pointcloud_robot1 = msg
        self.merge_pointclouds()

    def filter_points_in_region(self, points):
        """Filter points that are within the defined rectangular region."""
        x_in_region = (points[:, 0] >= self.min_x) & (points[:, 0] <= self.max_x)
        y_in_region = (points[:, 1] >= self.min_y) & (points[:, 1] <= self.max_y)
        return points[x_in_region & y_in_region]

    def merge_pointclouds(self):
        if self.pointcloud_robot0 and self.pointcloud_robot1:
            points_robot0 = np.array(list(pc2.read_points(self.pointcloud_robot0, field_names=("x", "y", "z"), skip_nans=True)))
            points_robot1 = np.array(list(pc2.read_points(self.pointcloud_robot1, field_names=("x", "y", "z"), skip_nans=True)))

            # Extract x, y, z fields and convert to float32
            points_robot0 = np.vstack([points_robot0['x'], points_robot0['y'], points_robot0['z']]).T.astype(np.float32)
            points_robot1 = np.vstack([points_robot1['x'], points_robot1['y'], points_robot1['z']]).T.astype(np.float32)

            # Filter points to only include those within the defined region
            points_robot0 = self.filter_points_in_region(points_robot0)
            points_robot1 = self.filter_points_in_region(points_robot1)

            # Debugging information
            print(f'Robot 0 Filtered PointCloud data:\n{points_robot0}')
            print(f'Robot 1 Filtered PointCloud data:\n{points_robot1}')
            print(f'Robot 0 Filtered PointCloud shape: {points_robot0.shape}')
            print(f'Robot 1 Filtered PointCloud shape: {points_robot1.shape}')

            self.accumulated_points = np.vstack((self.accumulated_points, points_robot0, points_robot1))
            merged_pointcloud = pc2.create_cloud_xyz32(self.pointcloud_robot0.header, self.accumulated_points)
            self.merged_pointcloud_pub.publish(merged_pointcloud)

def main(args=None):
    rclpy.init(args=args)
    map_merger = MapMerger()
    rclpy.spin(map_merger)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
