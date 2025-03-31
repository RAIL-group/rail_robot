import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

from .constants import UNOBSERVED_VAL
import tf_transformations as tf_trans
from .simulators import Simulator
from .common import Pose
import matplotlib.pyplot as plt

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


qos = QoSProfile(
    history = QoSHistoryPolicy.KEEP_LAST,
    depth = 10,
    reliability = QoSReliabilityPolicy.RELIABLE,
    durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
)

GRID_SIZE = (2000, 2000)
ORIGIN = (-100, -100)
RESOLUTION = 0.09

class RailMapPublisher(Node):
    def __init__(self):
        super().__init__('rail_map_publisher')
        self.robot_grid = None
        self.pose_ = None
        self.simulator = self.setup_simulator()

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/current_pose',
            self.pose_callback,
            10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/robot/map',
            self.map_callback,
            10
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/robot/rail_map',
            qos
        )

    def setup_simulator(self):
        args = lambda: None
        args.base_resolution = RESOLUTION
        args.inflation_radius_m = 0.09
        args.laser_max_range_m = 12
        args.disable_known_grid_correction = False
        args.field_of_view_deg = 360
        args.laser_scanner_num_points = 1024
        simulator = Simulator(args)
        return simulator

    def get_pose_array(self, translation, rotation):
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        euler = tf_trans.euler_from_quaternion(quaternion)
        pose = [translation.x, translation.y, euler[2]]  # [x, y, yaw]
        return pose

    def world_frame_to_map_frame(self, pose, origin, resolution):
        x = (pose[0] - origin[0]) / resolution
        y = (pose[1] - origin[1]) / resolution
        return y, x

    def pose_callback(self, msg):
        pose = self.get_pose_array(msg.pose.position, msg.pose.orientation)
        pose[2] = np.pi - pose[2]  # Correction for lidar pose
        self.pose_ = pose

    def map_callback(self, msg):
        self.msg = msg
        grid_shape = (msg.info.height, msg.info.width)
        occupancy_grid = np.array(msg.data).reshape(grid_shape)
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        resolution = msg.info.resolution
        if self.robot_grid is None:
            self.robot_grid = np.full(grid_shape, UNOBSERVED_VAL)

        if self.pose_ is not None:
            pose = self.world_frame_to_map_frame(self.pose_, origin, resolution)
            self.robot_grid = self.update_robot_grid(occupancy_grid, self.robot_grid, pose)
            self.publish_robot_grid(self.robot_grid, self.msg.info.origin, resolution)

    def update_robot_grid(self, known_map, robot_grid, pose):
        self.simulator.set_known_map(known_map)
        robot_pose = Pose(*pose)
        _, updated_grid = self.simulator.get_laser_scan_and_update_map(robot_pose, robot_grid)
        return updated_grid

    def publish_robot_grid(self, robot_grid, origin, resolution):
        """Converts the robot grid to an OccupancyGrid message and publishes it."""
        grid_msg = OccupancyGrid()
        grid_msg.header = self.msg.header
        grid_msg.info.width = robot_grid.shape[1]
        grid_msg.info.height = robot_grid.shape[0]
        grid_msg.info.origin = origin
        grid_msg.info.resolution = resolution
        grid_msg.data = robot_grid.astype(int).flatten().tolist()

        self.map_pub.publish(grid_msg)
        self.get_logger().info('Published Rail Map')

def main(args=None):
    rclpy.init(args=args)
    node = RailMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
