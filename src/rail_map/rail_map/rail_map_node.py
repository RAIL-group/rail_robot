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

# Right now this work, changing Origin to (-100, -100) won't work
GRID_SIZE = (2000, 2000)
ORIGIN = (-50, -50)
RESOLUTION = 0.05

class RailMapPublisher(Node):
    def __init__(self):
        super().__init__('rail_map_publisher')
        self.robot_grid = None
        self.pose_ = None
        self.simulator = self.setup_simulator()
        self.init_map = False
        self.static_map = None
        self.final_origin = None
        self.resolution = RESOLUTION

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

    def get_transformed_map_and_pose(self, final_map, current_map, final_origin, current_origin, resolution, pose=None):

        # Compute the offset between the two origins
        dx = int((current_origin[0] - final_origin[0]) / resolution)
        dy = int((current_origin[1] - final_origin[1]) / resolution)

        # Initialize the transformed map with UNOBSERVED_VAL
        transformed_map = np.full_like(final_map, UNOBSERVED_VAL)

        # shift_x and shift_y are swapped since world frame x-axis is grid axis 1 and y-axis is grid-axis 0
        y_end = dy + current_map.shape[0]
        x_end = dx + current_map.shape[1]
        transformed_map[dy:y_end, dx:x_end] = current_map

        if pose is None:
            return transformed_map, None

        transformed_pose = (pose[0] + dy, pose[1] + dx)

        return transformed_map, transformed_pose

    def map_callback(self, msg):
        self.msg = msg
        grid_shape = (msg.info.height, msg.info.width)
        occupancy_grid = np.array(msg.data).reshape(grid_shape)
        current_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        resolution = msg.info.resolution

        # Initialize the map.
        if not self.init_map:
            self.static_map = np.full(GRID_SIZE, UNOBSERVED_VAL)
            self.final_origin = ORIGIN
            self.resolution = resolution
            self.init_map = True

        if self.pose_ is not None:
            # Transform the current occupancy grid to the static map frame
            transformed_grid, transformed_pose = self.get_transformed_map_and_pose(
                self.static_map,
                occupancy_grid,
                self.final_origin,
                current_origin,
                resolution,
                self.world_frame_to_map_frame(self.pose_, current_origin, resolution)
            )

            if self.robot_grid is None:
                self.robot_grid = np.full(self.static_map.shape, UNOBSERVED_VAL)

            # Update grid and publish.
            self.simulator.set_known_map(transformed_grid)
            robot_pose = Pose(*transformed_pose, self.pose_[2])
            _, updated_grid = self.simulator.get_laser_scan_and_update_map(robot_pose, self.robot_grid)
            self.robot_grid = updated_grid

            self.publish_robot_grid(self.robot_grid, self.final_origin, self.resolution)

    def publish_robot_grid(self, robot_grid, origin, resolution):
        """Converts the robot grid to an OccupancyGrid message and publishes it."""
        grid_msg = OccupancyGrid()
        grid_msg.header = self.msg.header
        grid_msg.info.width = robot_grid.shape[1]
        grid_msg.info.height = robot_grid.shape[0]

        grid_msg.info.origin.position.x = float(origin[0])
        grid_msg.info.origin.position.y = float(origin[1])
        grid_msg.info.origin.position.z = float(self.msg.info.origin.position.z)

        grid_msg.info.origin.orientation = self.msg.info.origin.orientation

        grid_msg.info.resolution = float(resolution)
        grid_msg.data = robot_grid.astype(int).flatten().tolist()

        self.map_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RailMapPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
