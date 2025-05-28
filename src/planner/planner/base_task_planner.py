import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import yaml
import math
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import message_filters
from nav_msgs.msg import OccupancyGrid, Path
from scipy.optimize import linear_sum_assignment
import random
import numpy as np
from . import utils, plotting
from . import object_detection
import matplotlib.pyplot as plt
import mr_task
from cv_bridge import CvBridge
import matplotlib
import os
from PIL import Image as PILImage
from scipy.spatial.transform import Rotation as R


ALL_OBJECTS = [
    'bottle',
    'cell phone',
    'laptop',
    'credit card',
    'pan',
    'spoon',
    'grocery',
]

matplotlib.use('Agg')

get_revealed_objects = object_detection.get_revealed_objects
# get_revealed_objects = object_detection.get_revealed_objects_FAKE
PLANNER = mr_task.planner.LearnedMRTaskPlanner
# PLANNER = mr_task.planner.OptimisticMRTaskPlanner


class BaseTaskPlannerNode(PLANNER, Node):
    """Abstract Planner class"""
    def __init__(self, args, specification, name=None):
        name = name if name is not None else 'base_task_planner'
        Node.__init__(self, name)
        PLANNER.__init__(self, args, specification)

        # Parameters
        self.declare_parameter('all_robot_names', 'robot')
        self.declare_parameter(
            'container_info_file', '/home/abhish/ros2/rail_robot/src/rail_robot/worlds/floor_map_containers.yaml')

        self.all_robot_names = self.get_parameter(
            'all_robot_names').get_parameter_value().string_value.split(',')
        yaml_file = self.get_parameter(
            'container_info_file').get_parameter_value().string_value
        if yaml_file is None:
            raise ValueError("container_info_file cannot be None.")

        self.robot_poses_dict = {robot: None for robot in self.all_robot_names}
        self.robot_poses = [None for _ in self.all_robot_names]
        # self.robot_images = [None for _ in self.all_robot_names]

        # Get the graph from containers
        self.graph = utils.get_scene_graph_from_yaml(yaml_file)
        # plotting.plot_graph(self.graph)
        # plt.savefig('graph.png')
        # Subscribers
        self.pose_subscribers = {
            robot: message_filters.Subscriber(
                self, PoseStamped, f'/{robot}/current_pose')
            for robot in self.all_robot_names}
        self.bridge = CvBridge()

        # static_map_qos_profile = QoSProfile(
        #     depth=1,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        # )

        # self.occupancy_map_subscriber = self.create_subscription(
        #     OccupancyGrid,
        #     f'/{self.all_robot_names[0]}/map',
        #     self.occupancy_map_callback,
        #     static_map_qos_profile
        # )

        pose_ts = message_filters.ApproximateTimeSynchronizer(
            self.pose_subscribers.values(), 10, slop=4)
        pose_ts.registerCallback(self.set_poses)
        print('Waiting for all robot poses')

        self.navigators = [BasicNavigator(
            namespace=robot) for robot in self.all_robot_names]
        print("Waiting until nav2 is active for all robots")
        for nav in self.navigators:
            nav.lifecycleStartup()

        self.revealed_container_idxs = {}
        self.unexplored_container_nodes = [mr_task.core.Node(is_subgoal=True,
                                                             name=idx,
                                                             location=self.graph.get_node_position_by_idx(idx))
                                           for idx in self.graph.container_indices]
        print(self.unexplored_container_nodes)
        self.container_distances = self.compute_subgoal_distances(self.unexplored_container_nodes)
        self.is_task_complete = [False for _ in self.all_robot_names]

        self.explored_container_nodes = []
        observations = {'observed_graph': self.graph,
                        'observed_map': None}
        self.update(observations, self.robot_poses, self.explored_container_nodes,
                    self.unexplored_container_nodes, objects_found=())
        for node in self.unexplored_container_nodes:
            for obj in self.objects_to_find:
                PS, _, _ = self.node_prop_dict[(node, obj)]
                print(f'{(self.graph.get_node_name_by_idx(node.name), obj)}: {PS:.2f}')
        # exit()

        # plotting.plot_graph(self.graph)
        # plt.show()
        # print(self.robot_poses)
        # exit()

    def set_poses(self, *args):
        """Callback to process robot's current position and publish nearest goal."""
        for i, msg in enumerate(args):
            robot = self.all_robot_names[i]
            self.robot_poses_dict[robot] = msg
        for i, robot in enumerate(self.all_robot_names):
            pose = self.robot_poses_dict[robot]
            self.robot_poses[i] = pose.pose.position
        # if any(self.is_task_complete):
        self.plan_and_navigate()

    def compute_subgoal_distances(self, nodes):
        distances = {}

        # Compute subgoal-to-subgoal distances
        for node1 in nodes:
            for node2 in nodes:
                if node1 == node2:
                    distances[(node1, node2)] = 0.0
                    continue

                start_pose = self.create_pose_stamped(node1.location)
                goal_pose = self.create_pose_stamped(node2.location)
                path = self.navigators[0].getPath(start=start_pose, goal=goal_pose)
                distances[(node1, node2)] = self.get_path_length(path)

        # print(distances)
        return distances

    def get_inter_distances_nodes(self, nodes, robot_nodes, observed_map=None):
        distances = self.container_distances.copy()

        # Compute robot-to-subgoal distances
        for robot_idx, robot in enumerate(robot_nodes):
            navigator = self.navigators[robot_idx]
            robot_pose = self.create_pose_stamped(robot.start.location)

            for node in nodes:
                goal_pose = self.create_pose_stamped(node.location)
                path = navigator.getPath(start=robot_pose, goal=goal_pose)
                distances[(robot.start, node)] = self.get_path_length(path)
        # print(distances)
        return distances

    def create_pose_stamped(self, location):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(location[0])
        pose.pose.position.y = float(location[1])
        # print(location)
        quaternion = utils.euler_to_quaternion([0, 0, math.radians(location[2])])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def get_path_length(self, path):
        total_length = 0.0
        poses = path.poses

        for i in range(1, len(poses)):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            total_length += math.dist([p0.x, p0.y], [p1.x, p1.y])

        return total_length

    def stop_all_robots(self):
        print("Stopping all robots")
        for nav in self.navigators:
            nav.cancelTask()

    def plan_and_navigate(self):
        if self.dfa_planner.has_reached_accepting_state():
            print(f"Task {self.specification} complete!")
            self.stop_all_robots()
            exit()

        if len(self.unexplored_container_nodes) == 0:
            print("All containers explored.")
            for i, robot in enumerate(self.all_robot_names):
                self.is_task_complete[i] = True
            self.stop_all_robots()
            exit()
        # print('plan and navigate')
        # robot_subgoal_distances = self.get_robot_subgoal_distances()
        # robot_nodes = [mr_task.core.RobotNode(Node(location=(r_pose[0], r_pose[1]))) for r_pose in self.robot_poses]
        distances_fn = self.get_inter_distances_nodes

        joint_action, _ = self.compute_joint_action(distances_fn=distances_fn)
        # if joint_action is None:
        #     print(f"Task {self.specification} complete!")
        #     return

        goal_poses = {}
        for robot_idx, action in enumerate(joint_action):
            goal_pose = self.create_pose_stamped(action.target_node.location)
            goal_poses[robot_idx] = goal_pose, action.target_node.name

        for robot_idx, _ in enumerate(self.all_robot_names):
            nav = self.navigators[robot_idx]
            goal_pose, container_idx = goal_poses[robot_idx]
            nav.goToPose(goal_pose)
            print(f"Robot {robot_idx + 1} is navigating to {self.graph.get_node_name_by_idx(container_idx)}")

        while not any(self.is_task_complete):
            self.is_task_complete = [nav.isTaskComplete() for nav in self.navigators]
        print(self.is_task_complete)
        self.stop_all_robots()


        completed_robot_idx = self.is_task_complete.index(True)
        # for robot_ix, is_complete in enumerate(self.is_task_complete):
        #     if is_complete:
        #         completed_robot_idx = robot_idx

        reached_container_idx = joint_action[completed_robot_idx].target_node.name
        reached_container_name = self.graph.get_node_name_by_idx(reached_container_idx)
        result = self.navigators[completed_robot_idx].getResult()
        if result == TaskResult.FAILED:
            print(f'{completed_robot_idx} failed to reach container {reached_container_name}')
        if result == TaskResult.SUCCEEDED:
            print(f'Robot {completed_robot_idx + 1} reached container {reached_container_name}')
            camera_image = self.get_robot_image(completed_robot_idx)
            objects_found = get_revealed_objects(camera_image, ALL_OBJECTS, reached_container_name)
            print(f"Objects found at {reached_container_name}: {objects_found}")
            self.add_objects_to_graph(reached_container_idx, objects_found)
            self.revealed_container_idxs[reached_container_idx] = objects_found
            self.explored_container_nodes = [mr_task.core.Node(name=idx, props=objects,
                                                               location=self.graph.get_node_position_by_idx(idx))
                                             for idx, objects in self.revealed_container_idxs.items()]
            self.unexplored_container_nodes = self.get_unexplored_containers(reached_container_idx)
            observations = {'observed_graph': self.graph,
                            'observed_map': None}
            self.update(observations, self.robot_poses, self.explored_container_nodes,
                        self.unexplored_container_nodes, objects_found)

        self.is_task_complete[completed_robot_idx] = False

    def get_unexplored_containers(self, explored_container_idx):
        return [node for node in self.unexplored_container_nodes if node.name != explored_container_idx]

    def add_objects_to_graph(self, container_idx, objects):
        for name in objects:
            position = self.graph.get_node_position_by_idx(container_idx)
            obj_idx = self.graph.add_node({
                'id': f"{name}|0",
                'name': name,
                'position': position,
                'type': [0, 0, 0, 1]  # Object
            })
            self.graph.add_edge(container_idx, obj_idx)

    def occupancy_map_callback(self, msg):
        # Extract map metadata
        self.known_map_width = msg.info.width
        self.known_map_height = msg.info.height
        self.known_map_resolution = msg.info.resolution
        self.known_map_origin = msg.info.origin  # geometry_msgs/Pose

        # Convert 1D list to 2D numpy array and reshape
        data = np.array(msg.data, dtype=np.int8).reshape((self.known_map_height, self.known_map_width))
        # data[data==100] = 1
        # print(np.unique(data))
        # self.graph.ensure_connectivity(data.T)
        # Mask unknowns (-1) for better visualization

        # Set up figure
        plt.figure()
        extent = [
            self.known_map_origin.position.x,
            self.known_map_origin.position.x + self.known_map_width * self.known_map_resolution,
            self.known_map_origin.position.y,
            self.known_map_origin.position.y + self.known_map_height * self.known_map_resolution,
        ]

        # Show map
        plt.imshow(data, cmap='gray_r', origin='lower', extent=extent)
        plotting.plot_graph(self.graph)
        plt.title("Occupancy Grid Map")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.savefig('occupancy_map.png', dpi=600)
        exit()

    def get_robot_image(self, robot_idx, path='~/mr_task_data'):
        print(f"Getting image from robot {robot_idx}")
        path = os.path.expanduser(path)
        image_path = os.path.join(path, f"{self.all_robot_names[robot_idx]}_image.png")
        image = PILImage.open(image_path)
        return image


def find_action_list_from_cost_matrix_using_lsa(cost_matrix, subgoal_matrix):
    cost = cost_matrix
    num_robots = len(cost_matrix)
    left_robot = num_robots
    assigned_robot = 0
    joint_action = [None for i in range(num_robots)]
    count = 0
    while (left_robot != 0 and count < num_robots + 1):
        # find the lowest cost for the first 'k' robots, where k is the number of subgoals
        n_rows, n_cols = linear_sum_assignment(cost)
        for i, row in enumerate(n_rows):
            # assign the action to the robot if it is not previously assigned, i.e., not None
            if joint_action[row] is None:
                joint_action[row] = subgoal_matrix[row][n_cols[i]]
                assigned_robot += 1
                # replace the cost by a 'high number' so that it it doesn't get selected when doing lsa
                cost[row] = 1e11
            # decrement the left robot so that it loops and assigns to the remaining robot.
        left_robot = num_robots - assigned_robot
        count += 1
    # for every none items in the joint action, randomly assign a subgoal in the joint action that's not none
    if None in joint_action:
        non_none_items = [item for item in joint_action if item is not None]
        none_idx = [idx for idx, val in enumerate(joint_action) if val is None]
        for idx in none_idx:
            joint_action[idx] = random.choice(non_none_items)
    return joint_action


def load_points_from_yaml(yaml_file):
    """Loads container info from YAML file."""
    points = {}
    with open(yaml_file, 'r') as file:
        points_data = yaml.safe_load(file)

    for point_name, point_data in points_data.items():
        points[point_name] = (
            point_data['x'], point_data['y'], point_data['z'])

    return points


def main(args=None):
    rclpy.init(args=args)
    planner_args = lambda: None  # noqa
    planner_args.network_file = '/home/ab/lsp/data/mr_task/raihan_nn/fcnn.pt'
    planner_args.C = 10
    planner_args.num_iterations = 50000

    planner_node = BaseTaskPlannerNode(args=planner_args,
                                       specification='F bottle & F toiletpaper')
                                    #    specification='F plate & F fork & F plant & F book & F keys & F laptop & F creditcard & F cellphone & F bottle')
    rclpy.spin(planner_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
