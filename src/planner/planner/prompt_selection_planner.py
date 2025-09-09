import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import math
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
# import message_filters
from scipy.optimize import linear_sum_assignment
import random
import numpy as np
from pathlib import Path
from . import utils, plotting
import matplotlib.pyplot as plt
import mr_task
from cv_bridge import CvBridge
import matplotlib
import os
from PIL import Image as PILImage
import time
import lsp
from object_detection.srv import ObjectDetectionSrv
from object_search_select.policy_selection import get_lb_selection, get_ucb_selection, compute_lbcost_wavg

from object_search.planners import (
    OptimisticPlanner,
    LSPLLMGPTPlanner,
    LSPLLMGeminiPlanner,
    FullLLMGPTPlanner,
    FullLLMGeminiPlanner,
)
from object_search_select.planners import PolicySelectionPlanner

NUM_FRONTIERS_MAX = 8
EXPLORATION_C = 100 * 0.05
OBJECTS_CONTAINED = {
    "drawer": ['knife', 'bowl'],
    "countertop": ['plate', 'fork', 'spoon'],
    "oven": [],
    "chair": ['bag'],
    "sofa": ['wallet'],
    "couch": ['blanket'],
    "table": ['cellphone', 'keys', 'remotecontrol'],
    "bed": ['laptop', 'pillow'],
    "sidetable": ['keys']
}

TARGET_OBJ_BY_SEED = {
    0: 'laptop',  # ok
    1: 'cellphone',  # ok
    2: 'remotecontrol',  # ok
    3: 'pillow',
    4: 'fork',
    5: 'knife',
    6: 'spoon',
    7: 'bowl',
    8: 'bag',
    9: 'keys',
    10: 'blanket',
}

matplotlib.use('Agg')


class PromptSelectionPlannerNode(PolicySelectionPlanner, Node):
    """Abstract Planner class"""
    def __init__(self, target_obj_info, planners, chosen_planner_idx, args):
        name = 'prompt_selection_planner'
        Node.__init__(self, name)
        PolicySelectionPlanner.__init__(self, target_obj_info, planners, chosen_planner_idx, args)

        # Parameters
        self.declare_parameter('robot_name', 'robot')
        self.declare_parameter(
            'container_info_file', '/home/ab/projects/rail_robot/src/rail_robot/worlds/roberts_road_containers.yaml')

        self.robot_name = self.get_parameter(
            'robot_name').get_parameter_value().string_value
        yaml_file = self.get_parameter(
            'container_info_file').get_parameter_value().string_value
        if yaml_file is None:
            raise ValueError("container_info_file cannot be None.")

        # self.robot_images = [None for _ in self.all_robot_names]

        # Get the graph from containers
        self.graph = utils.get_scene_graph_from_yaml(yaml_file)
        self.grid = None
        self.subgoals = None
        self.initial_robot_pose = None
        self.robot_pose = None
        # self.robot_pose_msg = None
        self.is_task_complete = False
        self.target_object_container_idx = None
        self.net_motion = 0.0
        self.chosen_container_idxs = []

        self.container_distances = None
        self.room_distances = None

        for planner in self.planners:
            planner.get_robot_distances = self.get_robot_distances
            planner.get_subgoal_distances = self.get_subgoal_distances

        self.save_dir = Path(self.args.save_dir).expanduser()
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # self.pose_subscriber = self.create_subscription(
        #     PoseStamped, f'/{self.robot_name}/current_pose', self.set_pose, 10)
        self.pose_subscriber = self.create_subscription(
            PoseStamped, f'/{self.robot_name}/current_pose', self.set_pose_debug, 10)
        # print("Waiting for object detection service...")
        # self.object_detector = self.create_client(ObjectDetectionSrv, '/groundingdino_object_detection')
        # self.object_detector.wait_for_service()
        self.bridge = CvBridge()
        self.destroy = False

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

        self.navigator = BasicNavigator(namespace=self.robot_name)
        print("Waiting until nav2 is active")
        self.navigator.lifecycleStartup()
        # self.navigator = FakeNavigator(self)

        # plotting.plot_graph(self.graph)
        # plt.show()
        # print(self.robot_poses)
        # exit()

    def set_pose(self, msg):
        """Callback to process robot's current position."""
        if self.robot_pose is None:
            self.initial_robot_pose = msg.pose.position

        self.robot_pose = msg.pose.position
        if self.is_task_complete:
            self.save_trial_data()
            exit()

        self.plan_and_navigate()

    def set_pose_debug(self, msg, ):
        """Callback to process robot's current position debug."""
        # self.robot_pose_msg = msg
        if self.robot_pose is None:
            self.initial_robot_pose = msg.pose.position
            self.robot_pose = msg.pose.position
            return
        else:
            # self.robot_pose = msg.pose.position
            if self.subgoals is None:
                subgoals = self.graph.container_indices
                self.update(self.graph, self.grid, subgoals, self.robot_pose)
            if self.container_distances is None:
                self.container_distances = self.get_subgoal_distances(self.grid, self.subgoals)
            target_container = invert_container_mapping(OBJECTS_CONTAINED)[self.target_obj_info['name']]
            self.target_object_container_idx = self.graph.get_node_indices_by_name(target_container)[0]
            net_motion, _ = self.get_lowerbound_planner_costs(self)
            self.net_motion = net_motion
            print(f'Target object {self.target_obj_info["name"]} found at {target_container}')
            print(f'Cost: {net_motion}')
            self.save_trial_data()
            # print('Exiting...')
            self.destroy = True
            return

    def get_subgoal_distances(self, grid, subgoals):
        # if len(subgoals) <= 1:
        #     return None    def compute_selected_subgoal

        distances = {}

        # Compute subgoal-to-subgoal distances
        for s1 in subgoals:
            for s2 in subgoals:
                if s1 == s2:
                    continue
                if frozenset([s1, s2]) in distances:
                    continue
                # print(f"Computing distance between {s1.pose} and {s2.pose}")
                start_pose = self.create_pose_stamped(s1.pose)
                goal_pose = self.create_pose_stamped(s2.pose)
                path = self.navigator.getPath(start=start_pose, goal=goal_pose)
                distances[frozenset([s1, s2])] = self.get_path_length(path)

        return distances

    def get_robot_distances(self, grid, robot_pose, subgoals):
        distances = {}
        robot_pose = self.create_pose_stamped([robot_pose.x, robot_pose.y, 0])

        for s in subgoals:
            # print(f"Computing distance from {self.robot_name} to {s.pose}")
            goal_pose = self.create_pose_stamped(s.pose)
            path = self.navigator.getPath(start=robot_pose, goal=goal_pose)
            distances[s] = self.get_path_length(path)
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

    def plan_and_navigate(self):
        if self.is_task_complete:
            return
        if self.subgoals is None:
            subgoals = self.graph.container_indices
            self.update(self.graph, self.grid, subgoals, self.robot_pose)
        if self.container_distances is None:
            self.container_distances = self.get_subgoal_distances(self.grid, self.subgoals)
        self.robot_distances = self.get_robot_distances(self.grid, self.robot_pose, self.subgoals)
        chosen_container = self.compute_selected_subgoal()
        self.chosen_container_idxs.append(chosen_container.id)
        subgoal_pose = self.create_pose_stamped(chosen_container.pose)
        chosen_container_dist = self.robot_distances[chosen_container]
        self.navigator.goToPose(subgoal_pose)

        chosen_container_name = self.graph.get_node_name_by_idx(chosen_container.id)
        print(f"Robot {self.robot_name} is navigating to {chosen_container_name}")

        while not self.navigator.isTaskComplete():
            pass
        print(f"Robot {self.robot_name} reached {chosen_container_name}")

        result = self.navigator.getResult()
        if result == TaskResult.FAILED:
            print(f'Robot {self.robot_name} failed to reach container {chosen_container_name}')
        if result == TaskResult.SUCCEEDED:
            print(f'Robot {self.robot_name} reached container {chosen_container_name}')
            self.robot_pose = lsp.Pose(*chosen_container.pose)
            objects_found, probabilities, raw_image, annotated_image = self.get_detected_objects_FAKE(chosen_container_name)
            img_dir = self.save_dir / 'camera_images'
            img_dir.mkdir(parents=True, exist_ok=True)
            raw_image.save(img_dir /
                           f'raw_{self.robot_name}_{chosen_container_name}_{self.args.current_seed}.png')
            annotated_image.save(img_dir /
                                 f'annotated_{self.robot_name}_{chosen_container_name}_{self.args.current_seed}.png')
            print(f"Objects found at {chosen_container_name}: {objects_found}")
            self.add_objects_to_graph(chosen_container.id, objects_found)
            subgoals = [s.id for s in self.subgoals if s.id != chosen_container.id]
            self.update(self.graph, self.grid, subgoals, self.robot_pose)
            self.net_motion += chosen_container_dist
            print(f"Net motion so far: {self.net_motion}")

            if self.target_obj_info['name'] in objects_found:
                self.is_task_complete = True
                self.target_object_container_idx = chosen_container.id
                print(f"Target object {self.target_obj_info['name']} found at {chosen_container_name}. Task complete.")

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
        plt.savefig('occupancy_map_robotics.png', dpi=600)
        exit()

    def get_detected_objects(self, chosen_container_name):
        target_object = self.target_obj_info['name']
        request = ObjectDetectionSrv.Request()
        request.robot_name = self.robot_name
        request.objects_to_find = [target_object]
        response = self.object_detector.call(request)
        raw_image_cv = self.bridge.imgmsg_to_cv2(response.raw_image, desired_encoding='passthrough')
        raw_image_pil = PILImage.fromarray(raw_image_cv, 'RGB')
        annoted_image_cv = self.bridge.imgmsg_to_cv2(response.annotated_image, desired_encoding='passthrough')
        annoted_image_pil = PILImage.fromarray(annoted_image_cv, 'RGB')
        return response.objects_found, response.probabilities, raw_image_pil, annoted_image_pil

    def get_detected_objects_FAKE(self, chosen_container_name):
        # objects_found, probs, raw_image_pil, annotated_image_pil = self.get_detected_objects(chosen_container_name)
        objects_found, probs, raw_image_pil, annotated_image_pil = [], [], PILImage.new('RGB', (640, 480), color='gray'), PILImage.new('RGB', (640, 480), color='gray')
        return OBJECTS_CONTAINED[chosen_container_name], probs, raw_image_pil, annotated_image_pil

    def get_replay_costs_for_all_planners(self):
        lb_costs = np.full((len(self.planners), 2), np.nan)
        planner_costs = np.full(len(self.planners), np.nan)
        self.args.chosen_planner_idx = self.chosen_planner_idx
        for i, planner in enumerate(self.planners):
            self.args.replayed_planner_idx = i
            if i == self.chosen_planner_idx:
                # The cost of the chosen planner is the net distance traveled
                planner_costs[i] = self.net_motion
            else:
                # For other planners, get lower bound costs via offline replay
                optimistic_lb, simply_connected_lb = self.get_lowerbound_planner_costs(planner)
                lb_costs[i] = [optimistic_lb, simply_connected_lb]

        return planner_costs, lb_costs

    def get_lowerbound_planner_costs(self, planner):
        planner.graph = self.graph.copy()
        planner.grid = self.grid
        planner.subgoals = self.graph.container_indices
        robot_container_id = None
        replay_cost = 0.0
        planner.robot_pose = self.initial_robot_pose
        planner.container_distances = self.container_distances
        self.chosen_containers_idxs = []

        while robot_container_id != self.target_object_container_idx:
            planner.update(planner.graph, planner.grid, planner.subgoals, planner.robot_pose)
            chosen_container = planner.compute_selected_subgoal()
            self.chosen_containers_idxs.append(chosen_container.id)

            if robot_container_id is None:
                robot_distances = planner.get_robot_distances(planner.grid, planner.robot_pose, [chosen_container])
                replay_cost += robot_distances[chosen_container]
            else:
                replay_cost += self.container_distances[frozenset(
                    [robot_container_id, chosen_container])]

            planner.subgoals = [s for s in planner.subgoals if s != chosen_container.id]
            robot_container_id = chosen_container.id
            planner.robot_pose = lsp.Pose(*chosen_container.pose)

        chosen_container_names = [planner.graph.get_node_name_by_idx(idx) for idx in self.chosen_containers_idxs]
        print(f"Chosen containers during replay: {chosen_container_names}")
        return [replay_cost, replay_cost]

    def save_trial_data(self):
        priors_file = Path(self.args.save_dir) / f'priors_{self.args.env}_{self.args.current_seed}.txt'
        if priors_file.is_file():
            with open(priors_file) as f:
                priors = np.genfromtxt(priors_file)
                tot_cost_per_planner = priors[-4:-2]
                num_selection_per_planner = priors[-2:]
        else:
            tot_cost_per_planner = np.zeros((2, len(self.args.planner_names)))
            num_selection_per_planner = np.zeros((2, len(self.args.planner_names)))
        min_idx = self.args.chosen_planner_idx
        random.seed(self.args.current_seed)
        np.random.seed(self.args.current_seed)

        all_planners = '_'.join(self.args.planner_names)

        save_dir = self.save_dir
        chosen_planner = self.args.chosen_planner
        current_seed = self.args.current_seed

        cost_file = save_dir / f'cost_{chosen_planner}_all_{all_planners}_{self.args.env}_{current_seed}.txt'
        lb_costs_file = save_dir / f'lbc_{chosen_planner}_all_{all_planners}_{self.args.env}_{current_seed}.txt'
        target_file = save_dir / f'target_plcy_{chosen_planner}_envrnmnt_{self.args.env}_{current_seed}.txt'

        if cost_file.is_file():
            print(f'Data already exists for {chosen_planner}_all_{all_planners}_{self.args.env}_{current_seed}.')
            self.destroy = True
            return

        print(f'Generating data for {chosen_planner}_all_{all_planners}_{self.args.env}_{current_seed}.')

        print("Getting replay costs for all planners.")
        costs, lb_costs = self.get_replay_costs_for_all_planners()
        print(f"Costs: {costs}")
        print(f"Lower-bound costs: {lb_costs}")

        weighted_costs = compute_lbcost_wavg(costs, lb_costs, self.args.chosen_planner_idx, self.args.prob_short)
        weighted_costs_file = save_dir / f'weightedcosts_{self.args.env}_{self.args.current_seed}.txt'
        with open(weighted_costs_file, 'w') as f:
            np.savetxt(f, weighted_costs, newline=' ')
            f.write('\n')
        # nav_data_file = save_dir / f'navdata_{self.args.env}_{self.args.current_seed}.pkl'
        # with open(nav_data_file, 'wb') as f:
        #     pickle.dump(self.navigation_data, f)
        if self.args.selection_strategy == 'ucb':
            tot_cost_per_planner, num_selection_per_planner, min_idx = get_ucb_selection(weighted_costs,
                                                                                         tot_cost_per_planner,
                                                                                         num_selection_per_planner,
                                                                                         min_idx,
                                                                                         c=EXPLORATION_C)
        tot_cost_per_planner, num_selection_per_planner, min_idx = get_lb_selection(weighted_costs,
                                                                                    tot_cost_per_planner,
                                                                                    num_selection_per_planner,
                                                                                    min_idx,
                                                                                    c=EXPLORATION_C)
        with open(save_dir / f'priors_{self.args.env}_{self.args.current_seed + 1}.txt', 'w') as f:
            priors = np.vstack((tot_cost_per_planner, num_selection_per_planner))
            np.savetxt(f, priors)

        with open(save_dir / f'selected_{self.args.env}_{self.args.current_seed + 1}.txt', 'w') as f:
            f.write(f'{self.args.planner_names[min_idx]}\n')

        with open(cost_file, 'w') as f:
            np.savetxt(f, costs)
        with open(lb_costs_file, 'w') as f:
            np.savetxt(f, lb_costs)
        with open(target_file, 'w') as f:
            chosen_container_names = [self.graph.get_node_name_by_idx(idx) for idx in self.chosen_containers_idxs]
            chosen_planners = '\n'.join(chosen_container_names)
            f.write(f'{chosen_planners}\n')

        print(f"Saved trial data to {save_dir}")


def invert_container_mapping(objects_contained):
    """
    Invert the mapping from container -> objects
    to object -> container.
    """
    object_to_container = {}
    for container, objects in objects_contained.items():
        for obj in objects:
            object_to_container[obj] = container
    return object_to_container


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

    all_seeds = list(TARGET_OBJ_BY_SEED.keys())
    planner_names = [
        'lspgptpromptminimal',
        'lspgptpromptb',
        'fullgptpromptdirect',
        # 'lspgeminipromptminimal',
        # 'lspgeminipromptb',
        # 'fullgeminipromptdirect'
    ]
    for seed in all_seeds:
        for chosen_planner_name in planner_names:
            planner_args = lambda: None  # noqa
            planner_args.current_seed = seed
            planner_args.env = 'home'
            planner_args.selection_strategy = 'ucb'  # 'ucb' or 'ours'
            planner_args.prob_short = 0.0
            planner_args.resolution = 1.0
            planner_args.save_dir = f'~/lsp/data/prompt_selection_real_robot/gpt'

            target_obj_info = {
                'name': TARGET_OBJ_BY_SEED[planner_args.current_seed],
            }
            planners = [
                # OptimisticPlanner(target_obj_info, planner_args),
                # OptimisticPlanner(target_obj_info, planner_args),
                # LSPLLMGeminiPlanner(target_obj_info, planner_args, prompt_template_id='prompt_minimal'),
                # LSPLLMGeminiPlanner(target_obj_info, planner_args, prompt_template_id='prompt_b'),
                # FullLLMGeminiPlanner(target_obj_info, planner_args, prompt_template_id='prompt_direct'),
                LSPLLMGPTPlanner(target_obj_info, planner_args, prompt_template_id='prompt_minimal'),
                LSPLLMGPTPlanner(target_obj_info, planner_args, prompt_template_id='prompt_b'),
                FullLLMGPTPlanner(target_obj_info, planner_args, prompt_template_id='prompt_direct')

            ]

            planner_args.planner_names = planner_names
            save_dir = Path(planner_args.save_dir).expanduser()
            save_dir.mkdir(parents=True, exist_ok=True)
            # Get chosen planner computed for this trial
            chosen_planner_file = save_dir / f'selected_{planner_args.env}_{planner_args.current_seed}.txt'
            if chosen_planner_file.is_file():
                with open(chosen_planner_file) as f:
                    planner_args.chosen_planner = f.readlines()[-1].strip()
            else:
                planner_args.chosen_planner = planner_args.planner_names[0]
                with open(chosen_planner_file, 'w') as f:
                    f.write(f'{planner_args.chosen_planner}\n')
            planner_args.chosen_planner = chosen_planner_name
            planner_args.chosen_planner_idx = planner_args.planner_names.index(planner_args.chosen_planner)
            print(f'Running seed {planner_args.current_seed} with chosen planner {planner_args.chosen_planner}.')
            print(f'Target object: {target_obj_info["name"]}')
            print(f'Known target object location: {invert_container_mapping(OBJECTS_CONTAINED)[target_obj_info["name"]]}')
            # input("Press Enter to continue...")

            planner_node = PromptSelectionPlannerNode(target_obj_info, planners, planner_args.chosen_planner_idx, planner_args)
            while planner_node.destroy is False:
                rclpy.spin_once(planner_node)
            planner_node.destroy_node()
            # rclpy.spin(planner_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
