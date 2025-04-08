import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import math
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
import message_filters
from scipy.optimize import linear_sum_assignment
import random


class BasePlannerNode(Node):
    """Abstract Planner class"""

    def __init__(self, name=None):
        name = name if name is not None else 'base_planner'
        super().__init__(name)

        self.declare_parameter('all_robot_names', 'robot')
        self.declare_parameter(
            'container_info_file', '/home/abhish/ros2/rail_robot/src/rail_robot/worlds/floor_map_containers.yaml')

        self.all_robot_names = self.get_parameter(
            'all_robot_names').get_parameter_value().string_value.split(',')
        print(self.all_robot_names)
        yaml_file = self.get_parameter(
            'container_info_file').get_parameter_value().string_value

        if yaml_file is None:
            raise ValueError("container_info_file cannot be None.")

        self.points = load_points_from_yaml(yaml_file)

        self.robot_poses = {robot: None for robot in self.all_robot_names}

        self.pose_subscribers = {
            robot: message_filters.Subscriber(
                self, PoseStamped, f'/{robot}/current_pose')
            for robot in self.all_robot_names}

        ts = message_filters.ApproximateTimeSynchronizer(
            self.pose_subscribers.values(), 10, slop=4)
        ts.registerCallback(self.set_poses)
        print('Waiting for all robot poses')

        self.navigators = {robot: BasicNavigator(
            namespace=robot) for robot in self.all_robot_names}
        print("Waiting until nav2 is active for all robots")
        for nav in self.navigators.values():
            nav.lifecycleStartup()
        self.is_goal_reached = {robot: False for robot in self.all_robot_names}

    def set_poses(self, *args):
        """Callback to process robot's current position and publish nearest goal."""
        for i, msg in enumerate(args):
            robot = self.all_robot_names[i]
            self.robot_poses[robot] = msg
        if any(self.is_goal_reached):
            self.plan_and_navigate()

    def plan_and_navigate(self):
        if len(self.points) == 0:
            print("Task complete. All subgoals explored.")
            for robot in self.all_robot_names:
                self.is_goal_reached[robot] = True
            return

        robot_subgoal_distances = self.get_robot_subgoal_distances()
        joint_action = self.get_joint_action(robot_subgoal_distances)

        goal_poses = {}
        for robot in self.all_robot_names:
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = "map"
            point = self.points[joint_action[robot]]
            goal_pose.pose.position.x = point[0]
            goal_pose.pose.position.y = point[1]
            goal_pose.pose.position.z = point[2]
            goal_pose.pose.orientation.w = 1.0
            goal_poses[robot] = goal_pose

        for robot in self.all_robot_names:
            nav = self.navigators[robot]
            goal_pose = goal_poses[robot]
            nav.goToPose(goal_pose)

        while not any(self.is_goal_reached.values()):
            self.is_goal_reached = {robot: nav.isTaskComplete()
                                    for robot, nav in self.navigators.items()}

        completed_robot = None
        for robot, is_complete in self.is_goal_reached.items():
            if is_complete:
                completed_robot = robot

        reached_subgoal = joint_action[completed_robot]
        result = self.navigators[completed_robot].getResult()
        if result == TaskResult.FAILED:
            print(f'{completed_robot} failed to reach subgoal {reached_subgoal}')
        if result == TaskResult.SUCCEEDED:
            print(f'{completed_robot} reached subgoal {reached_subgoal}')
        self.is_goal_reached[completed_robot] = False
        del self.points[reached_subgoal]

    def get_robot_subgoal_distances(self):
        """Computes the ordering for visiting subgoals"""
        subgoal_distances = {}
        for robot in self.all_robot_names:
            robot_pose = self.robot_poses[robot]
            for subgoal, (px, py, _) in self.points.items():
                distance = math.sqrt(
                    (robot_pose.pose.position.x - px) ** 2 + (robot_pose.pose.position.y - py) ** 2)
                subgoal_distances[(robot, subgoal)] = distance

        return subgoal_distances

    def get_joint_action(self, robot_subgoal_distances):
        """Get subgoal assignments for each robot"""
        cost_matrix = []
        subgoal_matrix = []
        for robot in self.all_robot_names:
            cost = []
            subgoal = []
            for sg_name in self.points.keys():
                cost.append(robot_subgoal_distances[(robot, sg_name)])
                subgoal.append(sg_name)
            cost_matrix.append(cost)
            subgoal_matrix.append(subgoal)
        joint_action_list = find_action_list_from_cost_matrix_using_lsa(
            cost_matrix, subgoal_matrix)
        joint_action = {}
        for i, robot in enumerate(self.all_robot_names):
            joint_action[robot] = joint_action_list[i]

        return joint_action


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
    planner_node = BasePlannerNode()
    rclpy.spin(planner_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()