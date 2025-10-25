#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from rail_msgs.srv import SaveLocation, CreateTFFrame
from pathlib import Path
from scipy.spatial.transform import Rotation as R
import math
import yaml


class SaveLocationNode(Node):

    def __init__(self):
        super().__init__('save_location_service')
        self.declare_parameter('robot_name', 'robot')
        self.declare_parameter('locations_yaml_file', '~/containers.yaml')

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        locations_yaml_file = self.get_parameter('locations_yaml_file').get_parameter_value().string_value
        self.locations_yaml_file = Path(locations_yaml_file).expanduser()
        self.locations_yaml_file.parent.mkdir(parents=True, exist_ok=True)
        self.locations_yaml_file.touch(exist_ok=True)

        self.robot_pose = None
        self.clicked_point = None

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            f'{self.robot_name}/current_pose',
            self.robot_pose_callback,
            10
        )
        self.clicked_point_subscriber = self.create_subscription(
            PointStamped,
            f'{self.robot_name}/clicked_point',
            self.clicked_point_callback,
            10
        )

        self.save_clicked_point_location_service = self.create_service(
            SaveLocation, 'save_clicked_point_location', self.save_location_from_clicked_point_callback)
        self.save_robot_pose_location_service = self.create_service(
            SaveLocation, 'save_robot_pose_location', self.save_location_from_robot_pose_callback)

        self.create_tf_frame_client = self.create_client(
            CreateTFFrame, 'create_tf_frame')
        while not self.create_tf_frame_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TF Frame Creation service not available, waiting...')

        self.get_logger().info('Save Location Service is ready.')

    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose

    def clicked_point_callback(self, msg):
        self.clicked_point = msg.point
        self.get_logger().info(f'Received clicked point at ({msg.point.x}, {msg.point.y}, {msg.point.z})')

    def save_location_from_clicked_point_callback(self, request, response):
        '''Callback to save the last clicked point as a location. '''
        if self.clicked_point is None:
            response.status_message = ('No clicked point received yet. '
                                       'Please click a point in RViz first and then call this service.')
            return response
        pose = (self.clicked_point.x, self.clicked_point.y, 0.0)
        self._create_tf_and_save_location(request.location_name, request.parent_location, pose)
        response.status_message = (f'Saved location "{request.location_name}" '
                                   f'under "{request.parent_location}" with pose {pose} '
                                   f'at {self.locations_yaml_file}')
        self.clicked_point = None
        return response

    def save_location_from_robot_pose_callback(self, request, response):
        '''Callback to save the robot's current pose as a location.'''
        if self.robot_pose is None:
            response.status_message = ('Robot pose is not available yet. '
                                       f'Is {self.robot_name}/current_pose being published?')
            return response
        pose = (
            self.robot_pose.position.x,
            self.robot_pose.position.y,
            self.get_yaw_from_quaternion(self.robot_pose.orientation)
        )
        self._create_tf_and_save_location(request.location_name, request.parent_location, pose)
        response.status_message = (f'Saved location "{request.location_name}" '
                                   f'under "{request.parent_location}" with pose {pose} '
                                   f'at {self.locations_yaml_file}')
        return response

    def get_yaw_from_quaternion(self, q):
        r = R.from_quat([q.x, q.y, q.z, q.w])
        yaw = r.as_euler('xyz')[2]
        return math.degrees(yaw)

    def send_create_tf_frame_request(self, location_name, pose):
        tf_request = CreateTFFrame.Request(
            frame_name=location_name,
            x=pose[0],
            y=pose[1],
            yaw=pose[2]
        )
        self.get_logger().info(f'Creating TF frame: {location_name} at ({pose[0]}, {pose[1]}, {pose[2]})')

        future = self.create_tf_frame_client.call_async(tf_request)

        def handle_tf_response(fut):
            try:
                tf_response = fut.result()
                if not tf_response.success:
                    self.get_logger().error(f'Failed to create TF frame: {tf_response.message}')
                else:
                    self.get_logger().info(f'TF frame {tf_request.frame_name} created successfully.')
            except Exception as e:
                self.get_logger().error(f'Error while creating TF frame: {e}')

        future.add_done_callback(handle_tf_response)
        return

    def _create_tf_and_save_location(self, location_name, parent_location, pose):
        # Send TF frame creation request
        self.send_create_tf_frame_request(location_name, pose)

        # Save to YAML file (read existing data, update, write back)
        if self.locations_yaml_file.stat().st_size == 0:
            data = {}
        else:
            with open(self.locations_yaml_file, 'r') as f:
                data = yaml.safe_load(f) or {}

        x, y, yaw = pose
        if parent_location == "":
            # Save top-level location
            if location_name in data:
                # update existing pose with new values
                data[location_name]['x'] = x
                data[location_name]['y'] = y
                data[location_name]['yaw'] = yaw
            else:
                data[location_name] = {
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'containers': {}
                }
        else:
            # Save under parent container
            if parent_location not in data:
                data[parent_location] = {'x': 0, 'y': 0, 'yaw': 0, 'containers': {}}
            data[parent_location]['containers'][location_name] = {
                'x': x,
                'y': y,
                'yaw': yaw
            }

        with open(self.locations_yaml_file, 'w') as f:
            yaml.dump(data, f, sort_keys=False)


def main():
    rclpy.init()
    save_location_node = SaveLocationNode()
    rclpy.spin(save_location_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
