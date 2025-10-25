#!/usr/bin/env python3
from rclpy.node import Node
from rail_msgs.srv import GoToLocation
from rail_robot.navigation import get_go_to_location_fn


class GoToLocationNode(Node):

    def __init__(self):
        super().__init__('go_to_location_service')
        self.declare_parameter('all_robot_names', 'robot')
        self.all_robot_names = self.get_parameter(
            'all_robot_names').get_parameter_value().string_value.split(',')
        self.go_to_location_fns = {
            robot_name: get_go_to_location_fn(robot_name) for robot_name in self.all_robot_names
        }
        self.go_to_location_service = self.create_service(
            GoToLocation,
            'go_to_location',
            self.go_to_location_callback
        )

    def go_to_location_callback(self, request, response):
        robot_name = request.robot_name
        location_name = request.location_name

        if robot_name not in self.go_to_location_fns:
            response.success = False
            self.get_logger().error(f"Unknown robot name: {robot_name}")
            return response

        nav_status = self.go_to_location_fns[robot_name](location_name)
        if nav_status is None:
            response.success = False
            self.get_logger().error(f"Failed to navigate {robot_name} to {location_name}")
            return response

        response.success = True
        return response
