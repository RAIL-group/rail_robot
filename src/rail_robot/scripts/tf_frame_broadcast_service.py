#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from rail_msgs.srv import CreateTFFrame, DeleteTFFrame
import yaml
from pathlib import Path


class TFFrameBroadcaster(Node):
    """Node to broadcast a TF frame based on service requests."""

    def __init__(self):
        super().__init__('rail_tf_frame_broadcaster')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('locations_yaml_file', '')
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        locations_yaml_file = self.get_parameter('locations_yaml_file').get_parameter_value().string_value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_frame_service = self.create_service(
            CreateTFFrame,
            'create_tf_frame',
            self.create_frame_callback
        )
        self.delete_frame_service = self.create_service(
            DeleteTFFrame,
            'delete_tf_frame',
            self.delete_frame_callback
        )

        self.frames = self._load_frames_from_file(locations_yaml_file) if locations_yaml_file else {}

        # Timer to periodically broadcast transforms
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.get_logger().info("TF Frame Broadcaster Node Started")

    def timer_callback(self):
        """Broadcast all registered frames."""
        for frame_name, (x, y, yaw) in self.frames.items():
            self.broadcast_frame(frame_name, x, y, yaw)

    def create_frame_callback(self, request, response):
        frame_name = request.frame_name
        x = request.x
        y = request.y
        yaw = request.yaw

        self.frames[frame_name] = (x, y, yaw)
        response.success = True
        self.get_logger().info(f"Created TF frame: {frame_name} at ({x}, {y}, {yaw})")
        return response

    def delete_frame_callback(self, request, response):
        frame_name = request.frame_name
        if frame_name in self.frames:
            del self.frames[frame_name]
            response.success = True
            self.get_logger().info(f"Deleted TF frame: {frame_name}")
        else:
            response.success = False
            self.get_logger().warn(f"TF frame not found: {frame_name}")
        return response

    def broadcast_frame(self, frame_name, x, y, yaw):
        """Broadcast a single transform for the given frame."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = frame_name
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        quat = R.from_euler('z', yaw).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def _load_frames_from_file(self, yaml_file_path):
        frames = {}
        yaml_file_path = Path(yaml_file_path).expanduser()
        if not yaml_file_path.exists():
            self.get_logger().warn(f"YAML file not found: {yaml_file_path}. No frames loaded.")
            return frames

        with open(yaml_file_path, 'r') as f:
            data = yaml.safe_load(f) or {}

        for name, info in data.items():
            frames[name] = (info['x'], info['y'], info['yaw'])
            containers = info.get('containers', {})
            for cname, cinfo in containers.items():
                frames[cname] = (cinfo['x'], cinfo['y'], cinfo['yaw'])
        return frames


def main():
    rclpy.init()
    node = TFFrameBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
