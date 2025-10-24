#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import yaml


class SavedLocationsTFBroadcaster(Node):
    """Node to broadcast saved locations from YAML file as TF frames."""

    def __init__(self):
        super().__init__('saved_locations_tf_broadcaster')

        # Declare parameters
        self.declare_parameter('file_name', 'containers.yaml')
        self.declare_parameter('publish_rate', 1.0)  # Hz

        # Get parameters
        file_name = self.get_parameter('file_name').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # YAML file path
        self.locations_file_path = Path('~/').expanduser() / file_name
        if not self.locations_file_path.exists():
            self.get_logger().warn(f"File not found: {self.locations_file_path}")
            self.locations_file_path.touch()

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to periodically broadcast transforms
        self.timer = self.create_timer(1.0 / self.publish_rate, self.broadcast_saved_locations)

        self.get_logger().info(f"Broadcasting TFs from: {self.locations_file_path}")

    def broadcast_saved_locations(self):
        """Broadcast all saved locations as TF frames."""
        try:
            if self.locations_file_path.stat().st_size == 0:
                return

            with open(self.locations_file_path, 'r') as f:
                data = yaml.safe_load(f) or {}

            for name, info in data.items():
                self._broadcast_pose(name, info, parent_frame='map')
                if 'containers' in info:
                    for cname, cinfo in info['containers'].items():
                        self._broadcast_pose(cname, cinfo, parent_frame=name)

        except Exception as e:
            self.get_logger().warn(f"Failed to broadcast TFs: {e}")

    def _broadcast_pose(self, name, info, parent_frame='map'):
        """Send a single transform for one saved location."""
        if any(k not in info for k in ('x', 'y', 'yaw')):
            self.get_logger().warn(f"Skipping invalid entry: {name}")
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = name

        t.transform.translation.x = info['x']
        t.transform.translation.y = info['y']
        t.transform.translation.z = 0.0

        r = R.from_euler('z', info['yaw'], degrees=True)
        q = r.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = SavedLocationsTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
