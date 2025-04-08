import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.declare_parameter('robot_name', 'robot')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        self.base_frame = 'map'
        self.pose_frame = f'{self.robot_name}/base_link'

        self.timer = self.create_timer(1.0 / 10, self.publish_pose)
        self.pose_pub = self.create_publisher(PoseStamped, f'/{self.robot_name}/current_pose', 10)

    def publish_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.pose_frame, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.base_frame
            pose_msg.header.stamp = transform.header.stamp
            pose_msg.pose.position.x = translation.x
            pose_msg.pose.position.y = translation.y
            pose_msg.pose.position.z = translation.z
            pose_msg.pose.orientation = rotation
            self.pose_pub.publish(pose_msg)

        except TransformException as e:
            self.get_logger().warn(f'Error in transforming: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()