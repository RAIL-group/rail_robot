from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException
import rclpy
from rclpy.duration import Duration


class NavigationStatus:
    def __init__(self, is_done, get_result, cancel_task):
        self.is_done = is_done
        self.get_result = get_result
        self.cancel_task = cancel_task


def get_go_to_location_fn(robot_name):
    """
    Returns a function go_to_location(location_name)
    that:
      - Looks up TF transform 'map' -> location_name
      - Sends a goal via Nav2's BasicNavigator
      - Returns (is_done_fn, get_result_fn, cancel_task_fn)
    """

    node = rclpy.create_node(f'{robot_name}_go_to_location')

    # Initialize the navigator for this robot namespace
    navigator = BasicNavigator(namespace=robot_name)
    node.get_logger().info(f"Waiting for {robot_name}'s Nav2 stack to become active...")
    navigator.lifecycleStartup()

    # TF setup
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    def get_location_pose(location_name):
        """Look up transform for location_name -> map and return PoseStamped."""
        try:
            # Slight delay for TF to populate
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.2)
            tf = tf_buffer.lookup_transform(
                'map', location_name, rclpy.time.Time(), timeout=Duration(seconds=2.0)
            )

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            return pose

        except TransformException as e:
            node.get_logger().error(f"TF lookup failed for {location_name}: {e}")
            return None

    def go_to_location(location_name):
        """
        Sends navigation goal for the given TF frame.
        Returns 'NavigationStatus' object.
        """
        pose = get_location_pose(location_name)
        if pose is None:
            node.get_logger().error(f"Pose not found for {location_name}. Cannot navigate.")
            return None

        node.get_logger().info(f"Navigating to {location_name}...")
        navigator.goToPose(pose)

        return NavigationStatus(
            is_done=navigator.isTaskComplete,
            get_result=navigator.getResult,
            cancel_task=navigator.cancelTask
        )

    return go_to_location
