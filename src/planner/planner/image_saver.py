import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage
from cv_bridge import CvBridge
import message_filters
import os


class ImageSaverNode(Node):
    def __init__(self, output_dir='~/mr_task_data'):
        super().__init__('image_saver_node')

        self.declare_parameter('all_robot_names', 'robot')
        self.all_robot_names = self.get_parameter(
            'all_robot_names').get_parameter_value().string_value.split(',')
        self.output_dir = os.path.expanduser(output_dir)
        os.makedirs(self.output_dir, exist_ok=True)
        self.camera_img_subscribers = {
            robot: message_filters.Subscriber(
                self, Image, f'/{robot}/camera/color/image_raw')
            for robot in self.all_robot_names}
        camera_img_ts = message_filters.ApproximateTimeSynchronizer(
            self.camera_img_subscribers.values(), 10, slop=4)
        camera_img_ts.registerCallback(self.save_camera_images)

        self.bridge = CvBridge()

    def save_camera_images(self, *args):
        for i, msg in enumerate(args):
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            final_path = os.path.join(self.output_dir, f'{self.all_robot_names[i]}_image.png')
            temp_path = final_path + '.tmp.png'
            image = PILImage.fromarray(cv_image, 'RGB')
            image.save(temp_path)
            os.replace(temp_path, final_path)


def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaverNode()
    rclpy.spin(image_saver_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
