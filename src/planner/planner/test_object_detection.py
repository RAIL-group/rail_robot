import os
from PIL import Image as PILImage
from . import object_detection_local
import rclpy


def get_robot_image(robot_name, path='~/mr_task_data'):
    print(f"Getting image from {robot_name}")
    path = os.path.expanduser(path)
    image_path = os.path.join(path, f"{robot_name}_image.png")
    image = PILImage.open(image_path)
    return image


def main(args=None):
    # rclpy.init(args=args)
    robot_name = "robot1"
    camera_image = get_robot_image(robot_name)
    objects_to_find = ['pillow', 'remotecontrol']
    reached_container_name = robot_name
    objects_found = object_detection_local.get_revealed_objects(camera_image, objects_to_find, reached_container_name)
    print(f"Objects found at {reached_container_name}: {objects_found}")


if __name__ == '__main__':
    main()
