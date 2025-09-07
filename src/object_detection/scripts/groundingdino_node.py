#!/usr/bin/env python3

import os
import threading
import numpy as np
from functools import partial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage
from object_detection.srv import ObjectDetectionSrv
from groundingdino.util.inference import load_model, predict, annotate
import groundingdino.datasets.transforms as T

GROUNDINGDINO_PATH = "~/projects/rail_physical_robot/GroundingDINO"
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25

NAME_MAPPING = [
    ('toiletpaper', 'toilet paper'),
    ('creditcard', 'credit card'),
    ('cellphone', 'cell phone'),
    ('shelvingunit', 'shelving unit'),
    ('remotecontrol', 'remote control'),
]


def transform_image(image):
    transform = T.Compose([T.RandomResize([800], max_size=1333),
                           T.ToTensor(),
                           T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])
    image_transformed, _ = transform(image, None)
    return image_transformed


def detect_objects(model, camera_image, objects_to_find):
    image = transform_image(camera_image)
    name_mapping = {k: v for k, v in NAME_MAPPING}
    objects_to_find = [name_mapping.get(obj, obj) for obj in objects_to_find]
    text_prompt = " . ".join(objects_to_find) + " ."

    boxes, logits, objects_found = predict(
        model=model,
        image=image,
        caption=text_prompt,
        box_threshold=BOX_THRESHOLD,
        text_threshold=TEXT_THRESHOLD,
        device="cpu"
    )

    inverse_name_mapping = {v: k for k, v in NAME_MAPPING}
    objects_found = [inverse_name_mapping.get(obj, obj) for obj in objects_found]

    annotated_image = annotate(
        np.array(camera_image),
        boxes=boxes,
        logits=logits,
        phrases=objects_found,
    )
    annotated_image = PILImage.fromarray(annotated_image[..., ::-1].astype(np.uint8))

    prob_scores = logits.tolist()

    return objects_found, prob_scores, annotated_image


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('groundingdino_object_detection_node')

        self.bridge = CvBridge()
        self.latest_images = {}
        self.image_locks = {}

        self.declare_parameter('groundingdino_path', GROUNDINGDINO_PATH)
        self.declare_parameter('all_robot_names', 'robot')

        self.groundingdino_model = self.get_groundingdino_model()

        all_robot_names = self.get_parameter(
            'all_robot_names').get_parameter_value().string_value.split(',')

        for robot in all_robot_names:
            self.latest_images[robot] = None
            self.image_locks[robot] = threading.Lock()
            self.create_subscription(
                Image,
                f'/{robot}/camera/color/image_raw',
                partial(self.image_callback, robot_name=robot),
                10
            )

        self.service = self.create_service(
            ObjectDetectionSrv,
            '/groundingdino_object_detection',
            self.handle_object_detection
        )

        self.get_logger().info("GroundingDINO Object Detection Service Ready!")

        # use test images from ~/mr_task_data for debugging
        # import cv2
        # image_pil = PILImage.open(os.path.expanduser("~/mr_task_data/couch_image.png"))
        # image_cv = cv2.cvtColor(np.array(image_pil), cv2.COLOR_RGB2BGR)
        # img_msg = self.bridge.cv2_to_imgmsg(image_cv, encoding="bgr8")
        # self.latest_images[robot] = img_msg

    def get_groundingdino_model(self):
        groundingdino_path = self.get_parameter('groundingdino_path').get_parameter_value().string_value
        groundingdino_path = os.path.expanduser(groundingdino_path)
        config_path = os.path.join(groundingdino_path, "groundingdino/config/GroundingDINO_SwinT_OGC.py")
        weights_path = os.path.join(groundingdino_path, "groundingdino/weights/groundingdino_swint_ogc.pth")
        model = load_model(config_path, weights_path, device="cpu")
        return model

    def image_callback(self, msg, robot_name):
        with self.image_locks[robot_name]:
            self.latest_images[robot_name] = msg

    def handle_object_detection(self, request, response):
        robot_name = request.robot_name
        objects_to_find = request.objects_to_find

        if robot_name not in self.latest_images:
            self.get_logger().error(f"No subscriber for robot {robot_name}")
            return response

        with self.image_locks[robot_name]:
            msg = self.latest_images[robot_name]

        if msg is None:
            self.get_logger().warn(f"No image received yet for robot {robot_name}")
            return response

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        camera_image = PILImage.fromarray(cv_image, 'RGB')

        objects_found, probs, annotated_pil = detect_objects(self.groundingdino_model,
                                                             camera_image,
                                                             objects_to_find)
        self.get_logger().info(f"Objects detected by {robot_name}: {objects_found} with probabilities {probs}")
        raw_ros = self.bridge.cv2_to_imgmsg(np.array(camera_image), encoding="rgb8")
        annotated_ros = self.bridge.cv2_to_imgmsg(np.array(annotated_pil), encoding="rgb8")

        response.objects_found = objects_found
        response.probabilities = probs
        response.raw_image = raw_ros
        response.annotated_image = annotated_ros
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
