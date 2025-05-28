import os
from groundingdino.util.inference import load_model, predict
import groundingdino.datasets.transforms as T

GROUNDINGDINO_PATH = "/home/ab/projects/rail_physical_robot/GroundingDINO/groundingdino"
config_path = os.path.join(GROUNDINGDINO_PATH, "config/GroundingDINO_SwinT_OGC.py")
weights_path = os.path.join(GROUNDINGDINO_PATH, "weights/groundingdino_swint_ogc.pth")
model = load_model(config_path, weights_path, device="cpu")
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25

NAME_MAPPING = [
    ('toiletpaper', 'toilet paper'),
    ('creditcard', 'credit card'),
    ('cellphone', 'cell phone'),
    ('shelvingunit', 'shelving unit'),
]


def transform_image(image):
    transform = T.Compose(
        [
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ]
    )

    image_transformed, _ = transform(image, None)
    return image_transformed


def get_revealed_objects(camera_image, objects_to_find, reached_container_name='', output_dir='~/mr_task_data'):
    output_dir = os.path.expanduser(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    camera_image.save(os.path.join(output_dir, f'{reached_container_name}_image.png'))
    image = transform_image(camera_image)
    name_mapping = {k: v for k, v in NAME_MAPPING}
    objects_to_find = [name_mapping.get(obj, obj) for obj in objects_to_find]
    text_prompt = " . ".join(objects_to_find) + " ."
    _, _, objects_found = predict(
        model=model,
        image=image,
        caption=text_prompt,
        box_threshold=BOX_THRESHOLD,
        text_threshold=TEXT_THRESHOLD,
        device="cpu"
    )
    inverse_name_mapping = {v: k for k, v in NAME_MAPPING}
    objects_found = [inverse_name_mapping.get(obj, obj) for obj in objects_found]
    print(f"Objects found: {objects_found}")
    return objects_found


def get_revealed_objects_FAKE(camera_image, objects_to_find, reached_container_name='', output_dir='~/mr_task_data'):
    objects_contained = {
        'diningtable': ['plate', 'fork'],
        'countertop': ['toaster', 'laptop'],
        'drawer': ['book'],
        'sidetable': ['keys', 'book'],
        'sink': [],
        'dresser': ['wallet'],
        'garbagecan': []
    }
    return objects_contained[reached_container_name]
