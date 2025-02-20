import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

FGAM = -0.2802691155586553
FA = 0.5720307511084637
FFX = 137.01933323959736
FFY = 137.03194355038187
FCX = 399.6653434514119
FCY = 319.64949568986646

RGAM = 0.0050838274846042785
RA = 0.7064437783234423
RFX = 190.58158162432707
RFY = 190.5749086860181
RCX = 319.3219987712258
RCY = 319.71737830659083

WIDTH = 720
HEIGHT = 640

class ImageStitcher:
    def __init__(self, front_params, rear_params, width, height):
        self.width = width
        self.height = height
        self.fmapping_x, self.fmapping_y = self.get_projection(self.width, self.height, front_params)
        self.rmapping_x, self.rmapping_y = self.get_projection(self.width, self.height, rear_params)

    def get_projection(self, x, y, intrinsics):
        # Creates the camera projection mapping
        mapping_x = np.zeros((x, y), np.float32)
        mapping_y = np.zeros((x, y), np.float32)
        for i in range(0, x):
            for j in range(0, y):
                equirectangular = self.carttosphere(self.projection_characteristics(i, j, intrinsics))
                coord1 = np.round(x / 2 + ((equirectangular[2]) / np.pi) * x / 2)
                coord2 = np.round(y / 2 + ((equirectangular[1]) / np.pi) * y / 2)
                try:
                    mapping_x[int(coord1), int(coord2)] = i
                    mapping_y[int(coord1), int(coord2)] = j
                except Exception:
                    pass
        mapping_x = cv2.flip(cv2.transpose(mapping_x), flipCode=0)
        mapping_x = cv2.medianBlur(mapping_x, 5)
        mapping_y = cv2.flip(cv2.transpose(mapping_y), flipCode=0)
        mapping_y = cv2.medianBlur(mapping_y, 5)
        return mapping_x, mapping_y

    def convert(self, cv_image):
        # Takes care of transforms and sizes for the image
        front_image = cv_image[:, 0:cv_image.shape[1] // 2]
        rear_image = cv_image[:, cv_image.shape[1] // 2:]
        front_image = cv2.flip(cv2.transpose(front_image), flipCode=1)
        rear_image = cv2.flip(cv2.transpose(rear_image), flipCode=0)
        fronta, frontb = 180, 550
        reara, rearb = 180, 560
        temp_front = cv2.remap(front_image, self.fmapping_x, self.fmapping_y,
                               interpolation=cv2.INTER_LINEAR)[30:300, fronta:frontb]
        temp_rear = cv2.remap(rear_image, self.rmapping_x, self.rmapping_y,
                              interpolation=cv2.INTER_LINEAR)[30:300, reara:rearb]
        self.front_image = cv2.resize(temp_front, (self.height, self.width))
        self.rear_image = cv2.resize(temp_rear, (self.height, self.width))

    def intelli_stitch(self, cv_image):
        # Dynamically selects where to stitch the images based on intensity
        self.convert(cv_image)
        if self.front_image is not None and self.rear_image is not None:
            front = cv2.cvtColor(self.front_image, cv2.COLOR_BGR2GRAY)
            rear = cv2.cvtColor(self.rear_image, cv2.COLOR_BGR2GRAY)
            considered = 2
            range = 10
            r2f = self.find_match(front, rear, considered, range) + 1
            f2r = self.find_match(rear, front, considered, range) + 1

            panel1 = self.rear_image[:, :-r2f]
            panel2 = self.front_image[:, :-f2r]
            panel3 = self.rear_image[:, :-r2f]
            width = panel1.shape[1] + panel2.shape[1] + panel3.shape[1]
            height = panel1.shape[0]
            dest_image = np.zeros((height, width, 3), dtype=np.uint8)
            c = 0
            for img in [panel1, panel2, panel3]:
                dest_image[:, c:img.shape[1] + c, :] = img
                c += img.shape[1]

            return dest_image

    def find_match(self, img1, img2, num_cols, candidate_range):
        # Helper for intelligent stitching
        stitch_col = img1[:, 0:num_cols]
        min_error = 10000
        for i in range(num_cols, candidate_range + num_cols):
            stitch_candidate = img2[:, -i - num_cols:-i]
            error = np.sqrt(np.mean(np.power(stitch_col.flatten() - stitch_candidate.flatten(), 2)))
            if error < min_error:
                min_error = error
                loc = i + num_cols
        return loc

    def stitch(self, cv_image):
        # Applies static bounds for the stitching
        self.convert(cv_image)
        if self.front_image is not None and self.rear_image is not None:
            front = self.front_image
            rear = self.rear_image

            panel1 = rear[:, rear.shape[1] // 2:]
            panel2 = front[:, :]
            panel3 = rear[:, :rear.shape[1] // 2]

            width = panel1.shape[1] + panel2.shape[1] + panel3.shape[1]
            height = np.nanmax([panel1.shape[0], panel2.shape[0]])
            dest_image = np.zeros((height, width, 3), dtype=np.uint8)
            c = 0
            for img in [panel1, panel2, panel3]:
                dest_image[:, c:img.shape[1] + c, :] = img
                c += img.shape[1]
            dest_image = cv2.resize(dest_image, (512, 256))
            return dest_image

    def projection_characteristics(self, u, v, intrinsics):
        # Performs camera unprojection according to double sphere model
        a, gam, fx, fy, cx, cy = intrinsics
        mx = (u - cx) / fx
        my = (v - cy) / fy
        r2 = mx**2 + my**2
        mz = (1 - a**2 * r2) / (a * np.sqrt(1 - (2 * a - 1) * r2) + 1 - a)
        coeff = (mz * gam + np.sqrt(mz**2 + (1 - gam**2) * r2)) / (mz**2 + r2)
        return coeff * mx, coeff * my, coeff * mz - gam

    def carttosphere(self, coords):
        # Transforms x,y,z coordinates to spherical coordinates in equirectangular image
        r2 = coords[0]**2 + coords[1]**2 + coords[2]**2
        theta = np.arccos([coords[1] / np.sqrt(r2)])  # from 0 to pi
        phi = np.arctan2(coords[0], coords[2])  # from -pi to pi
        return r2, theta, phi

class PanoCameraNode(Node):
    def __init__(self):
        super().__init__('pano_camera_node')
        
        self.width = WIDTH
        self.height = HEIGHT
        self.publish_rate = 4.0  # 4Hz = 0.25 Seconds
        
        self.front_intrinsics = [FA, FGAM, FFX, FFY, FCX, FCY]
        
        self.rear_intrinsics =  [RA, RGAM, RFX, RFY, RCX, RCY]
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Camera initialization failed")
            
        self.stitcher = ImageStitcher(self.front_intrinsics, self.rear_intrinsics, self.width, self.height)
        
        self.publisher = self.create_publisher(
            Image,
            'pano_image',
            10
        )
        
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_pano
        )
        
        self.bridge = CvBridge()
        self.get_logger().info("Pano Camera Node initialized")

    def publish_pano(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture")
            return
            
        try:
            pano_img = self.stitcher.stitch(frame)
            msg = self.bridge.cv2_to_imgmsg(pano_img, encoding="passthrough")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
        except Exception as e:
            print(e)

def main(args=None):
    rclpy.init(args=args)
    node = PanoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()