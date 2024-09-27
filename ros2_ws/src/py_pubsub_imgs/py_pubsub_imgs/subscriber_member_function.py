import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from flir_camera_msgs.msg import ImageMetaData
from cv_bridge import CvBridge
import cv2
import os

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/flir_camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量的警告
        self.publisher = self.create_publisher(Image, '/flir_camera/image_raw_rgb', 10)
        self.bridge = CvBridge()
        self.image_counter = 0
        self.output_dir = 'output_images'
        os.makedirs(self.output_dir, exist_ok=True)

        self.meta_subscription = self.create_subscription(
            ImageMetaData,
            '/flir_camera/meta',
            self.meta_callback,
            10)
        self.meta_subscription  # 防止未使用变量的警告

        self.exposure_time = 0.0
        self.brightness = 0
        self.max_exposure_time = 0
        self.gain = 0.0
        self.image_timestamp = None
        self.meta_timestamp = None

    def listener_callback(self, msg):
        self.get_logger().info('Received image')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_color_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerRG2RGB)

        color_msg = self.bridge.cv2_to_imgmsg(cv_color_image, encoding='rgb8')
        color_msg.header = msg.header
        self.publisher.publish(color_msg)

        output_path = os.path.join(self.output_dir, f'image_{self.image_counter:04d}.jpg')
        cv2.imwrite(output_path, cv_color_image)
        self.image_counter += 1

        cv2.imshow('Received Image', cv_color_image)
        cv2.waitKey(1)

        self.image_timestamp = msg.header.stamp
        self.print_metadata()

    def meta_callback(self, msg):
        self.exposure_time = msg.exposure_time / 1000.0  # 转换为毫秒
        self.brightness = msg.brightness
        self.max_exposure_time = msg.max_exposure_time
        self.gain = msg.gain
        self.meta_timestamp = msg.header.stamp

    def print_metadata(self):
        self.get_logger().info(f"Image timestamp: {self.image_timestamp.sec}.{self.image_timestamp.nanosec}")
        self.get_logger().info(f"Meta timestamp: {self.meta_timestamp.sec}.{self.meta_timestamp.nanosec}")
        self.get_logger().info(f"Exposure time: {self.exposure_time:.2f} ms")
        self.get_logger().info(f"Brightness: {self.brightness}")
        self.get_logger().info(f"Max exposure time: {self.max_exposure_time} us")
        self.get_logger().info(f"Gain: {self.gain:.2f}")

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()