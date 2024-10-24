import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from flir_camera_msgs.msg import ImageMetaData
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime, timedelta

class MultiImageSubscriber(Node):
    def __init__(self):
        super().__init__('multi_image_subscriber_sync')
        
        # 声明一个整型参数'n',默认值为1
        self.declare_parameter('n', 1)
        self.n = self.get_parameter('n').value
        self.get_logger().info(f"Received parameter n: {self.n}")
        
        num_cameras = 2
        self.bridge = CvBridge()
        self.image_counters = [0] * num_cameras
        self.output_dirs = [f'output_images_cam{i}' for i in range(num_cameras)]
        for output_dir in self.output_dirs:
            os.makedirs(output_dir, exist_ok=True)

        self.image_subscriptions = []
        self.meta_subscriptions = []
        self.publishers_ = []
        for i in range(num_cameras):
            image_subscription = self.create_subscription(
                Image,
                f'/cam_sync/cam{i}/image_raw',
                lambda msg, camera_index=i: self.listener_callback(msg, camera_index),
                10)
            self.image_subscriptions.append(image_subscription)

            meta_subscription = self.create_subscription(
                ImageMetaData,
                f'/cam_sync/cam{i}/meta',
                lambda msg, camera_index=i: self.meta_callback(msg, camera_index),
                10)
            self.meta_subscriptions.append(meta_subscription)

            publisher = self.create_publisher(Image, f'/cam_sync/cam{i}/image_raw_rgb', 10)
            self.publishers_.append(publisher)

        self.exposure_times = [0.0] * num_cameras
        self.brightnesses = [0] * num_cameras
        self.max_exposure_times = [0] * num_cameras
        self.gains = [0.0] * num_cameras
        self.image_timestamps_pc = [None] * num_cameras
        self.meta_timestamps = [None] * num_cameras

    def listener_callback(self, msg, camera_index):
        self.get_logger().info(f'Received image from camera {camera_index}')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_color_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerRG2RGB)

        color_msg = self.bridge.cv2_to_imgmsg(cv_color_image, encoding='rgb8')
        color_msg.header = msg.header
        self.publishers_[camera_index].publish(color_msg)

        output_path = os.path.join(self.output_dirs[camera_index], f'image_{self.image_counters[camera_index]:04d}.jpg')
        cv2.imwrite(output_path, cv_color_image)
        self.image_counters[camera_index] += 1

        cv2.imshow(f'Received Image {camera_index}', cv_color_image)
        cv2.waitKey(1)

        self.image_timestamps_pc[camera_index] = self.convert_ros_timestamp_to_datetime(msg.header.stamp)
        self.print_metadata(camera_index)

    def meta_callback(self, msg, camera_index):
        self.exposure_times[camera_index] = msg.exposure_time / 1000.0  # 转换为毫秒
        self.brightnesses[camera_index] = msg.brightness
        self.max_exposure_times[camera_index] = msg.max_exposure_time
        self.gains[camera_index] = msg.gain
        self.meta_timestamps[camera_index] = msg.header.stamp

    def convert_ros_timestamp_to_datetime(self, ros_timestamp):
        return datetime.fromtimestamp(ros_timestamp.sec + ros_timestamp.nanosec / 1e9)

    def print_metadata(self, camera_index):
        self.get_logger().info(f"Camera {camera_index} PC timestamp: {self.image_timestamps_pc[camera_index]}")
        self.get_logger().info(f"Camera {camera_index} Meta timestamp: {self.meta_timestamps[camera_index]}")
        self.get_logger().info(f"Camera {camera_index} Exposure time: {self.exposure_times[camera_index]:.2f} ms")
        self.get_logger().info(f"Camera {camera_index} Brightness: {self.brightnesses[camera_index]}")  
        self.get_logger().info(f"Camera {camera_index} Max exposure time: {self.max_exposure_times[camera_index]} us")
        self.get_logger().info(f"Camera {camera_index} Gain: {self.gains[camera_index]:.2f}")

def main(args=None):
    rclpy.init(args=args) 
    multi_image_subscriber_sync = MultiImageSubscriber()
    rclpy.spin(multi_image_subscriber_sync)
    multi_image_subscriber_sync.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()