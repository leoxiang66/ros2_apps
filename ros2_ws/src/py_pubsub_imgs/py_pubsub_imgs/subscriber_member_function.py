import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

    def listener_callback(self, msg):
        self.get_logger().info('Received image')
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 使用OpenCV进行Debayering
        cv_color_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerRG2RGB)

        # 将处理后的彩色图像转换回ROS图像消息
        color_msg = self.bridge.cv2_to_imgmsg(cv_color_image, encoding='rgb8')
        color_msg.header = msg.header

        # 发布处理后的彩色图像
        self.publisher.publish(color_msg)

        # 保存处理后的彩色图像到本地
        output_path = os.path.join(self.output_dir, f'image_{self.image_counter:04d}.jpg')
        cv2.imwrite(output_path, cv_color_image)
        self.image_counter += 1

        # 显示图像
        cv2.imshow('Received Image', cv_color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()