import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_imgs', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        if self.count < 10:
            
            # 打开PNG图像文件
            pil_image = PILImage.open(f'/home/xiang-tao/git/ros2_apps/images/image_{self.count}.png')

            # 将PIL图像转换为NumPy数组
            cv_image = np.array(pil_image)

            # 将NumPy数组转换为ROS2图像消息
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')

            # 发布图像消息
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing image')
            self.count += 1
        else:
            return

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()