import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSimNode(Node):
    def __init__(self):
        super().__init__('camera_sim_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        timer_period = 0.1  # 发布频率为10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        # 生成模拟图像数据
        height, width = 480, 640
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image = cv2.circle(image, (width//2, height//2), 100, (0, 255, 0), -1)
        
        # 将OpenCV图像转换为ROS图像消息
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        
        # 发布图像消息
        self.publisher_.publish(image_msg)
        self.get_logger().info('Publishing image')

def main(args=None):
    rclpy.init(args=args)
    camera_sim_node = CameraSimNode()
    rclpy.spin(camera_sim_node)
    camera_sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()