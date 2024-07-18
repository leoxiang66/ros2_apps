import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubNode(Node):
    def __init__(self):
        super().__init__('camera_sub_node')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 在图像上绘制文本
        cv2.putText(cv_image, 'Subscribed Image', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 显示图像
        cv2.imshow('Camera Image', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_sub_node = CameraSubNode()
    rclpy.spin(camera_sub_node)
    camera_sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()