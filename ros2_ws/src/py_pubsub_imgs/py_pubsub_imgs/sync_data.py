import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, PointCloud2
from datetime import datetime, timezone, timedelta
import time
from queue import Queue
from threading import Thread

class TimestampSyncNode(Node):
    def __init__(self):
        super().__init__('timestamp_sync_node')
        self.img_que = Queue()
        self.lidar_que = Queue()


        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/hikvision/camera/image_raw/compressed',
            self.image_callback,
            10
        )
        self.pc2_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pc2_callback,
            10
        )
        
        self.image_pub = self.create_publisher(
            CompressedImage,
            '/AGV/camera/image_compressed',
            10
        )
        
        self.pc2_pub = self.create_publisher(
            PointCloud2,
            '/AGV/lidar',
            10
        )

        Thread(target=self._publish,daemon=True).start()

        print('TimestampSyncNode started.')
        
    def _publish(self):
        while rclpy.ok(): 
            if not self.img_que.empty() and not self.lidar_que.empty():
                print("Publishing data ...")
                img_msg = self.img_que.get()
                pc2_msg = self.lidar_que.get()
                self.image_pub.publish(img_msg)
                self.pc2_pub.publish(pc2_msg)
            else:
                time.sleep(0.001) 
        

    def image_callback(self, msg):
        print("receive image ...")
        self.img_que.put(msg)        
    
    def pc2_callback(self, msg):
        print("recieve PC2")
        self.lidar_que.put(msg)
            


def main(args=None):
    rclpy.init(args=args)
    node = TimestampSyncNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()