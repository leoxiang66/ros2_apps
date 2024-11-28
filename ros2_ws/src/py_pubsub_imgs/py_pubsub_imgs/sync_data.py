import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs_py import point_cloud2
import os
from datetime import datetime, timedelta
from typing import List, Tuple
import std_msgs.msg


class LivoxPointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('livox_point_cloud_subscriber')
        self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.listener_callback,
            10)

        

    def find_and_package_synchronized_data(
        self,
        pointcloud_data_pool: List[PointCloud2], 
        camera_image: Image, 
        image_timestamp: timedelta
        ) -> Tuple[PointCloud2, Image]:
        """
        This function finds the 10 closest points (before and after respectively) to `image_timestamp` in `pointcloud_data_pool`,
        repackages the selected points into a new PointCloud2 message, and returns a tuple containing
        the new PointCloud2 and the corresponding camera_image.

        Args:
            pointcloud_data_pool (list[PointCloud2]): A list of PointCloud2 messages representing the point cloud data pool.
            camera_image (Image): The camera image message.
            image_timestamp (timedelta): The timestamp of the camera image.

        Returns:
            tuple[PointCloud2, Image]: A tuple containing the repackaged PointCloud2 message and the corresponding camera_image.
        """
        pass
    
    def print_metadata(self, msg):
        self.get_logger().info(f"PTP mode: {msg.header}")
        print(dir(msg.header))
        # print(msg.get_fields_and_field_types())


    def listener_callback(self, msg):
        timestamp = self.convert_ros_timestamp_to_datetime(msg.header.stamp)
        self.get_logger().info(f"Timestamp: {timestamp}")
        print('\n\nReceived Livox point cloud')
        
        
   
    def convert_ros_timestamp_to_datetime(self, ros_timestamp):
        return datetime.fromtimestamp(ros_timestamp.sec + ros_timestamp.nanosec / 1e9)
        

def main(args=None):
    rclpy.init(args=args)
    livox_point_cloud_subscriber = LivoxPointCloudSubscriber()
    rclpy.spin(livox_point_cloud_subscriber)
    livox_point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()