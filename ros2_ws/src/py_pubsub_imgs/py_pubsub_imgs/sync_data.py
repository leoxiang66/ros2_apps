import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs_py import point_cloud2
import os
from datetime import datetime, timedelta
from typing import List
import std_msgs.msg

    
    



                
    
    
            

class LivoxPointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('livox_point_cloud_subscriber')
        self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.listener_callback,
            10)

        self.points = []
        
        



    def find_and_package_synchronized_data(
        self,
        pointcloud_data_pool: list[PointCloud2], 
        camera_image: Image, 
        image_timestamp: timedelta
        ) -> tuple[PointCloud2, Image]:
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


    def listener_callback(self, msg):
        print('\n\nReceived Livox point cloud')
        
        # 解析点云数据
        if not self.msg_fields:
            self.msg_fields = msg.fields
        points = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity", "tag", "line", "timestamp"))
        
        self.points.append(list(points)) 
        
        # 处理点云数据
        '''
        for point in points:
            x, y, z, intensity, tag, line, timestamp = point
            # 对每个点进行处理
            # 可以进行滤波、变换等操作
            pass
        '''
        
        # new_pc = point_cloud2.create_cloud(header,self.msg_fields,points)
   
        
        
    def creat_PC2msg_header(self):
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"  # 设置适当的帧 ID
        
        return header

def main(args=None):
    rclpy.init(args=args)
    livox_point_cloud_subscriber = LivoxPointCloudSubscriber()
    rclpy.spin(livox_point_cloud_subscriber)
    livox_point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()