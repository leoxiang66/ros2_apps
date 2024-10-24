import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import os
from datetime import datetime


class LivoxPointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('livox_point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量的警告
        self.point_cloud_counter = 0
        self.output_dir = 'output_livox_point_clouds'
        os.makedirs(self.output_dir, exist_ok=True)

    def listener_callback(self, msg):
        print('\n\nReceived Livox point cloud')
        
        # 解析点云数据
        points = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity", "tag", "line", "timestamp"))
        
        # 处理点云数据
        for point in points:
            x, y, z, intensity, tag, line, timestamp = point
            # 对每个点进行处理
            # 可以进行滤波、变换等操作
            pass
        
        '''
        # 保存点云数据到文件
        output_path = os.path.join(self.output_dir, f'livox_point_cloud_{self.point_cloud_counter:04d}.pcd')
        with open(output_path, 'w') as f:
            # 写入PCD文件头
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z intensity tag line timestamp\n')
            f.write('SIZE 4 4 4 4 1 1 8\n')
            f.write('TYPE F F F F U U F\n')
            f.write('COUNT 1 1 1 1 1 1 1\n')
            f.write(f'WIDTH {msg.width}\n')
            f.write(f'HEIGHT {msg.height}\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {msg.width * msg.height}\n')
            f.write('DATA ascii\n')
            
            # 写入点云数据
            for point in points:
                x, y, z, intensity, tag, line, timestamp = point
                f.write(f'{x} {y} {z} {intensity} {tag} {line} {timestamp}\n')
        '''       
        
        self.point_cloud_counter += 1
        
        # 打印点云元数据
        self.print_metadata(msg)
        
        self.get_logger().info("Information of the first point:")
        self.print_point_data(points[0])
        
    def print_point_data(self,point):
        x, y, z, intensity, tag, line, timestamp = point
        timestamp_sec = timestamp * 1e-9  # 将纳秒级别的时间戳转换为秒级别
        timestamp_datetime = datetime.fromtimestamp(timestamp_sec)
        self.get_logger().info(f'x,y,z: ({x:.3f},{y:.3f},{z:.3f})')
        self.get_logger().info(f'intensity: {intensity}')
        self.get_logger().info(f'tag: {tag}')
        self.get_logger().info(f'line: {line}')
        self.get_logger().info(f'timestamp: {timestamp_datetime}')
        

    def print_metadata(self, msg):
        timestamp = self.convert_ros_timestamp_to_datetime(msg.header.stamp)
        self.get_logger().info(f"Timestamp: {timestamp}")
        self.get_logger().info(f"Frame ID: {msg.header.frame_id}")
        self.get_logger().info(f"Point cloud width: {msg.width}")
        self.get_logger().info(f"Point cloud height: {msg.height}")
        self.get_logger().info(f"Point step: {msg.point_step}")
        self.get_logger().info(f"Row step: {msg.row_step}")
        self.get_logger().info(f"Is dense: {msg.is_dense}")

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