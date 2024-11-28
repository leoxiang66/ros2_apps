import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import os
from datetime import datetime
import matplotlib.pyplot as plt

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
        self.offsets = []

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
        
        self.point_cloud_counter += 1
        
        # 打印点云元数据
        # print(f"There are {len(points)} points in this PC2.")
        self.print_metadata(msg)
        
        self.get_logger().info("Information of the first point:")
        self.print_point_data(points[0])
        
        offset = msg.header.stamp.nanosec // 1000 
        offset = offset - 1000000 if offset > 500000 else offset
        
        self.offsets.append(offset)
        
        if len(self.offsets) == 100:
            self.plot_offsets()
        
    def print_point_data(self, point):
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
    
    def plot_offsets(self):
        x = list(range(100))
        y = self.offsets
        
        plt.figure(figsize=(20, 6))
        plt.plot(x, y, linestyle='--', marker='o', color='b')
        plt.xlabel('Index')
        plt.ylabel('Offset (microseconds)')
        plt.title('Offsets Line Plot')
        plt.grid(True)
        plt.tight_layout()
        
        # 将图片保存到文件
        output_path = os.path.join(self.output_dir, f'offsets_line_plot_{self.point_cloud_counter:04d}.png')
        plt.savefig(output_path)
        
        plt.close()  # 关闭图形窗口
        
        self.offsets = []  # 清空offsets列表,重新开始收集

def main(args=None):
    rclpy.init(args=args)
    livox_point_cloud_subscriber = LivoxPointCloudSubscriber()
    rclpy.spin(livox_point_cloud_subscriber)
    livox_point_cloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()