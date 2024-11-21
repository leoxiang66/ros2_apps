import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import os
from datetime import datetime
import matplotlib.pyplot as plt

class LivoxImuSubscriber(Node):
    def __init__(self):
        super().__init__('livox_imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/livox/imu',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量的警告
        self.imu_counter = 0


    def listener_callback(self, msg):
        print('\n\nReceived Livox IMU data')
        
        # 解析IMU数据
        timestamp = self.convert_ros_timestamp_to_datetime(msg.header.stamp)
        
        # 获取线性加速度
        linear_acceleration = msg.linear_acceleration
        acc_x = linear_acceleration.x
        acc_y = linear_acceleration.y
        acc_z = linear_acceleration.z
        
        # 获取角速度
        angular_velocity = msg.angular_velocity
        vel_x = angular_velocity.x
        vel_y = angular_velocity.y
        vel_z = angular_velocity.z
        
        # 获取四元数
        orientation = msg.orientation
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        
        self.imu_counter += 1
        
        # 打印IMU数据
        self.print_imu_data(timestamp, acc_x, acc_y, acc_z, vel_x, vel_y, vel_z, qx, qy, qz, qw)
        
    def print_imu_data(self, timestamp, acc_x, acc_y, acc_z, vel_x, vel_y, vel_z, qx, qy, qz, qw):
        self.get_logger().info(f"Timestamp: {timestamp}")
        self.get_logger().info(f"Linear Acceleration: x={acc_x:.4f}, y={acc_y:.4f}, z={acc_z:.4f}")
        self.get_logger().info(f"Angular Velocity: x={vel_x:.4f}, y={vel_y:.4f}, z={vel_z:.4f}")
        self.get_logger().info(f"Orientation: x={qx:.4f}, y={qy:.4f}, z={qz:.4f}, w={qw:.4f}")
        
        print("",end="\r")

    def convert_ros_timestamp_to_datetime(self, ros_timestamp):
        return datetime.fromtimestamp(ros_timestamp.sec + ros_timestamp.nanosec / 1e9)

def main(args=None):
    rclpy.init(args=args)
    livox_imu_subscriber = LivoxImuSubscriber()
    rclpy.spin(livox_imu_subscriber)
    livox_imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()