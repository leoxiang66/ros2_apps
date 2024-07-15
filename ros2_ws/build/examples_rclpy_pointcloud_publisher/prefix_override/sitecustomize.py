import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xiang-tao/git/ros2_apps/ros2_ws/install/examples_rclpy_pointcloud_publisher'
