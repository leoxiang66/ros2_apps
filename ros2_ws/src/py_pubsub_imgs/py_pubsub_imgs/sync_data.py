import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from datetime import datetime, timezone, timedelta

def nanosec2date(nanoseconds):
    """
    Convert a nanosecond timestamp to a human-readable date string.
    
    Args:
        nanoseconds (int): Timestamp in nanoseconds since the Unix epoch.
    
    Returns:
        str: Date as a string in ISO 8601 format (YYYY-MM-DD HH:MM:SS.ssssss).
    """
    # Convert nanoseconds to seconds
    seconds = nanoseconds // 1_000_000_000
    # Get the remaining nanoseconds as microseconds
    microseconds = (nanoseconds % 1_000_000_000) // 1_000

    # Create a datetime object from seconds and microseconds
    dt = datetime.fromtimestamp(seconds, tz=timezone.utc) + timedelta(microseconds=microseconds)

    # Return the formatted date string
    return dt.strftime('%Y-%m-%d %H:%M:%S.%f')



# Define the capacity of the timestamp arrays
CAPACITY = 80

class TimestampSyncNode(Node):
    def __init__(self):
        super().__init__('timestamp_sync_node')

        # Arrays to store timestamps
        self.image_timestamps = [0] * CAPACITY
        self.pc2_timestamps = [0] * CAPACITY
        self.image_index = 0
        self.pc2_index = 0
        self.printed = False 

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.pc2_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pc2_callback,
            10
        )

        print('TimestampSyncNode started.')

    def image_callback(self, msg):
        if self.printed:
            return

        # If both arrays are filled, calculate and print differences
        if self.image_index == CAPACITY and self.pc2_index == CAPACITY:
            sum_diff = 0
            for i in range(CAPACITY):
                print(f"image timestamp: {nanosec2date(self.image_timestamps[i])}")
                print(f"pc2 timestamp: {nanosec2date(self.pc2_timestamps[i])}")
                diff = abs(self.image_timestamps[i] - self.pc2_timestamps[i])
                sum_diff += diff
                print(f"diff[{i}]: {diff}")

            average_diff = sum_diff / CAPACITY
            print(f"Average timestamp difference: {average_diff} ns")
            self.printed = True
            return

        elif self.image_index == CAPACITY:
            return

        # Get the timestamp in nanoseconds
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # Save the timestamp in the array
        self.image_timestamps[self.image_index] = timestamp_ns

        # Increment the index
        self.image_index += 1

    def pc2_callback(self, msg):
        if self.pc2_index == CAPACITY:
            return

        # Get the timestamp in nanoseconds
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        

        # Save the timestamp in the array
        self.pc2_timestamps[self.pc2_index] = timestamp_ns

        # Increment the index
        self.pc2_index += 1


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