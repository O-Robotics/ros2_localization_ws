import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
import numpy as np

class IMUOrientationZeroer(Node):
    def __init__(self):
        super().__init__('imu_orientation_zeroer')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            50  # buffer size
        )
        self.publisher_ = self.create_publisher(Imu, '/imu/data_aligned', 10)

        self.buffer = []
        self.offset_rotation = None
        self.collecting = True
        self.max_samples = 30  # number of samples to average
        self.get_logger().info("Collecting initial IMU samples to compute orientation offset...")

    def imu_callback(self, msg):
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        if self.collecting:
            self.buffer.append(quat)
            if len(self.buffer) >= self.max_samples:
                self.compute_offset()
                self.collecting = False
                self.get_logger().info("Orientation offset computed.")
            return

        if self.offset_rotation is None:
            return

        current_rotation = R.from_quat(quat)
        corrected_rotation = self.offset_rotation * current_rotation
        corrected_quat = corrected_rotation.as_quat()

        # Republish corrected IMU message
        new_msg = Imu()
        new_msg.header = Header()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = msg.header.frame_id
        new_msg.orientation.x = corrected_quat[0]
        new_msg.orientation.y = corrected_quat[1]
        new_msg.orientation.z = corrected_quat[2]
        new_msg.orientation.w = corrected_quat[3]
        new_msg.orientation_covariance = msg.orientation_covariance
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        new_msg.linear_acceleration = msg.linear_acceleration
        new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.publisher_.publish(new_msg)

    def compute_offset(self):
        rotations = R.from_quat(np.array(self.buffer))
        avg_rotation = rotations.mean()
        self.offset_rotation = avg_rotation.inv()


def main(args=None):
    rclpy.init(args=args)
    node = IMUOrientationZeroer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()