import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import math
import serial

class TwistSubscriber(Node):
    START_FLAG: bytes = b'\xA5'

    DRIVE_REQUEST: bytes = b'\x30'
    STOP_REQUEST: bytes = b'\x31'

    def __init__(self):
        super().__init__("twist_subscriber")
        self.subscription = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.twist_callback,
                10
        )

        self.ser = serial.Serial("/dev/ttyAMA0", 115200)

        self.wheel_dist = 0.185 # m
        self.counts_per_revolution = 480
        self.max_counts_per_second = 900
        self.wheel_radius = 0.04
        self.speed_mult = self.counts_per_revolution / (self.wheel_radius * 2 * math.pi)

        self.get_logger().info("Twist subscriber node started!")

    def stop(self):
        self.ser.write(self.START_FLAG + self.STOP_REQUEST)

    def twist_callback(self, msg):
        linear_x = int((msg.linear.x * self.speed_mult * 127) / self.max_counts_per_second)
        angular_z = int((msg.angular.z * self.speed_mult * 127) / self.max_counts_per_second)

        self.get_logger().info(f"linear x: {linear_x}, angular z: {angular_z}")

        if linear_x > 127:
            linear_x = 127

        if linear_x < -127:
            linear_x = -127

        if angular_z > 127:
            angular_z = 127

        if angular_z < -127:
            angular_z = -127

        self.ser.write(self.START_FLAG + self.DRIVE_REQUEST + int(linear_x).to_bytes(1, 'big',  signed=True) + int(angular_z).to_bytes(1, 'big', signed=True))

    def __del__(self):
        self.stop()

def main(args=None):
    rclpy.init(args=args)
    twist_subscriber_node = TwistSubscriber()
    rclpy.spin(twist_subscriber_node)
    twist_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

