import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import serial


class TwistSubscriber(Node):

    START_FLAG: bytes = b'\xA5'

    DRIVE_REQUEST: bytes = b'\x30'
    STOP_REQUEST: bytes = b'\x31'

    def __init__(self):
        super().__init__('twist_subscriber')

        self.declare_parameter('wheel_dist', 0.175)
        self.declare_parameter('counts_per_revolution', 480.0)
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('max_counts_per_second', 900.0)

        self.subscription = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.twist_callback,
                10
        )

        self.ser = serial.Serial('/dev/ttyAMA0', 115200)

        self.wheel_dist = self.get_parameter('wheel_dist').value
        self.counts_per_revolution = self.get_parameter('counts_per_revolution').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_counts_per_second = self.get_parameter('max_counts_per_second').value

        self.speed_mult = self.counts_per_revolution / (self.wheel_radius * 2 * math.pi)

        self.get_logger().info('Twist subscriber node started!')

    def stop(self):
        self.ser.write(self.START_FLAG + self.STOP_REQUEST)

    def twist_callback(self, msg):
        linear_x = int((msg.linear.x * self.speed_mult * 127) / self.max_counts_per_second)
        angular_z = int((msg.angular.z * self.speed_mult * 127) / self.max_counts_per_second)

        def fitted(a):
            return min(max(a, -127), 127)

        linear_x = fitted(linear_x)
        angular_z = fitted(angular_z)

        linear_x_byte: bytes = int(linear_x).to_bytes(1, 'big',  signed=True)
        angular_z_byte: bytes = int(angular_z).to_bytes(1, 'big',  signed=True)

        self.ser.write(self.START_FLAG + self.DRIVE_REQUEST
                       + linear_x_byte
                       + angular_z_byte)

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
