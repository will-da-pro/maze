import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import serial

class TwistSubscriber(Node):
    def __init__(self):
        super().__init__("twist_subscriber")
        self.subscription = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.twist_callback,
                10
        )

        self.get_logger().info("Twist subscriber node started!")

    def twist_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info(f'Received Twist: Linear X={linear_x}, Angular Z={angular_z}')

def main(args=None):
    rclpy.init(args=args)
    twist_subscriber_node = TwistSubscriber()
    rclpy.spin(twist_subscriber_node)
    twist_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

