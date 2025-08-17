import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.twist_pulisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def scan_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    maze_navigator_node = NavigatorNode()
    rclpy.spin(maze_navigator_node)
    maze_navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

