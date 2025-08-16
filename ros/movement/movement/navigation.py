import numpy as np
import rclpy
from rclpy import node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import OccupancyGrid
from builtin_interfaces.msg import Time

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscriber(OccupancyGrid, 'map', self.map_callback, 10)

        self.map = OccupancyGrid()

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.navigate)

    def map_callback(self, msg):
        pass

    def navigate(self):
        pass
