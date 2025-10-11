import math

from maze_msgs.msg import Wall
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class LaserPoint:

    def __init__(self, angle: float, value: float) -> None:
        self.angle: float = angle
        self.value: float = value

    def __str__(self) -> str:
        return f'Angle: {self.angle}, Value: {self.value}'


class CartesianPoint:

    def __init__(self, x: float, y: float) -> None:
        self.x: float = x
        self.y: float = y

    def __str__(self) -> str:
        return f'x: {self.x}, y: {self.y}'


class WallSensorNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__('wall_sensor_node')
        self.left_publisher = None
        self.front_publisher = None
        self.right_publisher = None
        self.error_publisher = None
        self.subscriber_ = None

        self.fov: float = math.pi / 8  # radians
        self.max_range: float = 2.0  # metres
        self.min_range: float = 0.09

        self.target_distance: float = 0.14  # metres

        self.turn_mult: float = 1
        self.dist_mult: float = 10

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring WallSensorNode')

        self.left_publisher = self.create_publisher(Wall, 'left_wall', 10)
        self.front_publisher = self.create_publisher(Wall, 'front_wall', 10)
        self.right_publisher = self.create_publisher(Wall, 'right_wall', 10)
        self.error_publisher = self.create_publisher(Float64, 'maze_error', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating WallSensorNode')

        self.subscriber_ = self.create_subscription(LaserScan, 'scan',
                                                    self.scan_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating WallSensorNode')

        self.destroy_subscription(self.subscriber_)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.left_publisher)
        self.destroy_publisher(self.front_publisher)
        self.destroy_publisher(self.right_publisher)
        self.destroy_publisher(self.error_publisher)

        return TransitionCallbackReturn.SUCCESS

    @staticmethod
    def average_dist(points: list[LaserPoint]) -> float | None:
        if len(points) == 0:
            return None

        total: float = 0
        for point in points:
            total += point.value

        return total / len(points)

    def scan_callback(self, msg):
        front_points: list[LaserPoint] = []
        left_points: list[LaserPoint] = []
        right_points: list[LaserPoint] = []

        closest_front: LaserPoint | None = None
        closest_left: LaserPoint | None = None
        closest_right: LaserPoint | None = None

        for index, value in enumerate(msg.ranges):
            if (value > msg.range_max or value > self.max_range
                    or value < msg.range_min or value < self.min_range):
                continue

            angle = msg.angle_min + index * msg.angle_increment

            if angle > math.pi - self.fov / 2 or angle < -math.pi + self.fov / 2:
                new_point: LaserPoint = LaserPoint(angle, value)
                front_points.append(new_point)

                if closest_front is None or closest_front.value > new_point.value:
                    closest_front = new_point

            if angle > -1/2 * math.pi - self.fov / 2 and angle < -1/2 * math.pi + self.fov:
                new_point: LaserPoint = LaserPoint(angle, value)
                left_points.append(new_point)

                if closest_left is None or closest_left.value > new_point.value:
                    closest_left = new_point

            if angle > 1/2 * math.pi - self.fov / 2 and angle < 1/2 * math.pi + self.fov:
                new_point: LaserPoint = LaserPoint(angle, value)
                right_points.append(new_point)

                if closest_right is None or closest_right.value > new_point.value:
                    closest_right = new_point

        if closest_left is not None:
            left_msg = Wall()
            left_msg.closest_distance = closest_left.value
            left_msg.closest_angle = closest_left.angle
            left_msg.average_distance = self.average_dist(left_points)
            self.left_publisher.publish(left_msg)

            error_angle = (-1/2 * math.pi - closest_left.angle) * self.turn_mult
            error_dist = (self.target_distance - closest_left.value) * self.dist_mult

            #error = (error_angle + error_dist) * (1.5 ** (-error_angle * error_dist))
            error = error_angle + error_dist

            error_msg = Float64()
            error_msg.data = float(error)
            self.error_publisher.publish(error_msg)

        if closest_front is not None:
            front_msg = Wall()
            front_msg.closest_distance = closest_front.value
            front_msg.closest_angle = closest_front.angle
            front_msg.average_distance = self.average_dist(front_points)
            self.front_publisher.publish(front_msg)

        if closest_right is not None:
            right_msg = Wall()
            right_msg.closest_distance = closest_right.value
            right_msg.closest_angle = closest_right.angle
            right_msg.average_distance = self.average_dist(right_points)
            self.right_publisher.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
