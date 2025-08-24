import functools
import math
import sys

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from gpiozero import LED
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


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


class NavigatorNode(Node):

    def __init__(self) -> None:
        super().__init__('navigator_node')
        self.scan_callback_group = ReentrantCallbackGroup()
        self.odom_callback_group = ReentrantCallbackGroup()
        self.image_callback_group = ReentrantCallbackGroup()

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10,
                                                 callback_group=self.scan_callback_group)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10,
                                                 callback_group=self.odom_callback_group)
        self.image_sub = self.create_subscription(Image, 'camera_node/image_raw',
                                                  self.image_callback, 10,
                                                  callback_group=self.image_callback_group)
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.fov: float = math.pi / 8  # radians
        self.max_range: float = 2.0  # metres
        self.min_range: float = 0.09
        self.target_distance: float = 0.14  # metres
        self.front_turn_distance: float = 0.2
        self.max_speed: float = 0.25  # ms^-1
        self.default_speed: float = 0.20

        self.current_angle: float = 0
        self.target_angle: float = 0
        self.navigate: bool = True
        self.turn_mult: float = 0.5
        self.dist_mult: float = 1

        self.x: float = 0
        self.y: float = 0

        self.dx: float = 0
        self.dy: float = 0
        self.dtheta: float = 0

        self.turn_count: int = 0
        self.last_angle: float = 0

        self.min_green: float = 0.1
        self.min_red: float = 0.1
        self.min_black: float = 0.5
        self.min_silver: float = 0.75

        self.front_points: list[LaserPoint] = []
        self.left_points: list[LaserPoint] = []
        self.right_points: list[LaserPoint] = []

        self.closest_front: LaserPoint | None = None
        self.closest_left: LaserPoint | None = None
        self.closest_right: LaserPoint | None = None

        self.green_squares: list[CartesianPoint] = []
        self.red_squares: list[CartesianPoint] = []

        self.min_square_dist: float = 0.15

        self.red_led = LED(10)
        self.green_led = LED(11)

    def image_callback(self, msg):
        if not self.navigate:
            return

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        green_mask = cv2.inRange(hsv_image, (36, 25, 25), (70, 255, 255))
        red_mask = (cv2.inRange(hsv_image, (0, 100, 100), (10, 255, 255))
                    | cv2.inRange(hsv_image, (160, 100, 100), (179, 255, 255)))
        black_mask = cv2.inRange(hsv_image, (0, 0, 0), (179, 255, 40))
        silver_mask = cv2.inRange(hsv_image, (0, 0, 100), (179, 30, 140))

        n_g = cv2.countNonZero(green_mask)
        n_r = cv2.countNonZero(red_mask)
        n_b = cv2.countNonZero(black_mask)
        n_s = cv2.countNonZero(silver_mask)

        area: int = msg.width * msg.height

        if n_b / area > self.min_black:
            self.get_logger().info('Black')
            self.straight(self, -0.2)

            right_average = self.average_dist(self.right_points)

            if right_average > self.front_turn_distance:
                self.turn(self, math.pi / 2)

            else:
                self.turn(self, math.pi)

            return

        valid_green: bool = True

        for point in self.green_squares:
            if (math.sqrt((point.x - self.x) ** 2 + (point.y - self.y) ** 2)
                    < self.min_square_dist):
                valid_green = False
                break

        valid_red: bool = True

        for point in self.red_squares:
            if (math.sqrt((point.x - self.x) ** 2 + (point.y - self.y) ** 2)
                    < self.min_square_dist):
                valid_red = False
                break

        if n_g / area > self.min_green and valid_green:
            self.get_logger().info('Green')
            self.navigate = False
            self.stop()

            self.green_squares.append(CartesianPoint(self.x, self.y))

            sleep_time = 2

            rate = self.create_rate(1 / sleep_time)
            rate.sleep()

            self.navigate = True

        if n_r / area > self.min_red and valid_red:
            self.get_logger().info('Red')
            self.navigate = False
            self.stop()

            self.red_squares.append(CartesianPoint(self.x, self.y))

            sleep_time = 2

            rate = self.create_rate(1 / sleep_time)
            rate.sleep()

            self.navigate = True

        green_count = len(self.green_squares)
        red_count = len(self.red_squares)

        if n_s / area > self.min_silver and green_count + red_count >= 4:
            self.get_logger().info('Silver')
            self.navigate = False
            self.stop()

            self.display_victims()

        self.get_logger().info(f'green: {n_g}, red: {n_r}, black: {n_b}, silver: {n_s}')

    def display_victims(self):
        sleep_time = 1
        rate = self.create_rate(1 / sleep_time)

        for _ in self.green_squares:
            self.green_led.on()
            rate.sleep()
            self.green_led.off()
            rate.sleep()

        for _ in self.red_squares:
            self.red_led.on()
            rate.sleep()
            self.red_led.off()
            rate.sleep()

        sys.exit()

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """Convert quaternion to Euler.

        Convert a quaternion into Euler angles (roll, pitch, yaw).
        @param x: x component of quaternion
        @param y: y component of quaternion
        @param z: z component of quaternion
        @param w: w component of quaternion
        @return: (roll_x, pitch_y, yaw_z) Euler angles in radians
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def navigation(func):
        @functools.wraps(func)
        def wrapper(self, *args, **kwargs):
            self.navigate = False
            self.stop()

            result = func(*args, **kwargs)

            self.stop()
            self.navigate = True

            return result
        return wrapper

    @staticmethod
    def average_dist(points: list[LaserPoint]) -> float | None:
        if len(points) == 0:
            return None

        total: float = 0
        for point in points:
            total += point.value

        return total / len(points)

    def stop(self) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

    @navigation
    def straight(self, dist: float) -> None:
        target_x = dist * math.cos(self.current_angle) + self.x

        msg = Twist()
        msg.linear.x = self.default_speed if dist > 0 else -self.default_speed
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

        direction = 1 if target_x > self.x else -1

        moving: bool = True

        while moving:
            self.get_logger().info(f'dir: {direction}, target: {target_x}, x: {self.x}')
            if direction > 0 and self.x > target_x:
                moving = False

            elif direction < 0 and self.x < target_x:
                moving = False

            elif direction == 0 or dist == 0:
                moving = False

    @navigation
    def turn(self, angle: float) -> None:
        self.target_angle = self.current_angle + angle

        turn_vel = self.default_speed if angle > 0 else -self.default_speed

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = float(turn_vel)
        self.twist_publisher.publish(msg)

        turning: bool = True

        while turning:
            if turn_vel > 0 and self.current_angle > self.target_angle:
                turning = False

            elif turn_vel < 0 and self.current_angle < self.target_angle:
                turning = False

            elif turn_vel == 0:
                turning = False

    def odom_callback(self, msg) -> None:
        self.get_logger().info('Odom')
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.dx = msg.twist.twist.linear.x
        self.dy = msg.twist.twist.linear.y
        self.dtheta = msg.twist.twist.angular.z

        q = msg.pose.pose.orientation
        new_angle = self.euler_from_quaternion(q.x, q.y, q.z, q.w)[2]

        if new_angle < self.last_angle and self.dtheta > 0:
            self.turn_count += 1

        elif new_angle > self.last_angle and self.dtheta < 0:
            self.turn_count -= 1

        self.last_angle = new_angle

        self.current_angle = self.turn_count * 2 * math.pi + new_angle

    def scan_callback(self, msg) -> None:
        if not self.navigate:
            return

        self.front_points = []
        self.left_points = []
        self.right_points = []

        self.closest_front = None
        self.closest_left = None
        self.closest_right = None

        for index, value in enumerate(msg.ranges):
            if (value > msg.range_max or value > self.max_range
                    or value < msg.range_min or value < self.min_range):
                continue

            angle = msg.angle_min + index * msg.angle_increment

            if angle > math.pi - self.fov / 2 or angle < -math.pi + self.fov / 2:
                new_point: LaserPoint = LaserPoint(angle, value)
                self.front_points.append(new_point)

                if self.closest_front is None or self.closest_front.value > new_point.value:
                    self.closest_front = new_point

            if angle > -1/2 * math.pi - self.fov / 2 and angle < -1/2 * math.pi + self.fov:
                new_point: LaserPoint = LaserPoint(angle, value)
                self.left_points.append(new_point)

                if self.closest_left is None or self.closest_left.value > new_point.value:
                    self.closest_left = new_point

            if angle > 1/2 * math.pi - self.fov / 2 and angle < 1/2 * math.pi + self.fov:
                new_point: LaserPoint = LaserPoint(angle, value)
                self.right_points.append(new_point)

                if self.closest_right is None or self.closest_right.value > new_point.value:
                    self.closest_right = new_point

        self.drive()

    def drive(self) -> None:
        if not self.navigate:
            return

        average_front: float | None = self.average_dist(self.front_points)

        if average_front is not None and average_front < self.front_turn_distance:
            self.turn(self, math.pi / 2)
            return

        error: float = 0

        if len(self.left_points) == 0 or self.closest_left is None:
            self.get_logger().warn('No left points')
            error = -self.max_speed

        else:
            error = (-1/2 * math.pi - self.closest_left.angle) * self.turn_mult
            error += (self.target_distance - self.closest_left.value) * self.dist_mult

        cmd = Twist()
        cmd.linear.x = self.default_speed
        cmd.angular.z = error
        self.twist_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    maze_navigator_node = NavigatorNode()
    executor = MultiThreadedExecutor()  # Or adjust num_threads as needed
    executor.add_node(maze_navigator_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        maze_navigator_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
