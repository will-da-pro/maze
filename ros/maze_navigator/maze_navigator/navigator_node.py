import math
import numpy as np
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class LaserPoint:
    def __init__(self, angle: float, value: float):
        self.angle: float = angle
        self.value: float = value

    def __str__(self) -> str:
        return f"Angle: {self.angle}, Value: {self.value}"

class NavigatorNode(Node):
    def __init__(self) -> None:
        super().__init__('navigator_node')
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.fov: float = math.pi / 8 #radians
        self.range: float = 10.0 #metres
        self.target_distance: float = 0.1 # metres
        self.front_turn_distance: float = 0.2
        self.max_speed: float = 0.25 # ms^-1
        self.default_speed: float = 0.20

        self.current_angle: float = 0
        self.target_angle: float = 0
        self.turning: bool = False
        self.turn_vel: float = 0
        self.turn_mult: float = 0.5
        self.dist_mult: float = 1

        self.x: float = 0
        self.y: float = 0

        self.dx: float = 0
        self.dy: float = 0
        self.dtheta: float = 0

        self.turn_count: int = 0
        self.last_angle: float = 0

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
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
        self.turning = False

    def turn(self, angle: float) -> None:
        self.stop()
        self.target_angle = self.current_angle + angle

        self.turn_vel = self.default_speed if angle > 0 else -self.default_speed

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = float(self.turn_vel)
        self.twist_publisher.publish(msg)

        self.turning = True

    def odom_callback(self, msg) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.dx = msg.twist.twist.linear.x
        self.dy = msg.twist.twist.linear.y
        self.dtheta = msg.twist.twist.angular.z
        
        q = msg.pose.pose.orientation
        new_angle = self.euler_from_quaternion(q.x, q.y, q.z, q.w)[2]

        if new_angle < self.last_angle and self.dtheta > 0:
            self.get_logger().info(f"New Turn: {new_angle}, {self.dtheta}")
            self.turn_count += 1

        elif new_angle > self.last_angle and self.dtheta < 0:
            self.get_logger().info(f"New Turn: {new_angle}, {self.dtheta}")
            self.turn_count -= 1

        self.last_angle = new_angle

        self.current_angle = self.turn_count * 2 * math.pi + new_angle

    def scan_callback(self, msg) -> None:
        start_time = time.time()
        if self.turning:
            self.get_logger().info(f"current: {self.current_angle}, target: {self.target_angle}")
            if self.turn_vel > 0 and self.current_angle > self.target_angle:
                self.stop()
            elif self.turn_vel < 0 and self.current_angle < self.target_angle:
                self.stop()
            elif self.turn_vel == 0:
                self.stop()
            else:
                return

        front_points: list[LaserPoint] = []
        left_points: list[LaserPoint] = []
        right_points: list[LaserPoint] = []

        closest_front: LaserPoint | None = None
        closest_left: LaserPoint | None = None
        closest_right: LaserPoint | None = None
        
        self.get_logger().info(f"Scan Data: min: {msg.angle_min}, max: {msg.angle_max}, increment: {msg.angle_increment}")

        for index, value in enumerate(msg.ranges):
            if value > msg.range_max or value > self.range or value < msg.range_min:
                continue

            angle = msg.angle_min + index * msg.angle_increment

            if angle > math.pi - self.fov / 2  or angle < -math.pi + self.fov / 2:
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

        average_front: float | None = self.average_dist(front_points)
        self.get_logger().info(f"Average front: {average_front}, len: {len(front_points)}, closest: {closest_front}")

        if average_front is not None and average_front < self.front_turn_distance:
            self.turn(math.pi / 2)
            self.get_logger().info("Turning")
            return

        error: float = 0

        if len(left_points) == 0 or closest_left is None:
            self.get_logger().warn("No left points")
            error = -self.max_speed

        else:
            self.get_logger().info(f"Closest left: {closest_left}")
            error = (-1/2 * math.pi - closest_left.angle) * self.turn_mult
            error += (self.target_distance - closest_left.value) * self.dist_mult

        cmd = Twist()
        cmd.linear.x = self.default_speed
        cmd.angular.z = error
        self.twist_publisher.publish(cmd)

        self.get_logger().info(f"Error: {error}")

        end_time = time.time()
        self.get_logger().info(f"Time: {end_time - start_time}")

def main(args=None):
    rclpy.init(args=args)
    maze_navigator_node = NavigatorNode()
    rclpy.spin(maze_navigator_node)
    maze_navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

