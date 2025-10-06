import functools
import math
import sys

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from gpiozero import LED
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from maze_msgs.msg import Wall, Victims
from enum import Enum
import time

class State(Enum):
    INIT = 0
    NAVIGATING = 1
    REVERSING_FROM_WALL = 2
    TURNING_FROM_WALL = 3
    IDENTIFYING_VICTIM = 4
    REVERSING_FROM_HOLE = 5
    TURNING_FROM_HOLE = 6
    DISPLAYING_VICTIMS = 7
    STOP = 8

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

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.front_wall_sub = self.create_subscription(Wall, 'front_wall', self.front_wall_callback, 10)
        self.error_sub = self.create_subscription(Float64, 'maze_error', self.error_callback, 10)

        self.victims_sub = self.create_subscription(Victims, 'victims', self.victims_callback, 10)
        self.hole_sub = self.create_subscription(Bool, 'hole_visible', self.hole_callback, 10)
        self.exit_sub = self.create_subscription(Bool, 'exit_visible', self.exit_callback, 10)

        self.wall_sensor_client = self.create_client(ChangeState, '/wall_sensor_node/change_state')
    
        while not self.wall_sensor_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Wall sensor service not available, reconnecting...")

        self.camera_subscriber_client = self.create_client(ChangeState, '/camera_subscriber_node/change_state')

        while not self.camera_subscriber_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Camera subscriber service not available, reconnecting...")

        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.state_loop)
        
        self.current_state = State.INIT

        self.front_turn_distance: float = 0.16
        self.max_speed: float = 0.30  # ms^-1
        self.default_speed: float = 0.30

        self.error: float = 0

        self.current_angle: float = 0
        self.target_angle: float = 0

        self.x: float = 0
        self.y: float = 0

        self.dx: float = 0
        self.dy: float = 0
        self.dtheta: float = 0

        self.straight_start_pos = (0, 0)
        self.turn_start_angle = 0

        self.front_avg: float | None = None
        self.error: float = 0

        self.green_victim: bool = False
        self.red_victim: bool = False

        self.hole: bool = False
        self.exit: bool = False

        self.turn_count: int = 0
        self.last_angle: float = 0

        self.green_victims: list[CartesianPoint] = []
        self.red_victims: list[CartesianPoint] = []

        self.min_square_dist: float = 0.15

        self.red_led = LED(10)
        self.green_led = LED(11)

    def change_node_state(self, client, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)

    def display_victims(self):
        sleep_time = 1
        rate = self.create_rate(1 / sleep_time)

        for _ in self.green_victims:
            self.green_led.on()
            rate.sleep()
            self.green_led.off()
            rate.sleep()

        for _ in self.red_victims:
            self.red_led.on()
            rate.sleep()
            self.red_led.off()
            rate.sleep()

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

    def stop(self) -> None:
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

    def straight(self, reverse: bool = False) -> None:
        self.straight_start_pos = (self.x, self.y)

        msg = Twist()
        msg.linear.x = self.default_speed if not reverse else -self.default_speed
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)
    
    def check_straight_complete(self, dist: float) -> None:
        displacement = math.sqrt((self.x - self.straight_start_pos[0]) ** 2 + (self.y - self.straight_start_pos[1]) ** 2)
        
        if displacement > dist:
            return True

        return False

    def turn(self, clockwise: bool = True) -> None:
        self.turn_start_angle = self.current_angle

        turn_vel = self.default_speed if clockwise else -self.default_speed

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = float(turn_vel)
        self.twist_publisher.publish(msg)

    def check_turn_complete(self, angle: float) -> bool:
        if angle > 0 and self.current_angle - self.turn_start_angle > angle:
            return True

        elif angle < 0 and self.current_angle - self.turn_start_angle < angle:
            return True

        return False

    def odom_callback(self, msg) -> None:
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

    def front_wall_callback(self, msg) -> None:
        self.front_avg = msg.average_distance

    def error_callback(self, msg) -> None:
        self.error = msg.data
        
    def victims_callback(self, msg) -> None:
        self.red_victim = msg.red_victim
        self.green_victim = msg.green_victim

    def hole_callback(self, msg) -> None:
        self.hole = msg.data

    def exit_callback(self, msg) -> None:
        self.exit = msg.data

    def state_loop(self) -> None:
        self.get_logger().info(f'front average: {self.front_avg}')
        if self.current_state == State.INIT:
            self.get_logger().info("Configuring Wall Sensor")
            self.change_node_state(self.wall_sensor_client, Transition.TRANSITION_CONFIGURE)

            self.get_logger().info("Configuring Camera Subscriber")
            self.change_node_state(self.camera_subscriber_client, Transition.TRANSITION_CONFIGURE) 

            self.get_logger().info("Activating Wall Sensor")
            self.change_node_state(self.wall_sensor_client, Transition.TRANSITION_ACTIVATE)

            self.get_logger().info("Activating Camera Subscriber")
            self.change_node_state(self.camera_subscriber_client, Transition.TRANSITION_ACTIVATE) 

            self.current_state = State.NAVIGATING

        elif self.current_state == State.NAVIGATING:
            if self.front_avg is not None and self.front_avg <= self.front_turn_distance:
                self.stop()

                #self.change_node_state(self.camera_subscriber_client, Transition.TRANSITION_DEACTIVATE)

                self.straight(reverse=True)

                self.current_state = State.REVERSING_FROM_WALL

                return

            if self.red_victim:
                self.stop()

                valid_red: bool = True

                for point in self.red_victims:
                    if (math.sqrt((point.x - self.x) ** 2 + (point.y - self.y) ** 2)
                            < self.min_square_dist):
                        valid_red = False
                        break

                if valid_red:
                    self.red_victims.append(CartesianPoint(self.x, self.y))

                    #self.change_node_state(self.camera_subscriber_client, Transition.TRANSITION_DEACTIVATE)
                    #self.change_node_state(self.wall_sensor_client, Transition.TRANSITION_DEACTIVATE)

                    self.red_led.on()

                    self.wait_start_time = time.time()

                    self.current_state = State.IDENTIFYING_VICTIM

                    return

            if self.green_victim:
                self.stop()

                valid_green: bool = True

                for point in self.green_victims:
                    if (math.sqrt((point.x - self.x) ** 2 + (point.y - self.y) ** 2)
                            < self.min_square_dist):
                        valid_green = False
                        break

                if valid_green:
                    self.green_victims.append(CartesianPoint(self.x, self.y))

                    #self.change_node_state(self.camera_subscriber_client, Transition.TRANSITION_DEACTIVATE)
                    #self.change_node_state(self.wall_sensor_client, Transition.TRANSITION_DEACTIVATE)

                    self.green_led.on()

                    self.wait_start_time = time.time()

                    self.current_state = State.IDENTIFYING_VICTIM

                    return

            if self.hole:
                self.stop()
                
                self.straight(reverse=True)
                
                self.current_state = State.REVERSING_FROM_HOLE

                return

            if self.exit:
                self.stop()
                self.current_state = State.DISPLAYING_VICTIMS
                return

            msg = Twist()
            msg.linear.x = float(self.default_speed)
            msg.angular.z = float(self.error)
            self.twist_publisher.publish(msg)

        elif self.current_state == State.REVERSING_FROM_WALL:
            if not self.check_straight_complete(0.02):
                return

            self.stop()

            self.turn()
            self.current_state = State.TURNING_FROM_WALL

        elif self.current_state == State.TURNING_FROM_WALL:
            if not self.check_turn_complete(math.pi / 2):
                return

            self.stop()

            self.current_state = State.NAVIGATING

        elif self.current_state == State.IDENTIFYING_VICTIM:
            if time.time() - self.wait_start_time < 2:
                return

            self.green_led.off()
            self.red_led.off()

            self.current_state = State.NAVIGATING

        elif self.current_state == State.REVERSING_FROM_HOLE:
            if not self.check_straight_complete(0.15):
                return

            self.stop()

            self.turn()

            self.current_state = State.TURNING_FROM_HOLE

        elif self.current_state == State.TURNING_FROM_HOLE:
            if not self.check_turn_complete(math.pi / 2):
                return

            self.stop()

            self.current_state = State.NAVIGATING

        elif self.current_state == State.DISPLAYING_VICTIMS:
            robot.stop()
            self.display_victims()
            self.current_state = State.STOP

        elif self.current_state == State.STOP:
            robot.stop()
            sys.exit()
        
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
