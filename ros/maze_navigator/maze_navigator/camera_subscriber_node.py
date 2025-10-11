import cv2
from cv_bridge import CvBridge
from maze_msgs.msg import Victims
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class CameraSubscriberNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__('camera_subscriber_node')
        self.victim_publisher = None
        self.hole_publisher = None
        self.exit_publisher = None
        self.subscriber_ = None

        self.min_green: float = 0.01
        self.min_red: float = 0.1
        self.min_black: float = 0.5
        self.min_silver: float = 0.1

        self.green_lower = (42, 130, 130)
        self.green_upper = (70, 255, 255)
        self.red_lower_1 = (0, 160, 150)
        self.red_upper_1 = (7, 255, 255)
        self.red_lower_2 = (160, 160, 150)
        self.red_upper_2 = (179, 255, 255)
        self.black_lower = (0, 0, 0)
        self.black_upper = (179, 150, 30)
        self.silver_lower = (0, 0, 240)
        self.silver_upper = (179, 30, 255)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring CameraSubscriberNode')

        self.victim_publisher = self.create_publisher(Victims, 'victims', 10)
        self.hole_publisher = self.create_publisher(Bool, 'hole_visible', 10)
        self.exit_publisher = self.create_publisher(Bool, 'exit_visible', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating CameraSubscriberNode')

        self.subscriber_ = self.create_subscription(Image, 'camera/image_raw',
                                                    self.image_callback, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating CameraSubscriberNode')

        self.destroy_subscription(self.subscriber_)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.victim_publisher)
        self.destroy_publisher(self.hole_publisher)
        self.destroy_publisher(self.exit_publisher)

        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        green_mask = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
        red_mask = (cv2.inRange(hsv_image, self.red_lower_1, self.red_upper_1)
                    | cv2.inRange(hsv_image, self.red_lower_2, self.red_upper_2))
        black_mask = cv2.inRange(hsv_image, self.black_lower, self.black_upper)
        silver_mask = cv2.inRange(hsv_image, self.silver_lower, self.silver_upper)

        num_green = cv2.countNonZero(green_mask)
        num_red = cv2.countNonZero(red_mask)
        num_black = cv2.countNonZero(black_mask)
        num_silver = cv2.countNonZero(silver_mask)

        #self.get_logger().info(f"Green: {num_green}, Red: {num_red}, Black: {num_black}, Silver: {num_silver}")

        total_area: int = msg.width * msg.height

        green_frac = num_green / total_area
        red_frac = num_red / total_area
        black_frac = num_black / total_area
        silver_frac = num_silver / total_area

        victims = Victims()
        victims.green_victim = (green_frac >= self.min_green)
        victims.red_victim = (red_frac >= self.min_red)
        self.victim_publisher.publish(victims)

        hole_visible = Bool()
        hole_visible.data = (black_frac >= self.min_black)
        self.hole_publisher.publish(hole_visible)

        exit_visible = Bool()
        exit_visible.data = (silver_frac >= self.min_silver)
        self.exit_publisher.publish(exit_visible)


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()
