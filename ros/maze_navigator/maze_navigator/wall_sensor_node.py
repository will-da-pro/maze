import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan


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
        self.publisher_ = None
        self.subscriber_ = None

        self.fov: float = math.pi / 8  # radians
        self.max_range: float = 2.0  # metres
        self.min_range: float = 0.09
 
        self.front_points: list[LaserPoint] = []
        self.left_points: list[LaserPoint] = []
        self.right_points: list[LaserPoint] = []

        self.closest_front: LaserPoint | None = None
        self.closest_left: LaserPoint | None = None
        self.closest_right: LaserPoint | None = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.publisher_ = self.create_publisher(Float64, '/wall_error', 10)
        self.subscriber_ = self.create_subscription(LaserScan, 'scan', 
                                                    self.scan_callback, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating WallSensorNode')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating WallSensorNode')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisher_)
        self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS

    def scan_callback(self, msg):
        
        msg = Float64()
        msg.data = error
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()


