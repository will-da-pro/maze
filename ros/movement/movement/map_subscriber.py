import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription  # prevent unused variable warning

    def map_callback(self, msg):
        self.get_logger().info(f'Received map with resolution: {msg.info.resolution} and dimensions: {msg.info.width}x{msg.info.height}')
        # Access map data: msg.data (list of integers representing occupancy)
        # Access map metadata: msg.info (resolution, origin, width, height)

        try:
            for i in range(msg.info.width):
                text = ""
                for j in range(msg.info.height):
                    val = msg.data[msg.info.width * i + j]
                    text += '0' if val == -1 else str(min(int(val / 10), 9))

                self.get_logger().info(text)
        except Exception as _:
            return

def main(args=None):
    rclpy.init(args=args)
    map_subscriber = MapSubscriber()
    rclpy.spin(map_subscriber)
    map_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
