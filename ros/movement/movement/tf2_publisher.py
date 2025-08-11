import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class TfBroadcasterNode(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform) # Adjust rate as needed

    def publish_transform(self):
        t = TransformStamped()

        # Set the header information
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'  # Parent frame
        t.child_frame_id = 'robot_base' # Child frame

        # Set the translation (x, y, z)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set the rotation (quaternion) - example for a static rotation
        # For dynamic rotations, you might calculate based on time or other sensor data
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0 # Identity quaternion (no rotation)

        # You can also set dynamic transforms based on time or other data
        # For example, a rotating frame:
        # angle = self.get_clock().now().nanoseconds / 1e9 * 0.1 # Rotate at 0.1 rad/s
        # q = quaternion_from_euler(0, 0, angle) # You'd need to import 'tf_transformations'
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcasterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
