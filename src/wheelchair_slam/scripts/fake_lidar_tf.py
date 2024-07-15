import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg

class StaticTFPublisher(Node):

    def __init__(self):
        super().__init__('static_tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize the static transform
        self.static_transform = geometry_msgs.msg.TransformStamped()
        self.static_transform.header.frame_id = 'base_link'    # Parent frame
        self.static_transform.child_frame_id = 'rslidar'       # Child frame

        # Set the translation to (0, 0, 0)
        self.static_transform.transform.translation.x = 0.0
        self.static_transform.transform.translation.y = 0.0
        self.static_transform.transform.translation.z = 1.657

        # Set the rotation to identity quaternion (0, 0, 0, 1)
        self.static_transform.transform.rotation.x = 0.0
        self.static_transform.transform.rotation.y = 0.0
        self.static_transform.transform.rotation.z = 0.0
        self.static_transform.transform.rotation.w = 1.0

        # Publish the static transform
        self.timer = self.create_timer(1.0, self.publish_tf)  # Publish every 1 second

    def publish_tf(self):
        # Update the timestamp
        self.static_transform.header.stamp = self.get_clock().now().to_msg()

        # Publish the transform
        self.tf_broadcaster.sendTransform(self.static_transform)

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
