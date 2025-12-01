import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import tf_transformations


class InitialPosePub(Node):
    def __init__(self):
        super().__init__('initial_pose_pub')

        # Parameters
        self.declare_parameter('initial_pose.x', 0.0)
        self.declare_parameter('initial_pose.y', 0.0)
        self.declare_parameter('initial_pose.yaw_deg', 0.0)
        self.declare_parameter('publish_topic', '/initialpose')
        self.declare_parameter('publish_delay', 2.0)

        self.pub_topic = self.get_parameter('publish_topic').value
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.pub_topic, 10
        )

        delay = float(self.get_parameter('publish_delay').value)
        self.timer = self.create_timer(delay, self.timer_callback)
        self.published = False

    def timer_callback(self):
        if self.published:
            return

        x = float(self.get_parameter('initial_pose.x').value)
        y = float(self.get_parameter('initial_pose.y').value)
        yaw_deg = float(self.get_parameter('initial_pose.yaw_deg').value)
        yaw_rad = math.radians(yaw_deg)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Populate covariance
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = math.radians(5)**2

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published initial pose: x={x}, y={y}, yaw={yaw_deg}° → topic={self.pub_topic}"
        )

        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

