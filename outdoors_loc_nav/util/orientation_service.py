import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu
import math

class OrientationService(Node):
    """
    A tiny service that returns the robot's current yaw (ENU) in degrees.

    - Reads IMU orientation (quaternion)
    - Converts to yaw in ENU frame
    - Returns yaw_deg via Trigger.response.message
    """

    def __init__(self):
        super().__init__('robot_orientation_service')

        self.latest_yaw_deg = None
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.srv = self.create_service(
            Trigger,
            'robot/get_orientation',
            self.handle_orientation_request
        )

        self.get_logger().info("Orientation service ready: /robot/get_orientation")

    def imu_callback(self, msg: Imu):
        # Convert quaternion → yaw (ENU)
        q = msg.orientation
        # Quaternion → Euler (roll, pitch, yaw)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)  # radians

        self.latest_yaw_deg = math.degrees(yaw)

    def handle_orientation_request(self, request, response):
        if self.latest_yaw_deg is None:
            response.success = False
            response.message = "IMU data not yet received."
        else:
            response.success = True
            response.message = str(self.latest_yaw_deg)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = OrientationService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
