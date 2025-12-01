import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations
from cartographer_ros_msgs.srv import StartTrajectory, FinishTrajectory
import math


class InitialPosePub(Node):
    def __init__(self):
        super().__init__('initial_pose_pub')

        # Parameters
        self.declare_parameter('config_dir', '') # mandatory
        self.declare_parameter('config_base', 'cartographer_lds_2d.lua')

        self.config_dir = self.get_parameter('config_dir').value
        self.config_base = self.get_parameter('config_base').value

        if not self.config_dir:
            self.get_logger().fatal("Mandatory parameter 'config_dir' was not set. Shutting down node.")
            raise ValueError("mandatory parameter 'config_dir' must be set.")
        
        self.get_logger().info(f"Using configuration: {self.config_dir}/{self.config_base}")

        self.declare_parameter('initial_pose.x', 0.0)
        self.declare_parameter('initial_pose.y', 0.0)
        self.declare_parameter('initial_pose.yaw_deg', 0.0)
        self.declare_parameter("old_trajectory_id", 1)
        self.declare_parameter('publish_delay', 2.0)

        # Cartographer old trajectory ID to be deleted:
        self.old_trajectory_id = int(self.get_parameter("old_trajectory_id").value)

        # Cartographer trajectory services:
        self.finish_service_str = "/finish_trajectory"
        self.start_service_str = "/start_trajectory"

        # ---- Service Clients ----
        self.finish_client = self.create_client(FinishTrajectory, self.finish_service_str)
        self.start_client = self.create_client(StartTrajectory, self.start_service_str)

        delay = float(self.get_parameter('publish_delay').value)
        self.timer = self.create_timer(delay, self.timer_callback)
        self.published = False

    def timer_callback(self):
        if self.published:
            return

        # Fall-back: pose from initial_pose.yaml
        x = float(self.get_parameter('initial_pose.x').value)
        y = float(self.get_parameter('initial_pose.y').value)
        yaw_deg = float(self.get_parameter('initial_pose.yaw_deg').value)
        yaw_rad = math.radians(yaw_deg)

        # Actual pose query to orientation service:
        # TODO

        # RViz can send PoseWithCovarianceStamped() when you click on Position Estimate:
        # TODO

        # call 
        msg = PoseWithCovarianceStamped()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, yaw_rad)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.get_logger().info(
            f"FYI: initial pose: x={p.x:.2f}, y={p.y:.2f}, yaw(qz,qw)=({q.z:.3f}, {q.w:.3f})"
        )

        #
        # Call Cartographer trajectory services.
        #
        # See https://github.com/cartographer-project/cartographer_ros/issues/1652
        #     https://chatgpt.com/s/t_692dd65951208191b5932e5604b6d75b
        #
        # Start sequence: finish old trajectory → start new one
        self.get_logger().info(
            f"IP:  finish_old_trajectory"
        )

        self.finish_old_trajectory()

        self.get_logger().info(
            f"IP:  start_new_trajectory initial pose: x={x}, y={y}, yaw={yaw_deg}°"
        )

        self.start_new_trajectory(p, q)

        self.get_logger().info(
            f"OK: Finished call initial pose: x={x}, y={y}, yaw={yaw_deg}°"
        )

        self.published = True

    # ------------------------------------------
    # ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
    # waiting for service to become available...
    # requester: making request: cartographer_ros_msgs.srv.FinishTrajectory_Request(trajectory_id=0)
    # response:
    # cartographer_ros_msgs.srv.FinishTrajectory_Response(status=cartographer_ros_msgs.msg.StatusResponse(code=0, message='Finished trajectory 0.'))

    def finish_old_trajectory(self):
        self.get_logger().info(f"IP: Waiting for {self.finish_service_str} service...")

        self.finish_client.wait_for_service()

        self.get_logger().info(f"OK: {self.finish_service_str} service is up, calling...")

        request = FinishTrajectory.Request()
        request.trajectory_id = self.old_trajectory_id  # trajectory to be deleted ("finished"), usually 0

        future = self.finish_client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            self.get_logger().info(f"OK: Finished trajectory {self.old_trajectory_id} successfully.")
        else:
            self.get_logger().error(f"Error: Failed to call {self.finish_service_str} service")

    # ------------------------------------------
    # ros2 service call /start_trajectory cartographer_ros_msgs/srv/StartTrajectory \
    # "{ configuration_directory: '/home/sergei/robot_ws/src/outdoors_loc_nav/outdoors_loc_nav/params' \
    # , configuration_basename: 'cartographer_lds_2d.lua' \
    # , use_initial_pose: True, initial_pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.259, w: 0.966}}}"
    # waiting for service to become available...
    # requester: making request:
    #    cartographer_ros_msgs.srv.StartTrajectory_Request(configuration_directory='/home/sergei/robot_ws/src/outdoors_loc_nav/outdoors_loc_nav/params',
    #                                                      configuration_basename='cartographer_lds_2d.lua', use_initial_pose=True,
    #                                                      initial_pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.0, y=2.0, z=0.0),
    #                                                      orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.259, w=0.966)), relative_to_trajectory_id=0)
    # response:
    # cartographer_ros_msgs.srv.StartTrajectory_Response(status=cartographer_ros_msgs.msg.StatusResponse(code=0, message='Success.'), trajectory_id=1)

    def start_new_trajectory(self, pos, ori):
        self.get_logger().info(f"IP: Waiting for {self.start_service_str} service...")

        self.start_client.wait_for_service()

        self.get_logger().info(f"OK: {self.start_service_str} service is up, calling...")

        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_base
        req.use_initial_pose = True
        req.relative_to_trajectory_id = 0

        req.initial_pose.position.x = pos.x
        req.initial_pose.position.y = pos.y
        req.initial_pose.position.z = pos.z

        req.initial_pose.orientation.x = ori.x
        req.initial_pose.orientation.y = ori.y
        req.initial_pose.orientation.z = ori.z
        req.initial_pose.orientation.w = ori.w

        future = self.start_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            self.get_logger().info("OK: Started new Cartographer trajectory with initial pose!")
        else:
            self.get_logger().error(f"Error: Failed to call {self.start_service_str} service")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

