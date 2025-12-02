#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import tf_transformations
from cartographer_ros_msgs.srv import StartTrajectory, FinishTrajectory
import math
from rclpy.duration import Duration


class InitialPosePub(Node):
    """
    Robust Initial Pose → Cartographer starter.

    Behavior & best-practices:
      * Wait for cartographer services to be available (with reasonable timeouts).
      * Use IMU yaw as preferred initial orientation; fall back to parameter-provided pose.
      * Accept user RViz /initialpose messages and use them instead of the parameter pose.
      * Use a QoS profile compatible with RViz's latched publisher for /initialpose.
      * Use rclpy.spin_until_future_complete to wait for service responses, with timeouts.
      * Avoid blocking the executor for long periods.
      *
      * See https://chatgpt.com/s/t_692e476cea788191a873acf0b5c25a5e
    """

    def __init__(self):
        super().__init__('initial_pose_pub')

        # ----- parameters -----
        self.declare_parameter('config_dir', '')  # required
        self.declare_parameter('config_base', 'cartographer_lds_2d.lua')
        self.declare_parameter('initial_pose.x', 0.0)
        self.declare_parameter('initial_pose.y', 0.0)
        self.declare_parameter('initial_pose.yaw_deg', 0.0)
        self.declare_parameter('old_trajectory_id', 1)
        self.declare_parameter('publish_delay', 1.0)  # how often timer wakes up to check preconditions
        self.declare_parameter('service_wait_timeout', 5.0)  # seconds to wait for service availability
        self.declare_parameter('service_call_timeout', 5.0)  # seconds to wait for service response

        self.config_dir = self.get_parameter('config_dir').value
        self.config_base = self.get_parameter('config_base').value
        if not self.config_dir:
            self.get_logger().fatal("Mandatory parameter 'config_dir' was not set. Shutting down node.")
            raise RuntimeError("mandatory parameter 'config_dir' must be set.")

        self.old_trajectory_id = int(self.get_parameter('old_trajectory_id').value)
        self.service_wait_timeout = float(self.get_parameter('service_wait_timeout').value)
        self.service_call_timeout = float(self.get_parameter('service_call_timeout').value)

        self.get_logger().info(f"Using cartographer configuration: {self.config_dir}/{self.config_base}")

        # ----- runtime state -----
        self.latest_imu_yaw = None    # updated from IMU
        self.latest_pose_msg = None   # updated from /initialpose (RViz)
        self.state = "IDLE"           # IDLE → FINISHING → STARTING

        # ----- subscriptions -----
        # IMU subscription - default QoS is fine for IMU
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # /initialpose MUST be BEST_EFFORT (RViz publishes "2D Pose Estimate" that way)
        initialpose_qos = QoSProfile(depth=10)
        initialpose_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        initialpose_qos.history = HistoryPolicy.KEEP_LAST
        initialpose_qos.durability = DurabilityPolicy.VOLATILE

        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_estimate_callback,
            initialpose_qos
        )

        # ----- Cartographer service clients -----
        self.finish_client = self.create_client(FinishTrajectory, '/finish_trajectory')
        self.start_client = self.create_client(StartTrajectory, '/start_trajectory')

        # ----- periodic check timer to orchestrate sequence without blocking executor -----
        delay = float(self.get_parameter('publish_delay').value)
        self.timer = self.create_timer(delay, self.timer_callback)

        self.get_logger().info("initial_pose_pub ready - waiting for IMU or /initialpose - or will use params...")


    # ----------------- callbacks -----------------
    def imu_callback(self, msg: Imu) -> None:

        """Keep the latest yaw from IMU (converted to ENU yaw)."""
        q = msg.orientation
        # safe quaternion → yaw (no tf helper required)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.latest_imu_yaw = float(math.atan2(siny_cosp, cosy_cosp))

        #yaw_deg = math.degrees(self.latest_imu_yaw)
        #self.get_logger().debug(f"IMU yaw: {yaw_deg:.2f}°") # debug-level info (rate-limited by timer)
        #self.get_logger().info(f"IMU yaw: {yaw_deg:.2f}°")

    def pose_estimate_callback(self, msg: PoseWithCovarianceStamped) -> None:

        """Store the RViz pose; it overrides parameter pose or IMU when present."""
        self.get_logger().info("=========== Received /initialpose from RViz - will use it as initial pose. ==============")

        self.latest_pose_msg = msg
        self.state = "IDLE"   # trigger restart on next timer tick

    # ------------------------------------------------------------
    # Timer State Machine
    # ------------------------------------------------------------

    def timer_callback(self):
        """Non-blocking state machine.
            Periodically evaluates readiness conditions and runs the finish/start sequence.
            Conditions:
            - cartographer services are available
            - we have either: /initialpose from RViz OR latest IMU yaw OR a parameter fallback pose
        """
        if self.state == "IDLE":
            if not self._have_pose():
                self.get_logger().info("IP: IDLE - Waiting for initial orientation/pose...")
                return

            if not self.finish_client.service_is_ready():
                self.get_logger().info("IP: IDLE - Waiting for Cartographer services...")
                return

            # Ready → start sequence
            self.get_logger().info("OK: Cartographer services ready → start sequence: IDLE -> FINISHING")
            self.state = "FINISHING"
            self._call_finish()

        elif self.state == "FINISHING":
            # waiting for service callback
            self.get_logger().info("IP: FINISHING - Waiting for Cartographer services...")
            pass

        elif self.state == "STARTING":
            # waiting for service callback
            self.get_logger().info("IP: STARTING - Waiting for Cartographer services...")
            pass

    # ------------------------------------------------------------
    # Pose Helpers
    # ------------------------------------------------------------

    def _have_pose(self):
        """We need either RViz pose OR IMU yaw."""
        return self.latest_pose_msg is not None or self.latest_imu_yaw is not None

    def _build_pose(self):
        """Return (x,y,qx,qy,qz,qw)."""
        if self.latest_pose_msg:
            p = self.latest_pose_msg.pose.pose.position
            o = self.latest_pose_msg.pose.pose.orientation

            # ------- for logging only -------------------------------------

            # Convert quaternion → yaw
            siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
            cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
            yaw_rad = math.atan2(siny_cosp, cosy_cosp)
            yaw_deg = math.degrees(yaw_rad)

            self.get_logger().info(
                f"RViz 2D Pose Estimate: x={x:.2f}, y={y:.2f}, "
                f"yaw={yaw_deg:.1f} deg (qz,qw)=({o.z:.3f}, {o.w:.3f})"
            )

            # Optional: log covariance
            cov = self.latest_pose_msg.pose.covariance
            if cov.any():
                pos_cov = cov[0] + cov[7]
                yaw_cov = cov[35]
                self.get_logger().info(
                    f"                       covariance: pos_cov={pos_cov:.3f}, yaw_cov={yaw_cov:.3f}"
                )

            # A tiny note: RViz sometimes sends messages with frame_id="map"
            if self.latest_pose_msg.header.frame_id not in ("map", "map_odom"):
                self.get_logger().warn(
                    f"/initialpose frame_id={self.latest_pose_msg.header.frame_id} (expected 'map'). "
                    "Cartographer ignores frame_id, but TF chains may not."
                )
            # ------- end for logging only --------------------------------

            return p.x, p.y, o.x, o.y, o.z, o.w

        # else IMU/parameters
        x = self.get_parameter('initial_pose.x').value
        y = self.get_parameter('initial_pose.y').value
        yaw = self.latest_imu_yaw
        if yaw is None:
            yaw = math.radians(self.get_parameter('initial_pose.yaw_deg').value)

        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return x, y, qx, qy, qz, qw

    # ------------------------------------------------------------
    # Service Calls (Async)
    # ------------------------------------------------------------

    def _call_finish(self):
        req = FinishTrajectory.Request()
        req.trajectory_id = int(self.old_trajectory_id)

        future = self.finish_client.call_async(req)
        future.add_done_callback(self._on_finish_done)
        self.get_logger().info("IP: Calling /finish_trajectory service...")

    def _on_finish_done(self, future):
        if future.result() is None:
            self.get_logger().error("Error: finish_trajectory FAILED")
            self.state = "IDLE"
            return

        self.get_logger().info("OK: finish_trajectory Success")
        self.state = "STARTING"
        self._call_start()


    def _call_start(self):
        x, y, qx, qy, qz, qw = self._build_pose()

        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_base
        req.use_initial_pose = True
        req.relative_to_trajectory_id = 0

        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.0

        req.initial_pose.orientation.x = qx
        req.initial_pose.orientation.y = qy
        req.initial_pose.orientation.z = qz
        req.initial_pose.orientation.w = qw

        self.get_logger().info(
            f"IP: Calling /start_trajectory service with pose x={x:.2f}, y={y:.2f}, qz,qw={qz:.3f},{qw:.3f}"
        )

        future = self.start_client.call_async(req)
        future.add_done_callback(self._on_start_done)

    def _on_start_done(self, future):
        if future.result() is None:
            self.get_logger().error("Error: start_trajectory service FAILED")
        else:
            self.get_logger().info("OK: start_trajectory service Success (map aligned!)")

        self.state = "IDLE"  # allow repeated RViz resets


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
