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
        self.latest_yaw_rad = None    # updated from IMU
        self.latest_pose_msg = None   # updated from /initialpose (RViz)
        self.published = False        # when True we finished start sequence

        # ----- subscriptions -----
        # IMU subscription - default QoS is fine for IMU
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # /initialpose (RViz) is typically published with transient_local (latched) QoS.
        # Use compatibility: RELIABLE + TRANSIENT_LOCAL + reasonable depth.
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

        # ----- timer to orchestrate sequence without blocking executor -----
        delay = float(self.get_parameter('publish_delay').value)
        self.timer = self.create_timer(delay, self.timer_callback)

        self.get_logger().info("initial_pose_pub ready - waiting for IMU and /initialpose or params...")


    # ----------------- callbacks -----------------
    def imu_callback(self, msg: Imu) -> None:
        """Keep the latest yaw from IMU (converted to ENU yaw)."""
        if self.published:
            return

        q = msg.orientation
        # safe quaternion → yaw (no tf helper required)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.latest_yaw_rad = float(yaw)
        self.latest_yaw_deg = math.degrees(self.latest_yaw_rad)

        # debug-level info (rate-limited by timer)
        # self.get_logger().debug(f"IMU yaw: {self.latest_yaw_deg:.2f}°")
        #self.get_logger().info(f"IMU yaw: {self.latest_yaw_deg:.2f}°")

    def pose_estimate_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Store the RViz pose; it overrides parameter pose when present."""
        self.get_logger().info("=========== Received /initialpose from RViz - will use it as initial pose. ==============")
        self.latest_pose_msg = msg
        # ensure we (re)try start sequence
        self.published = False

    def pose_estimate_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """
        Handle RViz '2D Pose Estimate' messages.
        This overrides IMU-derived orientation and the parameter-based initial pose.

        RViz publishes PoseWithCovarianceStamped on /initialpose.
        """
        self.get_logger().info("=========== Received /initialpose from RViz — using it as initial pose for Cartographer ==============")

        # --- Extract x, y --------------------------------------------------------
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # --- Extract yaw from quaternion -----------------------------------------
        q = msg.pose.pose.orientation

        # Convert quaternion → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)

        self.get_logger().info(
            f"RViz 2D Pose Estimate: x={x:.2f}, y={y:.2f}, "
            f"yaw={yaw_deg:.1f} deg (qz,qw)=({q.z:.3f}, {q.w:.3f})"
        )

        # Store for later use (start_new_trajectory will prioritize this)
        self.latest_pose_msg = msg
        self.latest_yaw_rad = yaw_rad
        self.latest_yaw_deg = yaw_deg

        # --- Optional: log covariance -------------------------------------------
        cov = msg.pose.covariance
        if cov.any():
            pos_cov = cov[0] + cov[7]
            yaw_cov = cov[35]
            self.get_logger().info(
                f"                       covariance: pos_cov={pos_cov:.3f}, yaw_cov={yaw_cov:.3f}"
            )

        # --- Tell the node to restart the start-trajectory sequence -------------
        self.published = False

        # A tiny note: RViz sometimes sends messages with frame_id="map"
        if msg.header.frame_id not in ("map", "map_odom"):
            self.get_logger().warn(
                f"/initialpose frame_id={msg.header.frame_id} (expected 'map'). "
                "Cartographer ignores frame_id, but TF chains may not."
            )

        return

    # ----------------- orchestration -----------------
    def timer_callback(self) -> None:
        """
        Periodically evaluate readiness conditions and run the finish/start sequence exactly once.
        Conditions:
         - cartographer services are available
         - we have either: /initialpose OR latest IMU yaw OR parameter fallback pose
        """

        if self.published:
            return

        # Prefer a user-specified pose from RViz (latest_pose_msg),
        # otherwise prefer IMU yaw + parameter xy, otherwise parameter-only.
        use_pose = None
        use_x = None
        use_y = None
        use_yaw_rad = None

        if self.latest_pose_msg is not None:
            pm = self.latest_pose_msg
            use_pose = pm
            use_x = pm.pose.pose.position.x
            use_y = pm.pose.pose.position.y
            # yaw from incoming quaternion:
            q = pm.pose.pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            use_yaw_rad = float(yaw)
            self.get_logger().info("Using RViz /initialpose for start.")
        elif self.latest_yaw_rad is not None:
            use_x = float(self.get_parameter('initial_pose.x').value)
            use_y = float(self.get_parameter('initial_pose.y').value)
            use_yaw_rad = self.latest_yaw_rad
            self.get_logger().info("Using IMU yaw + parameter XY for start.")
        else:
            # fallback entirely to parameters
            use_x = float(self.get_parameter('initial_pose.x').value)
            use_y = float(self.get_parameter('initial_pose.y').value)
            yaw_deg = float(self.get_parameter('initial_pose.yaw_deg').value)
            use_yaw_rad = math.radians(yaw_deg)
            self.get_logger().info("Using parameter-provided initial pose (no IMU or RViz input).")

        # Check service availability (quick non-blocking waits)
        # Wait for finish service, then start service
        svc_ok = self.finish_client.wait_for_service(timeout_sec=self.service_wait_timeout)
        if not svc_ok:
            self.get_logger().warn(f"Waiting for /finish_trajectory service...")
            return

        svc_ok = self.start_client.wait_for_service(timeout_sec=self.service_wait_timeout)
        if not svc_ok:
            self.get_logger().warn(f"Waiting for /start_trajectory service...")
            return

        # At this point we have a pose to use and services are up. Run the sequence once.
        try:
            # finish previous trajectory
            self._call_finish_trajectory(self.old_trajectory_id)

            # start new trajectory using chosen pose
            self._call_start_trajectory(use_x, use_y, use_yaw_rad)

            # mark done
            self.published = True
            self.get_logger().info("Initial pose successfully applied to Cartographer.")
        except Exception as e:
            self.get_logger().error(f"Failed to run start sequence: {e}")

    # ----------------- internal helpers -----------------
    def _call_finish_trajectory(self, trajectory_id: int) -> None:
        self.get_logger().info(f"Calling /finish_trajectory for id={trajectory_id}...")
        req = FinishTrajectory.Request()
        req.trajectory_id = int(trajectory_id)

        fut = self.finish_client.call_async(req)
        r = rclpy.spin_until_future_complete(self, fut, timeout_sec=self.service_call_timeout)
        if fut.done() and fut.result() is not None:
            self.get_logger().info(f"Finished trajectory {trajectory_id}: {fut.result().status.message}")
        else:
            raise RuntimeError("Timeout or failure calling /finish_trajectory")

    def _call_start_trajectory(self, x: float, y: float, yaw_rad: float) -> None:
        # Build quaternion from yaw
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw_rad))

        self.get_logger().info(
            f"Calling /start_trajectory with pose x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw_rad):.2f}° (qz={qz:.3f}, qw={qw:.3f})"
        )

        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_base
        req.use_initial_pose = True
        req.relative_to_trajectory_id = 0

        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = 0.0

        req.initial_pose.orientation.x = float(qx)
        req.initial_pose.orientation.y = float(qy)
        req.initial_pose.orientation.z = float(qz)
        req.initial_pose.orientation.w = float(qw)

        fut = self.start_client.call_async(req)
        r = rclpy.spin_until_future_complete(self, fut, timeout_sec=self.service_call_timeout)
        if fut.done() and fut.result() is not None:
            res = fut.result()
            self.get_logger().info(f"Started trajectory id={res.trajectory_id}: {res.status.message}")
        else:
            raise RuntimeError("Timeout or failure calling /start_trajectory")

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
