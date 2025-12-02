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
        self.declare_parameter('old_trajectory_id', 1)
        self.declare_parameter('cycle_period', 1.0)  # how often timer wakes up to check preconditions
        self.declare_parameter('use_imu_yaw', True)  # if false, wait for RViz "2D Pose Estimate"

        self.config_dir = self.get_parameter('config_dir').value
        self.config_base = self.get_parameter('config_base').value
        if not self.config_dir:
            self.get_logger().fatal("Mandatory parameter 'config_dir' was not set. Shutting down node.")
            raise RuntimeError("mandatory parameter 'config_dir' must be set.")

        self.old_trajectory_id = int(self.get_parameter('old_trajectory_id').value)

        self.get_logger().info(f"Using cartographer configuration: {self.config_dir}/{self.config_base}")

        # preset values used with IMU orientation to create initial pose: 
        self.latest_x = self.get_parameter('initial_pose.x').value
        self.latest_y = self.get_parameter('initial_pose.y').value

        # ----- runtime state -----
        self.latest_imu_yaw = None    # updated from IMU
        self.latest_pose_msg = None   # updated from /initialpose (RViz "2D Pose Estimate")

        self.state = "BEGINNING"       # BEGINNING -> WORKING -> TRAJ_FINISH -> TRAJ_START -> IDLE -> WORKING

        # ----- subscriptions -----
        # IMU subscription - default QoS is fine for IMU
        if self.get_parameter('use_imu_yaw').value:
            self.get_logger().info("FYI: use_imu_yaw is true, will take orientation from IMU")
            self.imu_subscription = self.create_subscription(
                Imu,
                '/imu/data',
                self.imu_callback,
                10
            )
        else:
            self.get_logger().info("FYI: use_imu_yaw is false, will wait for RViz '2D Pose Estimate'")


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
        cycle_period = float(self.get_parameter('cycle_period').value)
        self.timer = self.create_timer(cycle_period, self.timer_callback)

        self.get_logger().info("OK: initial_pose_pub ready")


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

        if self.state == "BEGINNING":
            self.state = "WORKING"  # IMU yaw available, get to work

    def pose_estimate_callback(self, msg: PoseWithCovarianceStamped) -> None:

        """Store the RViz pose; it overrides parameter pose or IMU when present."""
        self.get_logger().info("=========== Received /initialpose from RViz - will use it as initial pose. ==============")

        self.latest_pose_msg = msg

        if self._have_pose() and self.state == "IDLE":
            self.state = "WORKING"   # trigger restart on next timer tick

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
        if self.state == "BEGINNING":   # the beginning

            if not self._have_pose():
                if self.get_parameter('use_imu_yaw').value:
                    self.get_logger().info("SM: BEGINNING - Waiting for initial orientation from IMU or RViz '2D Pose Estimate'...")
                else:
                    self.get_logger().info("SM: BEGINNING - Waiting for initial orientation from RViz '2D Pose Estimate'...")
                return

            self.get_logger().info("SM: have pose, BEGINNING -> WORKING")
            self.state = "WORKING"

        elif self.state == "WORKING":  # IMU yaw available, get to work the services
            
            if not self.finish_client.service_is_ready():
                self.get_logger().info("SM: WORKING - Waiting for Cartographer services...")
                return

            # Ready → start sequence
            self.get_logger().info("SM: OK: Cartographer services ready, WORKING -> TRAJ_FINISH")
            self.state = "TRAJ_FINISH"

        elif self.state == "TRAJ_FINISH":
            self.get_logger().info("SM: TRAJ_FINISH - finishing trajectory...")
            self._call_finish_trajectory()

        elif self.state == "TRAJ_START":
            self.get_logger().info("SM: TRAJ_START - starting new trajectory...")
            self._call_start_trajectory()

        elif self.state == "IDLE":
            # waiting for RViz "2D Pose Estimate"
            #self.get_logger().info("SM: IDLE")
            pass

    # ------------------------------------------------------------
    # Pose Helpers
    # ------------------------------------------------------------

    def _have_pose(self):
        """We need either RViz pose OR IMU yaw."""
        return self.latest_pose_msg is not None or self.latest_imu_yaw is not None

    def _build_pose(self):
        """
        only call if _have_pose()
        Return (x,y,qx,qy,qz,qw).
        """
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
                f"RViz 2D Pose Estimate: x={p.x:.2f}, y={p.y:.2f}, "
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
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, self.latest_imu_yaw)

        return self.latest_x, self.latest_y, qx, qy, qz, qw

    # ------------------------------------------------------------
    # Service Calls (Async)
    # ------------------------------------------------------------

    def _call_finish_trajectory(self):
        req = FinishTrajectory.Request()
        req.trajectory_id = int(self.old_trajectory_id)

        self.get_logger().info(f"IP: Calling /finish_trajectory service...  trajectory_id: {req.trajectory_id}")

        future = self.finish_client.call_async(req)
        future.add_done_callback(self._on_finish_done)

    def _on_finish_done(self, future):
        if future.result() is None:
            self.get_logger().error("Error: finish_trajectory FAILED, incrementing trajectory_id")
            self.old_trajectory_id += 1
            self._call_finish_trajectory()  # try again
            return

        self.get_logger().info(f"OK: finish_trajectory id: {self.old_trajectory_id} - Success")
        self.old_trajectory_id += 1
        self.state = "TRAJ_START"


    def _call_start_trajectory(self):
        x, y, qx, qy, qz, qw = self._build_pose()

        req = StartTrajectory.Request()
        req.configuration_directory = self.config_dir
        req.configuration_basename = self.config_base
        req.use_initial_pose = True
        #req.relative_to_trajectory_id = 0

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

        self.get_logger().info("SM: IDLE")
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
