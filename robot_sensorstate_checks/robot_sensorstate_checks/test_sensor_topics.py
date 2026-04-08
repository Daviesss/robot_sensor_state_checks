"""
ROS 2 Sensor Topic Unit Tests
Tests that lidar (scan), odom, cmd_vel, and image_topic are publishing adequately.

Requirements:
    pip install pytest
    ROS 2 environment must be sourced before running:
        source /opt/ros/<distro>/setup.bash

Run with:
    pytest test_sensor_topics.py -v
    # or directly:
    python3 -m pytest test_sensor_topics.py -v
"""

import time
import threading
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import LaserScan, Image , Imu 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# Configuration

TOPIC_CONFIG = {
    "scan": {
        "topic": "/scan",
        "msg_type": LaserScan,
        "min_rate_hz": 5.0,       # Minimum acceptable publish rate
        "timeout_sec": 5.0,       # How long to wait for first message
        "sample_window_sec": 3.0, # Window for rate measurement
        "qos": "best_effort",
    },
    "odom": {
        "topic": "/odom",
        "msg_type": Odometry,
        "min_rate_hz": 10.0,
        "timeout_sec": 5.0,
        "sample_window_sec": 3.0,
        "qos": "best_effort",   
    },
    "cmd_vel": {
        "topic": "/cmd_vel",
        "msg_type": Twist,
        "min_rate_hz": 1.0,
        "timeout_sec": 10.0,
        "sample_window_sec": 5.0,
        "qos": "reliable",
    },
    "image": {
        "topic": "/camera/image_raw",
        "msg_type": Image,
        "min_rate_hz": 5.0, 
        "timeout_sec": 5.0,
        "sample_window_sec": 3.0,
        "qos": "reliable",
    },
    "depth_image": {
        "topic": "camera/depth/image_raw",
        "msg_type": Image,
        "min_rate_hz": 5.0,
        "timeout_sec": 5.0,
        "sample_window_sec": 3.0,
        "qos": "reliable",      
    },
    "imu": {
        "topic": "/imu/data",
        "msg_type": Imu,
        "min_rate_hz": 50.0,       # IMUs typically run at 100–400 Hz; 50 Hz is a safe minimum
        "timeout_sec": 5.0,
        "sample_window_sec": 3.0,
        "qos": "best_effort",      # IMU sensor data profile: best-effort, volatile
    },
}

# QoS compatible with most sensor publishers (sensor data profile)
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# QoS for reliable topics like odom/cmd_vel
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)



# Helper: Topic Monitor Node
class TopicMonitor(Node):
    """Subscribes to a topic and records message arrival timestamps."""

    def __init__(self, topic: str, msg_type, qos_profile=None, node_name: str = None):
        safe_name = topic.replace("/", "_").lstrip("_")
        name = node_name or f"monitor_{safe_name}"
        super().__init__(name)

        self.topic = topic
        self.msg_type = msg_type
        self.received_timestamps: list[float] = []
        self.last_msg = None
        self._lock = threading.Lock()

        profile = qos_profile or SENSOR_QOS
        
        # subscription callback will record arrival times and store the last message
        self.subscription = self.create_subscription(
            msg_type,
            topic,
            self._callback,
            profile,
        )

    def _callback(self, msg):
        with self._lock:
            self.received_timestamps.append(time.monotonic())
            self.last_msg = msg

    @property
    def message_count(self) -> int:
        with self._lock:
            return len(self.received_timestamps)

    def received_any(self) -> bool:
        return self.message_count > 0

    def rate_hz(self, window_sec: float) -> float:
        """Compute publish rate over the most recent `window_sec` seconds."""
        now = time.monotonic()
        with self._lock:
            recent = [t for t in self.received_timestamps if now - t <= window_sec]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])

    def wait_for_message(self, timeout_sec: float, executor) -> bool:
        """Spin until at least one message is received or timeout expires."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            if self.received_any():
                return True
        return False



# Base Test Class


class ROS2TopicTestBase(unittest.TestCase):
    """Initialises rclpy once for the entire test module."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls._spin_thread = threading.Thread(target=cls._spin, daemon=True)
        cls._spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    @classmethod
    def _spin(cls):
        try:
            cls.executor.spin()
        except Exception:
            pass  # Executor shuts down on rclpy.shutdown()

    def _make_monitor(self, config_key: str) -> TopicMonitor:
        cfg = TOPIC_CONFIG[config_key]
        # Use best-effort QoS for scan/image, reliable for odom/cmd_vel
        # qos = SENSOR_QOS if config_key in ("scan", "image") else RELIABLE_QOS
        qos = SENSOR_QOS if cfg.get("qos") == "best_effort" else RELIABLE_QOS
        monitor = TopicMonitor(cfg["topic"], cfg["msg_type"], qos_profile=qos)
        self.executor.add_node(monitor)
        return monitor

    def _wait_and_measure(self, monitor: TopicMonitor, cfg: dict) -> tuple[bool, float]:
        """
        Returns (received_first_msg, measured_rate_hz).
        Spins the executor in a background thread so we just sleep here.
        """
        received = self._wait_for_first(monitor, cfg["timeout_sec"])
        if received:
            time.sleep(cfg["sample_window_sec"])
        rate = monitor.rate_hz(cfg["sample_window_sec"])
        return received, rate

    def _wait_for_first(self, monitor: TopicMonitor, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if monitor.received_any():
                return True
            time.sleep(0.05)
        return False



# Individual Topic Tests


class TestLidarScan(ROS2TopicTestBase):
    """Tests for /scan (LaserScan)."""

    def setUp(self):
        self.cfg = TOPIC_CONFIG["scan"]
        self.monitor = self._make_monitor("scan")

    def tearDown(self):
        self.executor.remove_node(self.monitor)
        self.monitor.destroy_node()

    def test_scan_is_publishing(self):
        """Topic should publish at least one message within timeout."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(
            received,
            f"No message received on {self.cfg['topic']} within "
            f"{self.cfg['timeout_sec']}s. Is your lidar driver running?",
        )

    def test_scan_rate_adequate(self):
        """Publish rate should meet minimum threshold."""
        received, rate = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
        self.assertGreaterEqual(
            rate,
            self.cfg["min_rate_hz"],
            f"/scan rate {rate:.2f} Hz is below minimum {self.cfg['min_rate_hz']} Hz",
        )

    def test_scan_message_fields(self):
        """LaserScan should have non-empty ranges and a valid angle span."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: LaserScan = self.monitor.last_msg
        self.assertIsNotNone(msg)
        self.assertGreater(len(msg.ranges), 0, "LaserScan.ranges is empty")
        self.assertGreater(
            msg.angle_max - msg.angle_min,
            0,
            "LaserScan angle span is zero or negative",
        )
        self.assertGreater(msg.range_max, msg.range_min, "range_max <= range_min")

    def test_scan_no_nan_inf_ranges(self):
        """Ranges should not be entirely NaN or Inf (some are OK, all is not)."""
        import math
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: LaserScan = self.monitor.last_msg
        valid = [r for r in msg.ranges if not (math.isnan(r) or math.isinf(r))]
        self.assertGreater(
            len(valid),
            0,
            "All LaserScan ranges are NaN or Inf — sensor may be faulty",
        )


class TestOdometry(ROS2TopicTestBase):
    """Tests for /odom (Odometry)."""

    def setUp(self):
        self.cfg = TOPIC_CONFIG["odom"]
        self.monitor = self._make_monitor("odom")

    def tearDown(self):
        self.executor.remove_node(self.monitor)
        self.monitor.destroy_node()

    def test_odom_is_publishing(self):
        """Odometry should arrive within timeout."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(
            received,
            f"No message received on {self.cfg['topic']} within "
            f"{self.cfg['timeout_sec']}s.",
        )

    def test_odom_rate_adequate(self):
        """Odometry rate should meet minimum threshold."""
        received, rate = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
        self.assertGreaterEqual(
            rate,
            self.cfg["min_rate_hz"],
            f"/odom rate {rate:.2f} Hz is below minimum {self.cfg['min_rate_hz']} Hz",
        )

    def test_odom_has_frame_ids(self):
        """Odometry header and child_frame_id should be non-empty."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: Odometry = self.monitor.last_msg
        self.assertTrue(
            msg.header.frame_id,
            "Odometry header.frame_id is empty",
        )
        self.assertTrue(
            msg.child_frame_id,
            "Odometry child_frame_id is empty",
        )

    def test_odom_covariance_not_all_zero(self):
        """Pose covariance should not be all zeros (would indicate uninitialised data)."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: Odometry = self.monitor.last_msg
        cov = list(msg.pose.covariance)
        self.assertTrue(
            any(v != 0.0 for v in cov),
            "All pose covariance values are zero — likely uninitialised",
        )


class TestCmdVel(ROS2TopicTestBase):
    """Tests for /cmd_vel (Twist)."""

    def setUp(self):
        self.cfg = TOPIC_CONFIG["cmd_vel"]
        self.monitor = self._make_monitor("cmd_vel")

    def tearDown(self):
        self.executor.remove_node(self.monitor)
        self.monitor.destroy_node()

    def test_cmd_vel_is_publishing(self):
        """cmd_vel should receive at least one message within timeout."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(
            received,
            f"No message on {self.cfg['topic']} within {self.cfg['timeout_sec']}s. "
            "Is a controller or teleop node publishing?",
        )

    def test_cmd_vel_rate_adequate(self):
        """cmd_vel rate should meet minimum threshold."""
        received, rate = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
        self.assertGreaterEqual(
            rate,
            self.cfg["min_rate_hz"],
            f"/cmd_vel rate {rate:.2f} Hz is below minimum {self.cfg['min_rate_hz']} Hz",
        )

    def test_cmd_vel_velocities_finite(self):
        """Linear and angular velocities should be finite numbers."""
        import math
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: Twist = self.monitor.last_msg
        for field, val in [
            ("linear.x", msg.linear.x),
            ("linear.y", msg.linear.y),
            ("linear.z", msg.linear.z),
            ("angular.x", msg.angular.x),
            ("angular.y", msg.angular.y),
            ("angular.z", msg.angular.z),
        ]:
            self.assertTrue(
                math.isfinite(val),
                f"cmd_vel.{field} = {val} is not finite",
            )


class TestImageTopic(ROS2TopicTestBase):
    """Tests for /image_topic (image)."""

    def setUp(self):
        self.cfg = TOPIC_CONFIG["image"]
        self.monitor = self._make_monitor("image")

    def tearDown(self):
        self.executor.remove_node(self.monitor)
        self.monitor.destroy_node()

    def test_image_is_publishing(self):
        """Image topic should publish at least one frame within timeout."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(
            received,
            f"No message on {self.cfg['topic']} within {self.cfg['timeout_sec']}s. "
            "Is the camera driver running?",
        )

    def test_image_rate_adequate(self):
        """Image publish rate should meet minimum threshold."""
        received, rate = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
        self.assertGreaterEqual(
            rate,
            self.cfg["min_rate_hz"],
            f"/image_topic rate {rate:.2f} Hz is below minimum {self.cfg['min_rate_hz']} Hz",
        )

    def test_image_dimensions_nonzero(self):
        """Image width and height must be non-zero."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: Image = self.monitor.last_msg
        self.assertGreater(msg.width, 0, "Image width is 0")
        self.assertGreater(msg.height, 0, "Image height is 0")

    def test_image_encoding_nonempty(self):
        """Image encoding field should be a non-empty string."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: Image = self.monitor.last_msg
        self.assertTrue(msg.encoding, "Image encoding field is empty")

    def test_image_data_matches_dimensions(self):
        """Image data size should match width × height × step."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: Image = self.monitor.last_msg
        expected_size = msg.height * msg.step
        actual_size = len(msg.data)
        self.assertEqual(
            actual_size,
            expected_size,
            f"Image data size mismatch: got {actual_size}, "
            f"expected {expected_size} (height={msg.height}, step={msg.step})",
        )
        

# Depth Image Tests
class TestDepthImage(ROS2TopicTestBase):
    """Tests for /camera/depth/image_raw (Image).
 
    Depth images are validated more strictly than RGB because downstream
    consumers (pointcloud generation, costmap inflation) require every
    frame to be structurally valid. A dropped or malformed depth frame
    creates phantom-free zones in the costmap that cause silent navigation
    failures.
 
    Encoding note:
        Gazebo depth cameras publish as '32FC1' (32-bit float, single channel,
        metres). Real depth cameras (RealSense, OAK-D) may publish '16UC1'
        (16-bit unsigned, millimetres). Both are valid the test accepts either.
    """
 
    def setUp(self):
        self.cfg = TOPIC_CONFIG["depth_image"]
        self.monitor = self._make_monitor("depth_image")
 
    def tearDown(self):
        self.executor.remove_node(self.monitor)
        self.monitor.destroy_node()
 
    # Liveness
 
    def test_depth_image_is_publishing(self):
        """Depth topic should publish at least one frame within timeout."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(
            received,
            f"No message on {self.cfg['topic']} within {self.cfg['timeout_sec']}s. "
            "Is the depth camera plugin active in your SDF?",
        )
 
    # Rate 
 
    def test_depth_image_rate_adequate(self):
        """Depth publish rate must meet the minimum threshold."""
        received, rate = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
        self.assertGreaterEqual(
            rate,
            self.cfg["min_rate_hz"],
            f"/camera/depth/image_raw rate {rate:.2f} Hz is below minimum "
            f"{self.cfg['min_rate_hz']} Hz. Check update_rate in your depth camera SDF config.",
        )
 
    # Dimensions 
 
    def test_depth_image_dimensions_nonzero(self):
        """Depth image width and height must be non-zero."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Image = self.monitor.last_msg
        self.assertGreater(msg.width, 0, "Depth image width is 0")
        self.assertGreater(msg.height, 0, "Depth image height is 0")
 
    # Encoding 
 
    def test_depth_image_encoding_is_depth_format(self):
        """Depth image encoding must be a recognised depth format.
 
        '32FC1' — Gazebo sim, RealSense (float metres)
        '16UC1' — RealSense, OAK-D (uint16 millimetres)
        """
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Image = self.monitor.last_msg
        valid_encodings = {"32FC1", "16UC1"}
        self.assertIn(
            msg.encoding, valid_encodings,
            f"Depth image encoding '{msg.encoding}' is not a recognised depth format "
            f"(expected one of {valid_encodings}). RGB data may have been bridged incorrectly.",
        )
 
    # Data integrity 
 
    def test_depth_image_data_matches_dimensions(self):
        """Depth image data buffer size must match height × step."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Image = self.monitor.last_msg
        expected_size = msg.height * msg.step
        actual_size = len(msg.data)
        self.assertEqual(
            actual_size, expected_size,
            f"Depth image data size mismatch: got {actual_size}, "
            f"expected {expected_size} (height={msg.height}, step={msg.step}). "
            "Possible partial frame or bridge truncation.",
        )
 
    def test_depth_image_not_entirely_zero(self):
        """Depth image should contain at least some non-zero pixels.
 
        An all-zero depth image means the camera sees nothing — either the
        scene is empty, the plugin is misconfigured, or the bridge is sending
        blank frames. A few zeros (reflective surfaces, out-of-range) is normal.
        """
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Image = self.monitor.last_msg
        self.assertTrue(
            any(b != 0 for b in msg.data),
            "Depth image data is entirely zero — camera may not see any geometry. "
            "Check that objects exist within the camera frustum in your world file.",
        )
 
    def test_depth_image_has_frame_id(self):
        """Depth image header.frame_id must be non-empty."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Image = self.monitor.last_msg
        self.assertTrue(
            msg.header.frame_id,
            "Depth image header.frame_id is empty — "
            "set optical_frame_id in your depth camera plugin.",
        )
        
        

# IMU Tests
class TestImu(ROS2TopicTestBase):
    """Tests for /imu/data (Imu).
 
    Covers liveness, publish rate, frame ID, data sanity (finite values,
    unit-quaternion orientation), and covariance population.
 
    Note on covariance in Gazebo:
        The libgazebo_ros_imu_sensor plugin populates covariance diagonals with
        the configured noise variances.  If your SDF sets gaussian_noise to 0.0
        for all axes, the covariance matrices will legitimately be all-zero —
        adjust the assertion or the plugin config accordingly.
    """
 
    def setUp(self):
        self.cfg = TOPIC_CONFIG["imu"]
        self.monitor = self._make_monitor("imu")
 
    def tearDown(self):
        self.executor.remove_node(self.monitor)
        self.monitor.destroy_node()
 
    # Liveness 
 
    def test_imu_is_publishing(self):
        """IMU should publish at least one message within timeout."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(
            received,
            f"No message received on {self.cfg['topic']} within "
            f"{self.cfg['timeout_sec']}s. Is the IMU driver / Gazebo plugin active?",
        )
 
    # Rate 
 
    def test_imu_rate_adequate(self):
        """IMU publish rate must meet the minimum threshold (default: 50 Hz)."""
        received, rate = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
        self.assertGreaterEqual(
            rate,
            self.cfg["min_rate_hz"],
            f"/imu/data rate {rate:.2f} Hz is below minimum {self.cfg['min_rate_hz']} Hz. "
            "Check update_rate in your SDF/URDF IMU plugin config.",
        )
 
    # Frame ID 
 
    def test_imu_has_frame_id(self):
        """IMU header.frame_id must be a non-empty string."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Imu = self.monitor.last_msg
        self.assertTrue(
            msg.header.frame_id,
            "Imu header.frame_id is empty — set the frame_name in your IMU plugin.",
        )
 
    # Orientation ───
 
    def test_imu_orientation_is_unit_quaternion(self):
        """Orientation quaternion magnitude should be ~1.0 (unit quaternion).
 
        A zero quaternion (0, 0, 0, 0) means the plugin is not populating
        orientation — either enable it or set
        orientation_covariance[0] = -1 to signal 'not provided'.
        """
        import math
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Imu = self.monitor.last_msg
        q = msg.orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        self.assertAlmostEqual(
            norm, 1.0, delta=0.01,
            msg=f"IMU orientation quaternion norm is {norm:.4f}, expected ~1.0. "
                "Plugin may not be publishing orientation.",
        )
 
    def test_imu_orientation_values_finite(self):
        """All orientation quaternion components must be finite."""
        import math
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Imu = self.monitor.last_msg
        q = msg.orientation
        for axis, val in [("x", q.x), ("y", q.y), ("z", q.z), ("w", q.w)]:
            self.assertTrue(
                math.isfinite(val),
                f"IMU orientation.{axis} = {val} is not finite",
            )
 
    # Angular Velocity 
 
    def test_imu_angular_velocity_finite(self):
        """Angular velocity components must all be finite."""
        import math
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Imu = self.monitor.last_msg
        av = msg.angular_velocity
        for axis, val in [("x", av.x), ("y", av.y), ("z", av.z)]:
            self.assertTrue(
                math.isfinite(val),
                f"IMU angular_velocity.{axis} = {val} is not finite",
            )
 
    def test_imu_angular_velocity_within_physical_limits(self):
        """Angular velocity should be within ±2000 °/s (≈ 34.9 rad/s) — a common gyro limit."""
        import math
        MAX_RAD_S = math.radians(2000)  # 34.9 rad/s
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Imu = self.monitor.last_msg
        av = msg.angular_velocity
        for axis, val in [("x", av.x), ("y", av.y), ("z", av.z)]:
            self.assertLessEqual(
                abs(val), MAX_RAD_S,
                f"IMU angular_velocity.{axis} = {val:.3f} rad/s exceeds physical limit "
                f"({MAX_RAD_S:.1f} rad/s). Possible noise / unit error.",
            )
 
    # Linear Acceleration 
 
    def test_imu_linear_acceleration_finite(self):
        """Linear acceleration components must all be finite."""
        import math
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Imu = self.monitor.last_msg
        la = msg.linear_acceleration
        for axis, val in [("x", la.x), ("y", la.y), ("z", la.z)]:
            self.assertTrue(
                math.isfinite(val),
                f"IMU linear_acceleration.{axis} = {val} is not finite",
            )
 
    def test_imu_linear_acceleration_gravity_plausible(self):
        """Linear acceleration magnitude should be in a plausible gravity range (1–20 m/s²).
 
        At rest, the IMU reads gravity (≈9.81 m/s²). The test allows a wide
        band to accommodate motion, but catches completely degenerate output.
        """
        import math
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Imu = self.monitor.last_msg
        la = msg.linear_acceleration
        magnitude = math.sqrt(la.x**2 + la.y**2 + la.z**2)
        self.assertGreater(
            magnitude, 1.0,
            f"IMU linear_acceleration magnitude {magnitude:.3f} m/s² is suspiciously low. "
            "Gravity should contribute ~9.81 m/s² at rest.",
        )
        self.assertLess(
            magnitude, 20.0,
            f"IMU linear_acceleration magnitude {magnitude:.3f} m/s² is suspiciously high. "
            "Possible unit mismatch or runaway noise.",
        )
 
    # Covariance
 
    def test_imu_angular_velocity_covariance_not_all_zero(self):
        """Angular velocity covariance should be populated or signal 'not provided' (-1 sentinel).
 
        REP-145: if covariance[0] == -1, the field is explicitly marked as
        not provided — that is acceptable.  All-zeros with no sentinel means
        the plugin forgot to fill it in.
        """
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Imu = self.monitor.last_msg
        cov = list(msg.angular_velocity_covariance)
        not_provided = cov[0] == -1.0
        self.assertTrue(
            not_provided or any(v != 0.0 for v in cov),
            "IMU angular_velocity_covariance is all zeros with no REP-145 sentinel. "
            "Set gaussian_noise in your SDF or mark as not provided (cov[0] = -1).",
        )
 
    def test_imu_linear_acceleration_covariance_not_all_zero(self):
        """Linear acceleration covariance should be populated or carry the -1 sentinel."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
 
        msg: Imu = self.monitor.last_msg
        cov = list(msg.linear_acceleration_covariance)
        not_provided = cov[0] == -1.0
        self.assertTrue(
            not_provided or any(v != 0.0 for v in cov),
            "IMU linear_acceleration_covariance is all zeros with no REP-145 sentinel. "
            "Set gaussian_noise in your SDF or mark as not provided (cov[0] = -1).",
        )
 



# Aggregate / Cross-topic Tests


class TestAllTopicsPublishing(ROS2TopicTestBase):
    """Smoke test — verify all four topics publish within their timeout."""

    def test_all_topics_alive(self):
        """Every configured topic should deliver at least one message."""
        monitors = {}
        for key, cfg in TOPIC_CONFIG.items():
            qos = SENSOR_QOS if key in ("scan", "image") else RELIABLE_QOS
            m = TopicMonitor(cfg["topic"], cfg["msg_type"], qos_profile=qos,
                             node_name=f"all_monitor_{key}")
            self.executor.add_node(m)
            monitors[key] = (m, cfg)

        # Let all monitors collect for the longest timeout
        max_timeout = max(cfg["timeout_sec"] for _, cfg in monitors.values())
        time.sleep(max_timeout)

        failures = []
        for key, (monitor, cfg) in monitors.items():
            if not monitor.received_any():
                failures.append(f"  • {cfg['topic']} ({key}): no messages received")

        for key, (monitor, cfg) in monitors.items():
            self.executor.remove_node(monitor)
            monitor.destroy_node()

        self.assertFalse(
            failures,
            "The following topics are NOT publishing:\n" + "\n".join(failures),
        )



# Entry point
if __name__ == "__main__":
    unittest.main(verbosity=2)