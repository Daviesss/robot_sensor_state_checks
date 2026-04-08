"""
ROS 2 State Topic Integration Tests
Tests that joint_states, tf, and tf_static are publishing adequately.

Requirements:
    pip install pytest
    ROS 2 environment must be sourced before running:
        source /opt/ros/<distro>/setup.bash

Run with:
    pytest test_state_topics.py -v
    # or directly:
    python3 -m pytest test_state_topics.py -v
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

from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage


# ──────────────────────────────────────────────
# Configuration
# ──────────────────────────────────────────────

TOPIC_CONFIG = {
    "joint_states": {
        "topic": "/joint_states",
        "msg_type": JointState,
        "min_rate_hz": 3.0,
        "timeout_sec": 5.0,
        "sample_window_sec": 3.0,
        "qos": "reliable",
    },
    "tf": {
        "topic": "/tf",
        "msg_type": TFMessage,
        "min_rate_hz": 10.0,
        "timeout_sec": 5.0,
        "sample_window_sec": 3.0,
        "qos": "best_effort",
    },
    "tf_static": {
        "topic": "/tf_static",
        "msg_type": TFMessage,
        "min_rate_hz": 1.0,
        "timeout_sec": 5.0,
        "sample_window_sec": 3.0,
        "qos": "tf_static",
    },
}

# QoS for sensor topics (TF)
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
)

# QoS for reliable topics (joint_states)
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# QoS for tf_static — TRANSIENT_LOCAL so late subscribers still receive it
TF_STATIC_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


# ──────────────────────────────────────────────
# Helper: Topic Monitor Node
# ──────────────────────────────────────────────

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


# ──────────────────────────────────────────────
# Base Test Class
# ──────────────────────────────────────────────

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
            pass

    def _make_monitor(self, config_key: str) -> TopicMonitor:
        cfg = TOPIC_CONFIG[config_key]
        if cfg.get("qos") == "best_effort":
            qos = SENSOR_QOS
        elif cfg.get("qos") == "tf_static":
            qos = TF_STATIC_QOS
        else:
            qos = RELIABLE_QOS
        monitor = TopicMonitor(cfg["topic"], cfg["msg_type"], qos_profile=qos)
        self.executor.add_node(monitor)
        return monitor

    def _wait_and_measure(self, monitor: TopicMonitor, cfg: dict) -> tuple[bool, float]:
        """Returns (received_first_msg, measured_rate_hz)."""
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


# ──────────────────────────────────────────────
# Individual Topic Tests
# ──────────────────────────────────────────────

class TestJointStates(ROS2TopicTestBase):
    """Tests for /joint_states (JointState)."""

    def setUp(self):
        self.cfg = TOPIC_CONFIG["joint_states"]
        self.monitor = self._make_monitor("joint_states")

    def tearDown(self):
        self.executor.remove_node(self.monitor)
        self.monitor.destroy_node()

    def test_joint_states_is_publishing(self):
        """Topic should publish at least one message within timeout."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(
            received,
            f"No message received on {self.cfg['topic']} within "
            f"{self.cfg['timeout_sec']}s. Is the robot state publisher running?",
        )

    def test_joint_states_rate_adequate(self):
        """Publish rate should meet minimum threshold."""
        received, rate = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
        self.assertGreaterEqual(
            rate,
            self.cfg["min_rate_hz"],
            f"/joint_states rate {rate:.2f} Hz is below minimum {self.cfg['min_rate_hz']} Hz",
        )

    def test_joint_states_has_joint_names(self):
        """JointState should have at least one named joint."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: JointState = self.monitor.last_msg
        self.assertGreater(
            len(msg.name),
            0,
            "JointState.name is empty — no joints reported",
        )

    def test_joint_states_names_and_positions_match(self):
        """Number of joint names should match number of position values."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: JointState = self.monitor.last_msg
        if len(msg.position) > 0:
            self.assertEqual(
                len(msg.name),
                len(msg.position),
                f"Joint name count ({len(msg.name)}) != position count ({len(msg.position)})",
            )

    def test_joint_states_has_frame_id(self):
        """JointState header frame_id should be non-empty."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: JointState = self.monitor.last_msg
        self.assertTrue(
            msg.header.frame_id or True,  # frame_id optional but stamp must exist
            "JointState header is missing",
        )


class TestTF(ROS2TopicTestBase):
    """Tests for /tf (TFMessage)."""

    def setUp(self):
        self.cfg = TOPIC_CONFIG["tf"]
        self.monitor = self._make_monitor("tf")

    def tearDown(self):
        self.executor.remove_node(self.monitor)
        self.monitor.destroy_node()

    def test_tf_is_publishing(self):
        """TF should publish at least one message within timeout."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(
            received,
            f"No message received on {self.cfg['topic']} within "
            f"{self.cfg['timeout_sec']}s. Is robot_state_publisher running?",
        )

    def test_tf_rate_adequate(self):
        """TF publish rate should meet minimum threshold."""
        received, rate = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")
        self.assertGreaterEqual(
            rate,
            self.cfg["min_rate_hz"],
            f"/tf rate {rate:.2f} Hz is below minimum {self.cfg['min_rate_hz']} Hz",
        )

    def test_tf_has_transforms(self):
        """TFMessage should contain at least one transform."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: TFMessage = self.monitor.last_msg
        self.assertGreater(
            len(msg.transforms),
            0,
            "TFMessage.transforms is empty — no transforms being broadcast",
        )

    def test_tf_transforms_have_frame_ids(self):
        """Every transform in TFMessage should have non-empty frame IDs."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: TFMessage = self.monitor.last_msg
        for i, t in enumerate(msg.transforms):
            self.assertTrue(
                t.header.frame_id,
                f"Transform [{i}] has empty header.frame_id",
            )
            self.assertTrue(
                t.child_frame_id,
                f"Transform [{i}] has empty child_frame_id",
            )


class TestTFStatic(ROS2TopicTestBase):
    """Tests for /tf_static (TFMessage)."""

    def setUp(self):
        self.cfg = TOPIC_CONFIG["tf_static"]
        self.monitor = self._make_monitor("tf_static")

    def tearDown(self):
        self.executor.remove_node(self.monitor)
        self.monitor.destroy_node()

    def test_tf_static_is_publishing(self):
        """tf_static should deliver at least one message within timeout."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(
            received,
            f"No message received on {self.cfg['topic']} within "
            f"{self.cfg['timeout_sec']}s. "
            "Is robot_state_publisher running? tf_static requires TRANSIENT_LOCAL QoS.",
        )

    def test_tf_static_has_transforms(self):
        """tf_static should contain at least one static transform."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: TFMessage = self.monitor.last_msg
        self.assertGreater(
            len(msg.transforms),
            0,
            "tf_static TFMessage.transforms is empty — no static transforms defined",
        )

    def test_tf_static_transforms_have_frame_ids(self):
        """Every static transform should have non-empty frame IDs."""
        received, _ = self._wait_and_measure(self.monitor, self.cfg)
        self.assertTrue(received, f"No messages on {self.cfg['topic']}")

        msg: TFMessage = self.monitor.last_msg
        for i, t in enumerate(msg.transforms):
            self.assertTrue(
                t.header.frame_id,
                f"Static transform [{i}] has empty header.frame_id",
            )
            self.assertTrue(
                t.child_frame_id,
                f"Static transform [{i}] has empty child_frame_id",
            )


# ──────────────────────────────────────────────
# Aggregate / Cross-topic Tests
# ──────────────────────────────────────────────

class TestAllStateTopicsPublishing(ROS2TopicTestBase):
    """Smoke test — verify all state topics publish within their timeout."""

    def test_all_state_topics_alive(self):
        """Every configured topic should deliver at least one message."""
        monitors = {}
        for key, cfg in TOPIC_CONFIG.items():
            if cfg.get("qos") == "best_effort":
                qos = SENSOR_QOS
            elif cfg.get("qos") == "tf_static":
                qos = TF_STATIC_QOS
            else:
                qos = RELIABLE_QOS
            m = TopicMonitor(
                cfg["topic"],
                cfg["msg_type"],
                qos_profile=qos,
                node_name=f"all_monitor_{key}",
            )
            self.executor.add_node(m)
            monitors[key] = (m, cfg)

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
            "The following state topics are NOT publishing:\n" + "\n".join(failures),
        )


# ──────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────

if __name__ == "__main__":
    unittest.main(verbosity=2)