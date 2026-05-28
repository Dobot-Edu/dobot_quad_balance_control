#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from .config import load_yaml


class DemoMonitorNode(Node):
    """Classroom-friendly terminal monitor for the balance control demo."""

    def __init__(self, config_file: str):
        super().__init__("dobot_balance_demo_monitor_node")
        config = load_yaml(config_file)
        display_cfg = config.get("display", {})
        control_cfg = config.get("control", {})

        self._period = max(0.1, float(display_cfg.get("monitor_period_seconds", 0.5)))
        self._stale_timeout = max(0.5, float(display_cfg.get("stale_timeout_seconds", 2.0)))
        self._target_roll = float(control_cfg.get("target_roll_deg", 0.0))
        self._target_pitch = float(control_cfg.get("target_pitch_deg", 0.0))

        self._latest_imu: list[float] | None = None
        self._latest_attitude: list[float] | None = None
        self._latest_event = "等待数据"
        self._last_imu_time = 0.0
        self._last_attitude_time = 0.0
        self._trigger_count = 0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Float32MultiArray, "/balance_control/imu_rpy", self._imu_cb, sensor_qos)
        self.create_subscription(Float32MultiArray, "/balance_control/attitude", self._attitude_cb, sensor_qos)
        self.create_subscription(String, "/balance_control/state", self._state_cb, 10)
        self.create_subscription(String, "/balance_control/imu_status", self._imu_status_cb, 10)
        self.create_timer(self._period, self._render)

        self.get_logger().info("课堂展示面板已启动：观察 [IMU]、[FILTER]、[ERROR]、[CMD]、[EVENT]")

    def _imu_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._latest_imu = [math.degrees(float(v)) for v in msg.data[:3]]
            self._last_imu_time = time.time()

    def _attitude_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self._latest_attitude = [float(v) for v in msg.data[:6]]
            self._last_attitude_time = time.time()

    def _state_cb(self, msg: String):
        self._latest_event = msg.data
        if "[TRIGGER]" in msg.data or "compensation" in msg.data:
            self._trigger_count += 1
            self.get_logger().info("[EVENT] %s" % msg.data)

    def _imu_status_cb(self, msg: String):
        self._latest_event = msg.data

    def _render(self):
        now = time.time()
        imu_ok = self._latest_imu is not None and now - self._last_imu_time <= self._stale_timeout
        ctrl_ok = (
            self._latest_attitude is not None
            and now - self._last_attitude_time <= self._stale_timeout
        )

        imu = self._latest_imu or [float("nan"), float("nan"), float("nan")]
        att = self._latest_attitude or [float("nan")] * 6
        raw_roll, raw_pitch, filt_roll, filt_pitch, cmd_roll, cmd_pitch = att
        err_roll = filt_roll - self._target_roll
        err_pitch = filt_pitch - self._target_pitch

        lines = [
            "",
            "================ Dobot Balance Control Demo ================",
            "[IMU]    roll=%+7.2f deg | pitch=%+7.2f deg | yaw=%+7.2f deg | %s"
            % (imu[0], imu[1], imu[2], "OK" if imu_ok else "WAIT"),
            "[RAW]    roll=%+7.2f deg | pitch=%+7.2f deg" % (raw_roll, raw_pitch),
            "[FILTER] roll=%+7.2f deg | pitch=%+7.2f deg | %s"
            % (filt_roll, filt_pitch, "OK" if ctrl_ok else "WAIT"),
            "[ERROR]  roll=%+7.2f deg | pitch=%+7.2f deg" % (err_roll, err_pitch),
            "[CMD]    roll=%+7.2f deg | pitch=%+7.2f deg" % (cmd_roll, cmd_pitch),
            "[EVENT]  trigger_count=%03d | %s" % (self._trigger_count, self._latest_event),
            "============================================================",
        ]
        self.get_logger().info("\n".join(lines))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="balance_control_config.yaml 配置文件路径")
    args = parser.parse_args(argv)

    rclpy.init()
    node = DemoMonitorNode(args.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
