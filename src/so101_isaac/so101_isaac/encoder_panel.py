#!/usr/bin/env python3
"""Live terminal panel of ESP32 encoder values, for eyeballing esp_joint_bridge.

    ros2 run so101_isaac encoder_panel

Runs alongside Isaac Sim (a separate terminal, doesn't touch Isaac Sim or
its code) and shows, per joint: the raw AS5600 count, the physical degree
it decodes to (esp_joint_bridge.py's raw_to_angle(), pre home_deg/direction)
and the final radians/degrees actually sent to joint_commander. Handy while
tuning home_deg/direction in config/esp_bridge.yaml without staring at
`ros2 topic echo`.

Quit with q or Ctrl+C.
"""

from __future__ import annotations

import curses
import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_isaac.joint_commander import DEFAULT_JOINTS


class EncoderPanel(Node):
    def __init__(self) -> None:
        super().__init__("encoder_panel")

        self.declare_parameter("joint_names", DEFAULT_JOINTS)
        self.declare_parameter("encoder_topic", "/esp_joint_bridge/encoder_raw")
        self.declare_parameter("target_topic", "/joint_commander/set_target")

        self.joint_names: list[str] = list(self.get_parameter("joint_names").value)
        self.raw: dict[str, float] = {}
        self.deg: dict[str, float] = {}
        self.target_rad: dict[str, float] = {}
        self.last_encoder_stamp = None
        self.last_target_stamp = None

        encoder_topic = self.get_parameter("encoder_topic").value
        target_topic = self.get_parameter("target_topic").value
        self.create_subscription(JointState, encoder_topic, self._on_encoder, 10)
        self.create_subscription(JointState, target_topic, self._on_target, 10)

    def _on_encoder(self, msg: JointState) -> None:
        for name, deg, raw in zip(msg.name, msg.position, msg.effort):
            self.raw[name] = raw
            self.deg[name] = deg
        self.last_encoder_stamp = self.get_clock().now()

    def _on_target(self, msg: JointState) -> None:
        for name, rad in zip(msg.name, msg.position):
            self.target_rad[name] = rad
        self.last_target_stamp = self.get_clock().now()

    def age_s(self, stamp) -> float | None:
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9


def _run(stdscr, node: EncoderPanel) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)

        try:
            key = stdscr.getch()
        except curses.error:
            key = -1
        if key in (ord("q"), ord("Q")):
            return

        stdscr.erase()
        stdscr.addstr(0, 0, "ESP32 encoder panel -- q to quit", curses.A_BOLD)

        enc_age = node.age_s(node.last_encoder_stamp)
        enc_status = f"{enc_age:4.1f}s ago" if enc_age is not None else "no data yet"
        target_age = node.age_s(node.last_target_stamp)
        target_status = f"{target_age:4.1f}s ago" if target_age is not None else "no data yet"
        stdscr.addstr(1, 0, f"encoder_raw: {enc_status}    set_target: {target_status}")

        header = f"{'joint':<16}{'raw':>8}{'deg':>10}{'target rad':>12}{'target deg':>12}"
        stdscr.addstr(3, 0, header, curses.A_UNDERLINE)

        for i, name in enumerate(node.joint_names):
            raw = node.raw.get(name)
            deg = node.deg.get(name)
            rad = node.target_rad.get(name)
            row = (
                f"{name:<16}"
                f"{'' if raw is None else f'{raw:8.0f}'}"
                f"{'' if deg is None else f'{deg:10.2f}'}"
                f"{'' if rad is None else f'{rad:12.4f}'}"
                f"{'' if rad is None else f'{math.degrees(rad):12.2f}'}"
            )
            stdscr.addstr(4 + i, 0, row)

        stdscr.refresh()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderPanel()
    try:
        curses.wrapper(_run, node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except curses.error as exc:
        print(f"[encoder_panel] terminal UI error (needs a real TTY): {exc}", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
