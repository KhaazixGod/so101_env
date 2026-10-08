#!/usr/bin/env python3
"""Bridge ESP32-S3 (PCA9548A + 6x AS5600) angle readings into joint_commander.

The firmware prints all six AS5600 channels on one line, e.g.::

    Kênh 0: Raw: 1234 | Kênh 1: Không phát hiện AS5600 | ...

This node parses that line and converts each channel's raw 12-bit reading
to a physical joint angle using the JOINTS calibration table below -- ported
verbatim from /home/cuong-phan/test/main.py (the standalone log reader used
to calibrate the hardware), so the two must be kept in sync: if you
re-calibrate there (new ref_raw from a fresh reference pose), copy the new
numbers here too.

That physical angle is then re-centred on home_deg (config/esp_bridge.yaml)
-- the reading each channel had at whatever pose should map to the SO-101's
own zero/home pose in the URDF -- and flipped per-channel by `direction` if
the magnet runs opposite the URDF's positive direction. The result is
republished on joint_commander's ``~/set_target`` topic, so
joint_commander's existing ramp limiting and limit clamping
(joint_commander.py) smooths it before anything reaches Isaac Sim's
/joint_command. This node never talks to Isaac Sim directly and is useless
without joint_commander already running (so101_isaac.launch.py starts it by
default).
"""

from __future__ import annotations

import math
import re
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_isaac.joint_commander import DEFAULT_JOINTS

NUM_CHANNELS = 6

# Matches one "Kênh N: ..." field; findall() walks all six in the line.
CHANNEL_RE = re.compile(r"Kênh\s+(\d+):\s+(?:Raw:\s*(\d+)|Không phát hiện)")

CPR = 4096  # AS5600, 12 bit: 0..4095

# Per-channel hardware calibration, copied from main.py's JOINTS table:
#   ref_raw : raw reading at the reference pose below
#   ref_deg : this channel's physical angle at that reference pose
#   min/max : the channel's physical range of motion, degrees
#   wrap    : True -> angle wraps 0..360 (joint_5, continuous-ish rotation);
#             False -> clamped to [min, max]
#   sign    : +1 if raw increases as the physical angle increases, -1 if
#             the magnet runs backwards (a hardware property, not tunable
#             per deployment -- see `direction` in esp_bridge.yaml for the
#             separate sim-side flip)
# ch0..ch5 line up with joint_names' order: joint_1, 2, 3, 4, 5, gripper_joint.
JOINTS = [
    {"ref_raw": 2233, "ref_deg": 90, "min": 0, "max": 180, "wrap": False, "sign": +1},
    {"ref_raw": 1790, "ref_deg": 90, "min": 0, "max": 180, "wrap": False, "sign": +1},
    {"ref_raw": 2702, "ref_deg": 90, "min": 0, "max": 180, "wrap": False, "sign": +1},
    {"ref_raw": 3396, "ref_deg": 90, "min": 0, "max": 180, "wrap": False, "sign": +1},
    {"ref_raw": 3542, "ref_deg": 180, "min": 0, "max": 360, "wrap": True, "sign": +1},
    {"ref_raw": 3913, "ref_deg": 0, "min": 0, "max": 90, "wrap": False, "sign": +1},
]


def find_port() -> str | None:
    """Prefer the ESP32-S3's native USB VID, then common USB-UART bridges."""
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == 0x303A:
            return p.device
    for p in ports:
        if p.vid in (0x10C4, 0x1A86, 0x0403):  # CP210x, CH340, FTDI
            return p.device
    return ports[0].device if ports else None


def parse_line(line: str) -> dict[int, int]:
    """{channel: raw}; a channel with no sensor attached is left out."""
    return {int(ch): int(raw) for ch, raw in CHANNEL_RE.findall(line) if raw}


def raw_to_angle(ch: int, raw: int) -> float:
    """Raw 12-bit reading -> this channel's physical angle, degrees."""
    j = JOINTS[ch]
    # Offset from the reference raw, wrapped to [-2048, 2048) so a raw that
    # crosses the 4095 -> 0 rollover is still handled correctly.
    delta = (raw - j["ref_raw"] + CPR // 2) % CPR - CPR // 2
    angle = j["ref_deg"] + j["sign"] * delta * 360.0 / CPR
    if j["wrap"]:
        return angle % 360.0
    return max(j["min"], min(j["max"], angle))


class EspJointBridge(Node):
    def __init__(self) -> None:
        super().__init__("esp_joint_bridge")

        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required for esp_joint_bridge (pip install pyserial / apt install python3-serial)"
            ) from exc
        self._serial_module = serial

        self.declare_parameter("port", "")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("joint_names", DEFAULT_JOINTS)
        self.declare_parameter("set_target_topic", "/joint_commander/set_target")
        # Physical angle (post raw_to_angle, degrees) that should map to 0
        # rad -- i.e. the SO-101's own home pose -- on the sim side.
        self.declare_parameter("home_deg", [0.0] * NUM_CHANNELS)
        # +1.0 or -1.0 per channel: flip if the joint moves opposite the
        # URDF's positive direction once mapped into the sim.
        self.declare_parameter("direction", [1.0] * NUM_CHANNELS)

        self.joint_names: list[str] = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != NUM_CHANNELS:
            raise ValueError(f"joint_names must have {NUM_CHANNELS} entries, one per AS5600 channel")

        self.home_deg = [float(v) for v in self.get_parameter("home_deg").value]
        self.direction = [float(v) for v in self.get_parameter("direction").value]
        for param_name, values in (("home_deg", self.home_deg), ("direction", self.direction)):
            if len(values) != NUM_CHANNELS:
                raise ValueError(f"{param_name} must have {NUM_CHANNELS} entries, one per AS5600 channel")

        port = self.get_parameter("port").value or find_port()
        if not port:
            raise RuntimeError("no serial port found for the ESP32; set the 'port' parameter")
        baud = int(self.get_parameter("baud").value)

        # Create the object before opening and drop DTR/RTS first -- opening
        # the port otherwise resets the ESP32 (same as main.py).
        self._ser = serial.Serial()
        self._ser.port = port
        self._ser.baudrate = baud
        self._ser.timeout = 1.0
        self._ser.dtr = False
        self._ser.rts = False
        self._ser.open()

        topic = self.get_parameter("set_target_topic").value
        self._pub = self.create_publisher(JointState, topic, 10)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True, name="esp_joint_bridge-serial")
        self._thread.start()

        self.get_logger().info(f"esp_joint_bridge: {port} @ {baud} baud -> {topic}, joints={self.joint_names}")

    # -- serial read thread ---------------------------------------------

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw_bytes = self._ser.readline()
            except self._serial_module.SerialException as exc:
                self.get_logger().error(f"serial error, stopping read loop: {exc}")
                return
            if not raw_bytes:
                continue
            line = raw_bytes.decode("utf-8", errors="replace").rstrip()
            if not line:
                continue
            data = parse_line(line)
            if not data:
                continue
            try:
                self._publish(data)
            except Exception:
                if self._stop.is_set():
                    return  # shutting down -- the context going away mid-publish is expected
                raise

    def _publish(self, data: dict[int, int]) -> None:
        names = []
        positions = []
        for ch, name in enumerate(self.joint_names):
            raw = data.get(ch)
            if raw is None:
                continue
            deg = raw_to_angle(ch, raw)
            signed = self.direction[ch] * (deg - self.home_deg[ch])
            rad = math.radians(signed)
            rad = math.atan2(math.sin(rad), math.cos(rad))  # wrap to [-pi, pi]
            names.append(name)
            positions.append(rad)
        if not names:
            return

        # joint_commander's ~/set_target clamps to the URDF limits itself, so
        # this node doesn't need to duplicate that check.
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self._pub.publish(msg)

    # -- lifecycle --------------------------------------------------------

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self._ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = EspJointBridge()
    except Exception as exc:
        print(f"[esp_joint_bridge] {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("esp_joint_bridge stopping")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
