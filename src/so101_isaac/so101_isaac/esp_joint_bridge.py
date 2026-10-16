#!/usr/bin/env python3
"""Bridge ESP32-S3 (PCA9548A + 6x AS5600) angle readings into joint_commander.

The firmware prints all six AS5600 channels on one line, e.g.::

    Kênh 0: Raw: 1234 | Kênh 1: Không phát hiện AS5600 | ...

This node parses that line and converts each channel's raw 12-bit reading to
a physical joint angle using the JOINTS calibration table below. It started
as a port of /home/cuong-phan/test/main.py's JOINTS table (the standalone
log reader used to calibrate the hardware) but has since diverged: that
script still uses a single reference point per channel, which has a built-in
discontinuity (see raw_to_angle()'s docstring) that this file's two-point
calibration avoids. Re-calibrating a channel means updating its raw_a/deg_a
and raw_b/deg_b here directly (two measured points), not copying from
main.py.

That physical angle is then linearly remapped from the channel's whole
measured [min, max] sensor sweep (JOINTS) onto its URDF joint's whole
[lower, upper] limit (joint_commander.DEFAULT_LIMITS, degrees), oriented by
`direction` (config/esp_bridge.yaml) if the magnet runs opposite the URDF's
positive direction. so101.urdf (2026-10-12: swapped for the SML lab's
rebuilt model) takes its joint limits straight from each servo's real
mechanical travel, which is exactly what each channel's [min, max] was
calibrated against -- so every channel's sensor span matches its URDF span
by construction, and this mapping is 1:1 real-to-sim motion with no
clamping at either end. (An earlier version of this scheme instead
re-centred on a home_deg reading and flipped by direction, which only works
when the URDF limit happens to sit +/-90 deg symmetric around home; that
stopped being reliably true across models, so it's gone.)

The result is republished on joint_commander's ``~/set_target`` topic, so
joint_commander's existing ramp limiting and limit clamping
(joint_commander.py) smooths it before anything reaches Isaac Sim's
/joint_command. This node never talks to Isaac Sim directly and is useless
without joint_commander already running (so101_isaac.launch.py starts it by
default).

It also republishes the same per-channel data pre home/direction, raw counts
and physical degrees, on ``~/encoder_raw`` -- that's what encoder_panel.py
reads to show live sensor values in a terminal.
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

from so101_isaac.joint_commander import DEFAULT_JOINTS, DEFAULT_LIMITS

NUM_CHANNELS = 6

# Matches one "Kênh N: ..." field; findall() walks all six in the line.
CHANNEL_RE = re.compile(r"Kênh\s+(\d+):\s+(?:Raw:\s*(\d+)|Không phát hiện)")

CPR = 4096  # AS5600, 12 bit: 0..4095

# Per-channel hardware calibration: two measured (raw, degree) points each,
# "a" and "b" -- raw_to_angle() does a straight line through them. Replaces
# the old single-reference-point + assumed-symmetric-range scheme, which
# only approximated a channel's real raw endpoints (and, independently, had
# the unwrap bug raw_to_angle()'s docstring used to describe).
#
#   raw_a/deg_a, raw_b/deg_b : two measured raw readings and the physical
#                              angle at each. raw_b need not be the smaller
#                              or larger of the two -- the slope's sign comes
#                              out of whichever way the real data runs.
#                              raw_b may be given "unwrapped" (e.g. 4406
#                              instead of 310) when the channel's real travel
#                              crosses the AS5600's 4095 -> 0 rollover; see
#                              EspJointBridge._unwrap for how that's tracked.
#   min/max : the channel's physical range of motion, degrees -- the result
#             is clamped to this unless wrap is set
#   wrap    : True -> angle wraps 0..360 (joint_5, continuous rotation);
#             False -> clamped to [min, max]
#
# ch0..ch5 line up with joint_names' order: joint_1, 2, 3, 4, 5, gripper_joint.
JOINTS = [
    # joint_1 -- derived (ref_raw=2233 at 90 deg, assumed symmetric +/-90 deg); not re-measured.
    {"raw_a": 1209, "deg_a": 0.0, "raw_b": 3257, "deg_b": 180.0, "min": 0, "max": 180, "wrap": False},
    # joint_2 -- derived (ref_raw=1790 at 90 deg, assumed symmetric); not re-measured.
    {"raw_a": 766, "deg_a": 0.0, "raw_b": 2814, "deg_b": 180.0, "min": 0, "max": 180, "wrap": False},
    # joint_3 -- derived (ref_raw=2702 at 90 deg, assumed symmetric); not re-measured.
    {"raw_a": 1678, "deg_a": 0.0, "raw_b": 3726, "deg_b": 180.0, "min": 0, "max": 180, "wrap": False},
    # joint_4 -- measured 2026-10: raw=2300 at 0 deg, raw=310 at 180 deg, travel
    # crosses the AS5600 rollover (2300 -> 4095 -> 0 -> 310), so raw_b is the
    # unwrapped 310 + 4096.
    {"raw_a": 2300, "deg_a": 0.0, "raw_b": 310 + CPR, "deg_b": 180.0, "min": 0, "max": 180, "wrap": False},
    # joint_5 -- continuous rotation, no real endpoints to measure; raw_b is a
    # synthetic point one full AS5600 turn (4096 raw = 360 deg) past raw_a,
    # same as the old ref_raw/ref_deg + 360/CPR-per-count scheme.
    {"raw_a": 3542, "deg_a": 180.0, "raw_b": 3542 + CPR, "deg_b": 180.0 + 360.0, "min": 0, "max": 360, "wrap": True},
    # gripper_joint -- measured 2026-10: raw=3913 fully closed (0 deg), raw=2850
    # at 90 deg open (raw decreases as the gripper opens).
    {"raw_a": 3913, "deg_a": 0.0, "raw_b": 2850, "deg_b": 90.0, "min": 0, "max": 90, "wrap": False},
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


def raw_to_angle(ch: int, unwrapped_raw: float) -> float:
    """Continuous (unwrapped) raw reading -> this channel's physical angle, degrees.

    `unwrapped_raw` must already be continuous (see EspJointBridge._unwrap) --
    this function does a plain straight-line map through JOINTS[ch]'s two
    calibration points, no modulo. Computing delta as
    `(raw - raw_a + CPR//2) % CPR - CPR//2` (shortest path straight from a
    fixed point) looks equivalent but has exactly one discontinuity, 180 deg
    raw-wise opposite raw_a: a joint whose travel reaches that point gets a
    sudden jump in its computed angle even though nothing is wrong with the
    sensor itself. Unwrapping against the previous sample instead of a fixed
    point (see _unwrap) avoids that entirely, including at the AS5600's own
    4095 -> 0 rollover.
    """
    j = JOINTS[ch]
    scale = (j["deg_b"] - j["deg_a"]) / (j["raw_b"] - j["raw_a"])
    angle = j["deg_a"] + scale * (unwrapped_raw - j["raw_a"])
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
        # Raw AS5600 counts (effort[]) and the physical degree they decode to
        # (position[]), pre home_deg/direction -- for encoder_panel.py, not
        # consumed by joint_commander.
        self.declare_parameter("encoder_topic", "~/encoder_raw")
        # +1.0 or -1.0 per channel: whether the channel's measured [min, max]
        # sweep maps onto the URDF joint's [lower, upper] limit in that order
        # (+1.0) or reversed (-1.0) -- flip if the magnet runs opposite the
        # URDF's positive direction.
        self.declare_parameter("direction", [1.0] * NUM_CHANNELS)

        self.joint_names: list[str] = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != NUM_CHANNELS:
            raise ValueError(f"joint_names must have {NUM_CHANNELS} entries, one per AS5600 channel")

        self.direction = [float(v) for v in self.get_parameter("direction").value]
        if len(self.direction) != NUM_CHANNELS:
            raise ValueError(f"direction must have {NUM_CHANNELS} entries, one per AS5600 channel")

        # Each channel's URDF joint limit, degrees, for _sim_target_deg().
        self._urdf_limits_deg: list[tuple[float, float]] = []
        for name in self.joint_names:
            lower_rad, upper_rad = DEFAULT_LIMITS.get(name, (-math.pi, math.pi))
            self._urdf_limits_deg.append((math.degrees(lower_rad), math.degrees(upper_rad)))

        # Per-channel continuous raw position, see _unwrap(). Read/written
        # only from _read_loop's thread, so no lock needed.
        self._unwrapped: dict[int, float] = {}

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

        encoder_topic = self.get_parameter("encoder_topic").value
        self._encoder_pub = self.create_publisher(JointState, encoder_topic, 10)

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

    def _unwrap(self, ch: int, raw: int) -> float:
        """Fold `raw` onto a continuous (not 0..4095-wrapped) track for channel ch.

        Anchored on the previous sample (or JOINTS[ch]["raw_a"] for the very
        first one) rather than always on a fixed point -- see raw_to_angle()'s
        docstring for why a fixed anchor breaks.
        """
        anchor = self._unwrapped.get(ch, float(JOINTS[ch]["raw_a"]))
        delta = (raw - anchor + CPR / 2) % CPR - CPR / 2
        unwrapped = anchor + delta
        self._unwrapped[ch] = unwrapped
        return unwrapped

    def _sim_target_deg(self, ch: int, deg: float) -> float:
        """Map a channel's physical-sensor degree reading onto its URDF joint's
        own degree range: linearly remap the channel's whole measured
        [min, max] sensor sweep (JOINTS) onto its whole URDF [lower, upper]
        limit, oriented by direction[ch]. See the module docstring for why
        this is a 1:1 mapping with no clamping at either end.
        """
        j = JOINTS[ch]
        sensor_lo, sensor_hi = j["min"], j["max"]
        urdf_lo, urdf_hi = self._urdf_limits_deg[ch]
        target_lo, target_hi = (urdf_lo, urdf_hi) if self.direction[ch] >= 0 else (urdf_hi, urdf_lo)
        frac = (deg - sensor_lo) / (sensor_hi - sensor_lo)
        return target_lo + frac * (target_hi - target_lo)

    def _publish(self, data: dict[int, int]) -> None:
        names = []
        positions = []
        encoder_names = []
        encoder_deg = []
        encoder_raw = []
        for ch, name in enumerate(self.joint_names):
            raw = data.get(ch)
            if raw is None:
                continue
            deg = raw_to_angle(ch, self._unwrap(ch, raw))
            encoder_names.append(name)
            encoder_deg.append(deg)
            encoder_raw.append(float(raw))

            target_deg = self._sim_target_deg(ch, deg)
            rad = math.radians(target_deg)
            rad = math.atan2(math.sin(rad), math.cos(rad))  # wrap to [-pi, pi]
            names.append(name)
            positions.append(rad)

        if encoder_names:
            encoder_msg = JointState()
            encoder_msg.header.stamp = self.get_clock().now().to_msg()
            encoder_msg.name = encoder_names
            encoder_msg.position = encoder_deg  # physical degrees, pre home/direction
            encoder_msg.effort = encoder_raw  # raw AS5600 counts, 0..4095
            self._encoder_pub.publish(encoder_msg)

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
