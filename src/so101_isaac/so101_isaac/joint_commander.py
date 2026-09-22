#!/usr/bin/env python3
"""Edit the SO-101 joint state that Isaac Sim is simulating.

The node holds one target angle per joint and streams it to Isaac Sim's
``ROS2SubscribeJointState`` node, which feeds the articulation drives.  The
target can be changed three ways while it runs:

1. ``ros2 param set /joint_commander target.joint_2 0.6``
2. publishing a partial ``sensor_msgs/JointState`` on ``~/set_target``::

       ros2 topic pub --once /joint_commander/set_target sensor_msgs/msg/JointState \
           '{name: ["joint_1"], position: [0.8]}'

3. calling the ``~/go_home`` std_srvs/Trigger service.

Targets are rate limited by ``max_speed`` so the arm ramps instead of
snapping, and the first target is seeded from the measured ``/joint_states``
so the arm never jumps on startup.
"""

from __future__ import annotations

import math
import threading

import rclpy
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

DEFAULT_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper_joint"]

# Limits from so101.urdf.xacro, radians. Used to clamp whatever is commanded.
DEFAULT_LIMITS = {
    "joint_1": (-3.14159, 0.90101),
    "joint_2": (-1.20475, 1.93684),
    "joint_3": (-1.55826, 1.23427),
    "joint_4": (-0.210268, 2.58226),
    "joint_5": (-0.79539, 0.78539),
    "gripper_joint": (-0.451427, 0.59577),
}


class JointCommander(Node):
    def __init__(self) -> None:
        super().__init__("joint_commander")

        self.declare_parameter("joint_names", DEFAULT_JOINTS)
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("command_topic", "/joint_command")
        self.declare_parameter("rate_hz", 60.0)
        self.declare_parameter("max_speed", 1.0)  # rad/s, ramp limit
        self.declare_parameter("home", [0.0] * len(DEFAULT_JOINTS))
        self.declare_parameter("clamp_to_limits", True)

        self.joint_names: list[str] = list(self.get_parameter("joint_names").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.clamp = bool(self.get_parameter("clamp_to_limits").value)

        home = list(self.get_parameter("home").value)
        if len(home) != len(self.joint_names):
            home = [0.0] * len(self.joint_names)
        self.home = home

        self._lock = threading.Lock()
        self._measured: dict[str, float] = {}
        self._target: dict[str, float] = dict(zip(self.joint_names, self.home))
        self._command: dict[str, float] | None = None  # ramped output, None until seeded

        # One `target.<joint>` parameter per joint, so `ros2 param set` works.
        for name in self.joint_names:
            lower, upper = self._limits(name)
            self.declare_parameter(
                f"target.{name}",
                self._target[name],
                ParameterDescriptor(
                    description=f"commanded angle for {name}, radians",
                    floating_point_range=[FloatingPointRange(from_value=lower, to_value=upper, step=0.0)],
                ),
            )
        self.add_on_set_parameters_callback(self._on_set_parameters)

        state_topic = self.get_parameter("state_topic").value
        command_topic = self.get_parameter("command_topic").value

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(JointState, command_topic, sensor_qos)
        self.create_subscription(JointState, state_topic, self._on_state, sensor_qos)
        self.create_subscription(JointState, "~/set_target", self._on_set_target, 10)
        self.create_service(Trigger, "~/go_home", self._on_go_home)

        self._timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f"joint_commander: {state_topic} -> {command_topic}, joints={self.joint_names}, "
            f"max_speed={self.max_speed} rad/s"
        )

    # -- helpers ------------------------------------------------------------

    def _limits(self, name: str) -> tuple[float, float]:
        return DEFAULT_LIMITS.get(name, (-math.pi, math.pi))

    def _clamp(self, name: str, value: float) -> float:
        if not self.clamp:
            return value
        lower, upper = self._limits(name)
        return min(max(value, lower), upper)

    # -- inputs -------------------------------------------------------------

    def _on_state(self, msg: JointState) -> None:
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._measured[name] = position
            if self._command is None and all(n in self._measured for n in self.joint_names):
                # Seed the ramp from where the arm actually is.
                self._command = {n: self._measured[n] for n in self.joint_names}
                self.get_logger().info("seeded command from measured joint state")

    def _on_set_target(self, msg: JointState) -> None:
        if not msg.name or len(msg.position) < len(msg.name):
            self.get_logger().warn("set_target ignored: needs matching name[] and position[]")
            return
        updated = {}
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                if name not in self._target:
                    self.get_logger().warn(f"set_target: unknown joint '{name}', ignored")
                    continue
                value = self._clamp(name, float(position))
                self._target[name] = value
                updated[name] = value
        if updated:
            self.set_parameters(
                [rclpy.parameter.Parameter(f"target.{n}", rclpy.Parameter.Type.DOUBLE, v) for n, v in updated.items()]
            )

    def _on_set_parameters(self, params) -> SetParametersResult:
        with self._lock:
            for param in params:
                if not param.name.startswith("target."):
                    continue
                name = param.name[len("target.") :]
                if name not in self._target:
                    return SetParametersResult(successful=False, reason=f"unknown joint '{name}'")
                lower, upper = self._limits(name)
                value = float(param.value)
                if self.clamp and not (lower <= value <= upper):
                    return SetParametersResult(
                        successful=False, reason=f"{name}={value:.4f} outside limits [{lower:.4f}, {upper:.4f}]"
                    )
                self._target[name] = value
        return SetParametersResult(successful=True)

    def _on_go_home(self, _request, response) -> Trigger.Response:
        with self._lock:
            self._target = dict(zip(self.joint_names, self.home))
        response.success = True
        response.message = f"target reset to home {self.home}"
        return response

    # -- output -------------------------------------------------------------

    def _tick(self) -> None:
        step = self.max_speed / self.rate_hz
        with self._lock:
            if self._command is None:
                return  # wait until /joint_states tells us where the arm is
            names = list(self.joint_names)
            positions = []
            for name in names:
                current = self._command[name]
                desired = self._target[name]
                delta = desired - current
                if step > 0.0 and abs(delta) > step:
                    current += math.copysign(step, delta)
                else:
                    current = desired
                self._command[name] = current
                positions.append(current)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C, or the context was shut down from outside (ros2 launch, a
        # SIGTERM from a test harness). Both are ordinary ways to stop.
        node.get_logger().info("joint_commander stopping")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
