"""Integration test for joint_commander, with no simulator involved.

Stands in for Isaac Sim: publishes /joint_states, watches /joint_command, and
exercises every way of changing the target. Start joint_commander first, then
run this. ``test/run_integration.sh`` does both.
"""
import sys, time, threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper_joint"]


class FakeIsaac(Node):
    def __init__(self):
        super().__init__("fake_isaac")
        self.state = dict.fromkeys(JOINTS, 0.0)
        self.last_cmd = None
        self.cmd_count = 0
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/joint_command", self._on_cmd, 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _on_cmd(self, msg):
        self.cmd_count += 1
        self.last_cmd = dict(zip(msg.name, msg.position))
        # Pretend the drives track the command perfectly.
        self.state.update(self.last_cmd)

    def _tick(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = JOINTS
        m.position = [self.state[j] for j in JOINTS]
        m.velocity = [0.0] * len(JOINTS)
        self.pub.publish(m)


def spin_for(exe, seconds):
    end = time.time() + seconds
    while time.time() < end:
        exe.spin_once(timeout_sec=0.05)


def main():
    rclpy.init()
    fake = FakeIsaac()
    from rclpy.executors import SingleThreadedExecutor
    exe = SingleThreadedExecutor()
    exe.add_node(fake)

    print("waiting for joint_commander ...")
    for _ in range(200):
        exe.spin_once(timeout_sec=0.05)
        if fake.cmd_count > 0:
            break
    assert fake.cmd_count > 0, "FAIL: no /joint_command received -- commander did not seed"
    print(f"PASS seeding: got {fake.cmd_count} commands, first = {fake.last_cmd}")

    # --- 1. set a target via parameter -----------------------------------
    cli = fake.create_client(SetParameters, "/joint_commander/set_parameters")
    ok = False
    for _ in range(100):
        exe.spin_once(timeout_sec=0.05)
        if cli.service_is_ready():
            ok = True
            break
    assert ok, "FAIL: /joint_commander/set_parameters never appeared"

    req = SetParameters.Request()
    req.parameters = [Parameter("target.joint_2", Parameter.Type.DOUBLE, 0.6).to_parameter_msg()]
    fut = cli.call_async(req)
    while not fut.done():
        exe.spin_once(timeout_sec=0.05)
    res = fut.result().results[0]
    assert res.successful, f"FAIL: set_parameters rejected: {res.reason}"
    print("PASS param set accepted")

    spin_for(exe, 3.0)
    got = fake.last_cmd["joint_2"]
    assert abs(got - 0.6) < 1e-3, f"FAIL: joint_2 did not reach 0.6, got {got}"
    print(f"PASS ramp to target: joint_2 = {got:.5f}")

    # --- 2. out-of-limit target must be rejected --------------------------
    req = SetParameters.Request()
    req.parameters = [Parameter("target.joint_5", Parameter.Type.DOUBLE, 5.0).to_parameter_msg()]
    fut = cli.call_async(req)
    while not fut.done():
        exe.spin_once(timeout_sec=0.05)
    res = fut.result().results[0]
    assert not res.successful, "FAIL: out-of-limit target was accepted"
    print(f"PASS limit rejection: {res.reason}")

    # --- 3. set_target topic ---------------------------------------------
    pub = fake.create_publisher(JointState, "/joint_commander/set_target", 10)
    spin_for(exe, 0.5)
    m = JointState()
    m.name = ["joint_1", "gripper_joint"]
    m.position = [-0.8, 0.5]
    pub.publish(m)
    spin_for(exe, 3.0)
    j1, gj = fake.last_cmd["joint_1"], fake.last_cmd["gripper_joint"]
    assert abs(j1 + 0.8) < 1e-3, f"FAIL: joint_1 = {j1}"
    assert abs(gj - 0.5) < 1e-3, f"FAIL: gripper_joint = {gj}"
    print(f"PASS set_target topic: joint_1 = {j1:.5f}, gripper_joint = {gj:.5f}")

    # --- 4. go_home service -----------------------------------------------
    home = fake.create_client(Trigger, "/joint_commander/go_home")
    for _ in range(100):
        exe.spin_once(timeout_sec=0.05)
        if home.service_is_ready():
            break
    fut = home.call_async(Trigger.Request())
    while not fut.done():
        exe.spin_once(timeout_sec=0.05)
    assert fut.result().success, "FAIL: go_home failed"
    spin_for(exe, 3.0)
    far = max(abs(v) for v in fake.last_cmd.values())
    assert far < 1e-3, f"FAIL: not home, max |angle| = {far}"
    print(f"PASS go_home: max |angle| = {far:.6f}")

    # --- 5. rate limiting --------------------------------------------------
    req = SetParameters.Request()
    req.parameters = [Parameter("target.joint_1", Parameter.Type.DOUBLE, -3.0).to_parameter_msg()]
    fut = cli.call_async(req)
    while not fut.done():
        exe.spin_once(timeout_sec=0.05)
    spin_for(exe, 0.5)   # max_speed 1.0 rad/s -> ~0.5 rad after 0.5 s, not -3.0
    j1 = fake.last_cmd["joint_1"]
    assert -1.2 < j1 < -0.2, f"FAIL: expected a ramp around -0.5 after 0.5 s, got {j1}"
    print(f"PASS rate limiting: joint_1 = {j1:.5f} after 0.5 s (not -3.0)")

    print("\nALL CHECKS PASSED")
    fake.destroy_node()
    rclpy.shutdown()


main()
