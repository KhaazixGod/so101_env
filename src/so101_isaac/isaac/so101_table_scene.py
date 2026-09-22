#!/usr/bin/env python3
"""Standalone Isaac Sim app: SO-101 arm standing on a table, bridged to ROS 2.

Builds the scene procedurally (no Nucleus / online assets needed):

    /World
      /World/PhysicsScene
      /World/Ground          static box floor
      /World/Table           table top + 4 legs, static colliders
      /World/so101           reference to so101.usda (Physics variant = physx)
      /World/ActionGraph     ROS 2 bridge graph

ROS 2 topics created by the action graph:

    /joint_states   (sensor_msgs/JointState)  published  -- arm state out
    /joint_command  (sensor_msgs/JointState)  subscribed -- arm targets in
    /clock          (rosgraph_msgs/Clock)     published
    /tf             (tf2_msgs/TFMessage)      published  (--publish-tf)

Run it with the Isaac Sim python, in a shell where ROS 2 Jazzy is sourced:

    source /opt/ros/jazzy/setup.bash
    /home/pread/env_isaac/bin/python3 so101_table_scene.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys

# ---------------------------------------------------------------------------
# Joints of the SO-101, in the order the URDF/USD declares them.
# ---------------------------------------------------------------------------
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper_joint"]

# Prim paths, relative to where we reference the robot onto the stage.
ROBOT_PRIM = "/World/so101"
ARTICULATION_ROOT = f"{ROBOT_PRIM}/Geometry/world/base_link"
JOINTS_SCOPE = f"{ROBOT_PRIM}/Physics"

DEFAULT_USD = os.path.normpath(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "..",
        "so101_description",
        "urdf",
        "so101",
        "so101.usda",
    )
)


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    p.add_argument("--usd", default=DEFAULT_USD, help="path to so101.usda (default: sibling so101_description package)")
    p.add_argument("--headless", action="store_true", help="run without the Isaac Sim UI")

    # --- table -------------------------------------------------------------
    p.add_argument("--table-height", type=float, default=0.74, help="height of the table top surface, m")
    p.add_argument("--table-size", type=float, nargs=2, default=(1.20, 0.80), metavar=("LEN", "WIDTH"))
    p.add_argument("--table-thickness", type=float, default=0.04)

    # --- robot placement ---------------------------------------------------
    p.add_argument("--robot-xy", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"),
                   help="position of the robot on the table top, relative to the table centre")
    p.add_argument("--robot-yaw", type=float, default=0.0, help="robot heading on the table, degrees")
    p.add_argument("--no-auto-place", action="store_true",
                   help="do not snap the robot down onto the table using its bounding box")

    # --- drives ------------------------------------------------------------
    p.add_argument("--stiffness", type=float, default=60.0,
                   help="angular drive stiffness (USD units: force*m per DEGREE)")
    p.add_argument("--damping", type=float, default=4.0,
                   help="angular drive damping (USD units: force*m per DEGREE/s)")
    p.add_argument("--max-force", type=float, default=None,
                   help="override drive maxForce (Nm); default keeps the value authored in the USD")
    p.add_argument("--home", type=float, nargs=len(JOINT_NAMES), default=[0.0] * len(JOINT_NAMES),
                   metavar="RAD", help=f"initial pose, radians, in the order {JOINT_NAMES}")

    # --- ros 2 -------------------------------------------------------------
    p.add_argument("--state-topic", default="/joint_states")
    p.add_argument("--command-topic", default="/joint_command")
    p.add_argument("--node-namespace", default="")
    p.add_argument("--domain-id", type=int, default=None,
                   help="ROS_DOMAIN_ID; default reads the environment variable")
    p.add_argument("--no-publish-tf", action="store_true", help="skip the /tf publisher")
    p.add_argument("--no-publish-clock", action="store_true", help="skip the /clock publisher")

    # --- misc --------------------------------------------------------------
    p.add_argument("--physics-hz", type=float, default=120.0)
    p.add_argument("--save-usd", default=None, help="write the assembled scene to this .usd/.usda and keep running")
    p.add_argument("--max-steps", type=int, default=0,
                   help="quit after this many simulation steps; 0 runs until the app is closed")

    args, _unknown = p.parse_known_args(argv)
    return args


ARGS = parse_args(sys.argv[1:])

if not os.path.isfile(ARGS.usd):
    sys.exit(f"[so101_table_scene] robot USD not found: {ARGS.usd}\nPass one with --usd /path/to/so101.usda")

# ---------------------------------------------------------------------------
# SimulationApp must be constructed before any other Isaac Sim / omni import.
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": ARGS.headless})

import carb  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

try:
    # Registered by the PhysX schema extension, so it only exists once kit is up.
    from pxr import PhysxSchema  # noqa: E402
except ImportError:  # pragma: no cover
    PhysxSchema = None


def enable_ros2_bridge() -> None:
    """Turn on the ROS 2 bridge extension and fail loudly if it cannot load."""
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    simulation_app.update()
    if not manager.is_extension_enabled("isaacsim.ros2.bridge"):
        carb.log_error(
            "isaacsim.ros2.bridge failed to load. Either source a ROS 2 Jazzy install\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "or point LD_LIBRARY_PATH at the Jazzy libraries bundled with Isaac Sim\n"
            "  export ROS_DISTRO=jazzy RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
            "  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<isaacsim>/exts/isaacsim.ros2.core/jazzy/lib\n"
            "run_scene.sh does this for you."
        )
        simulation_app.close()
        sys.exit(1)


def make_physics_scene(stage: Usd.Stage, path: str, hz: float) -> None:
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path(path))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    if PhysxSchema is None:
        carb.log_warn("PhysxSchema unavailable, leaving PhysX solver settings at their defaults")
        return
    physx = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(path))
    physx.CreateEnableCCDAttr().Set(True)
    physx.CreateEnableStabilizationAttr().Set(True)
    physx.CreateSolverTypeAttr().Set("TGS")
    physx.CreateTimeStepsPerSecondAttr().Set(int(hz))


def make_physics_material(stage: Usd.Stage, path: str, static_fr: float, dynamic_fr: float, restitution: float):
    UsdShade.Material.Define(stage, Sdf.Path(path))
    prim = stage.GetPrimAtPath(path)
    material = UsdPhysics.MaterialAPI.Apply(prim)
    material.CreateStaticFrictionAttr().Set(static_fr)
    material.CreateDynamicFrictionAttr().Set(dynamic_fr)
    material.CreateRestitutionAttr().Set(restitution)
    return prim


def make_box(
    stage: Usd.Stage,
    path: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    color: tuple[float, float, float],
    physics_material=None,
) -> Usd.Prim:
    """A unit cube scaled into a static collider box."""
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.CreateSizeAttr(1.0)  # spans -0.5..0.5, so scale == real dimensions
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])

    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))

    prim = cube.GetPrim()
    # No RigidBodyAPI -> PhysX treats it as an immovable static collider.
    UsdPhysics.CollisionAPI.Apply(prim)
    if physics_material is not None:
        binding = UsdShade.MaterialBindingAPI.Apply(prim)
        binding.Bind(
            UsdShade.Material(physics_material),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics",
        )
    return prim


def build_table(stage: Usd.Stage, args: argparse.Namespace, physics_material) -> None:
    length, width = args.table_size
    top_z = args.table_height
    thickness = args.table_thickness
    leg = 0.06
    leg_height = top_z - thickness

    stage.DefinePrim("/World/Table", "Xform")

    make_box(
        stage,
        "/World/Table/Top",
        size=(length, width, thickness),
        center=(0.0, 0.0, top_z - thickness / 2.0),
        color=(0.72, 0.56, 0.38),
        physics_material=physics_material,
    )

    inset = leg / 2.0 + 0.03
    corners = (
        (length / 2.0 - inset, width / 2.0 - inset),
        (length / 2.0 - inset, -(width / 2.0 - inset)),
        (-(length / 2.0 - inset), width / 2.0 - inset),
        (-(length / 2.0 - inset), -(width / 2.0 - inset)),
    )
    for index, (x, y) in enumerate(corners):
        make_box(
            stage,
            f"/World/Table/Leg_{index}",
            size=(leg, leg, leg_height),
            center=(x, y, leg_height / 2.0),
            color=(0.45, 0.33, 0.22),
            physics_material=physics_material,
        )


def build_lighting(stage: Usd.Stage) -> None:
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Lights/DomeLight"))
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.96, 1.0))

    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/KeyLight"))
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(1.0)
    key_xform = UsdGeom.Xformable(key)
    key_xform.ClearXformOpOrder()
    key_xform.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))


def add_robot(stage: Usd.Stage, usd_path: str) -> Usd.Prim:
    prim = stage.DefinePrim(ROBOT_PRIM, "Xform")
    prim.GetReferences().AddReference(os.path.abspath(usd_path))

    variants = prim.GetVariantSets()
    if variants.HasVariantSet("Physics"):
        variants.GetVariantSet("Physics").SetVariantSelection("physx")
    return prim


def place_robot(stage: Usd.Stage, prim: Usd.Prim, args: argparse.Namespace) -> None:
    """Put the robot on the table, optionally snapping it down with its bbox."""
    x, y = args.robot_xy
    z = args.table_height

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(x, y, z))
    xform.AddRotateZOp().Set(float(args.robot_yaw))

    if args.no_auto_place:
        return

    # The robot's base_link origin is not its lowest point (the URDF rotates the
    # model by 90 deg about X), so measure the loaded geometry and drop it until
    # the lowest vertex touches the table top.
    for _ in range(60):
        simulation_app.update()

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    if box.IsEmpty():
        carb.log_warn("[so101_table_scene] empty bounding box, leaving the robot at the nominal height")
        return

    z_min = box.GetMin()[2]
    corrected_z = z + (args.table_height - z_min)
    translate_op.Set(Gf.Vec3d(x, y, corrected_z))
    carb.log_info(f"[so101_table_scene] auto-placed robot at z={corrected_z:.4f} (bbox min was {z_min:.4f})")


def configure_drives(stage: Usd.Stage, args: argparse.Namespace) -> None:
    """Give every angular drive a stiffness so position commands actually hold.

    The USD shipped with so101_description authors damping and maxForce but no
    stiffness, which makes the drives behave like pure dampers -- the arm would
    sag under gravity and ignore /joint_command.
    """
    for name, home_rad in zip(JOINT_NAMES, args.home):
        path = f"{JOINTS_SCOPE}/{name}"
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            carb.log_error(f"[so101_table_scene] joint prim missing: {path}")
            continue

        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")

        home_deg = math.degrees(home_rad)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(args.stiffness)
        drive.CreateDampingAttr().Set(args.damping)
        drive.CreateTargetPositionAttr().Set(home_deg)
        drive.CreateTargetVelocityAttr().Set(0.0)
        if args.max_force is not None:
            drive.CreateMaxForceAttr().Set(args.max_force)

        # PhysicsJointStateAPI:angular is already applied in physics.usda, so we
        # only have to author the values to start the arm in the home pose.
        prim.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(home_deg)
        prim.CreateAttribute("state:angular:physics:velocity", Sdf.ValueTypeNames.Float).Set(0.0)


def build_action_graph(args: argparse.Namespace) -> None:
    """ROS 2 bridge graph: publish /joint_states, subscribe /joint_command."""
    keys = og.Controller.Keys

    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
    ]
    connections = [
        # state out: articulation -> /joint_states
        ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
        ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
        ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
        ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
        ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
        ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
        ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
        ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
        ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
        ("Context.outputs:context", "PublishJointState.inputs:context"),
        # command in: /joint_command -> articulation drives
        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
        ("Context.outputs:context", "SubscribeJointState.inputs:context"),
        ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
        ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
        ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
    ]
    values = [
        ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(ARTICULATION_ROOT)]),
        ("ArticulationController.inputs:robotPath", ARTICULATION_ROOT),
        ("PublishJointState.inputs:topicName", args.state_topic),
        ("PublishJointState.inputs:nodeNamespace", args.node_namespace),
        ("SubscribeJointState.inputs:topicName", args.command_topic),
        ("SubscribeJointState.inputs:nodeNamespace", args.node_namespace),
    ]

    if args.domain_id is None:
        values.append(("Context.inputs:useDomainIDEnvVar", True))
    else:
        values.append(("Context.inputs:useDomainIDEnvVar", False))
        values.append(("Context.inputs:domain_id", args.domain_id))

    if not args.no_publish_clock:
        nodes.append(("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"))
        connections += [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ]

    if not args.no_publish_tf:
        nodes.append(("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"))
        connections += [
            ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
            ("Context.outputs:context", "PublishTF.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
        ]
        values += [
            ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(ARTICULATION_ROOT)]),
            ("PublishTF.inputs:nodeNamespace", args.node_namespace),
        ]

    og.Controller.edit(
        {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: nodes,
            keys.CONNECT: connections,
            keys.SET_VALUES: values,
        },
    )


def main() -> None:
    args = ARGS
    enable_ros2_bridge()

    omni.usd.get_context().new_stage()
    simulation_app.update()
    stage = omni.usd.get_context().get_stage()

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(world)

    make_physics_scene(stage, "/World/PhysicsScene", args.physics_hz)
    grip_material = make_physics_material(stage, "/World/PhysicsMaterials/Grippy", 0.9, 0.8, 0.0)

    make_box(
        stage,
        "/World/Ground",
        size=(20.0, 20.0, 0.10),
        center=(0.0, 0.0, -0.05),
        color=(0.30, 0.32, 0.35),
        physics_material=grip_material,
    )
    build_table(stage, args, grip_material)
    build_lighting(stage)

    robot = add_robot(stage, args.usd)
    simulation_app.update()
    place_robot(stage, robot, args)
    configure_drives(stage, args)
    simulation_app.update()

    build_action_graph(args)
    simulation_app.update()

    if args.save_usd:
        stage.GetRootLayer().Export(os.path.abspath(args.save_usd))
        print(f"[so101_table_scene] scene written to {os.path.abspath(args.save_usd)}", flush=True)

    print(
        "[so101_table_scene] running.\n"
        f"  publishing : {args.state_topic}\n"
        f"  subscribing: {args.command_topic}\n"
        f"  joints     : {JOINT_NAMES}",
        flush=True,
    )

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    steps = 0
    try:
        while simulation_app.is_running():
            simulation_app.update()
            steps += 1
            if args.max_steps and steps >= args.max_steps:
                print(f"[so101_table_scene] reached --max-steps {args.max_steps}, quitting")
                break
    except KeyboardInterrupt:
        pass
    finally:
        timeline.stop()
        simulation_app.close()


main()
