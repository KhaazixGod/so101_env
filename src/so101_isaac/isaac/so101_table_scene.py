#!/usr/bin/env python3
"""Standalone Isaac Sim app: SO-101 arm standing on a table, bridged to ROS 2.

Builds the scene procedurally (no Nucleus / online assets needed):

    /World
      /World/PhysicsScene
      /World/Ground          static box floor
      /World/Table           table top + 4 legs, static colliders
      /World/Conveyor        belt (kinematic, PhysX surface velocity) + 4 legs, against --table-edge
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

import yaml

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

# so101_isaac/config/joint_drives.yaml, next to so101_isaac/isaac/this file.
DEFAULT_DRIVE_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "joint_drives.yaml")
)

# Fallback gains used when no --drive-config is given/found and a joint has no
# --joint-drive override: matches the historical global --stiffness/--damping.
HARDCODED_DEFAULT_GAINS = {"stiffness": 60.0, "damping": 4.0, "max_force": None}


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    p.add_argument("--usd", default=DEFAULT_USD, help="path to so101.usda (default: sibling so101_description package)")
    p.add_argument("--headless", action="store_true", help="run without the Isaac Sim UI")

    # --- table -------------------------------------------------------------
    p.add_argument("--table-height", type=float, default=0.74, help="height of the table top surface, m")
    p.add_argument("--table-size", type=float, nargs=2, default=(1.20, 0.80), metavar=("LEN", "WIDTH"))
    p.add_argument("--table-thickness", type=float, default=0.04)

    # --- table edge ----------------------------------------------------------
    # Which edge "front/back/left/right" refers to, and where the robot and
    # conveyor default to. front/back are the two LONG edges (run along X, at
    # y = -W/2 / +W/2); left/right are the two SHORT edges (run along Y, at
    # x = -L/2 / +L/2). Default table size is 1.20 x 0.80 (X x Y), so front/back
    # are the "horizontal" edges (canh ngang) in a landscape view of the table.
    p.add_argument("--table-edge", choices=("front", "back", "left", "right"), default="front",
                   help="the table edge the robot and conveyor default to (front/back = the long "
                        "edges at y=-W/2 / y=+W/2; left/right = the short edges at x=-L/2 / x=+L/2)")

    # --- robot placement ---------------------------------------------------
    p.add_argument("--robot-xy", type=float, nargs=2, default=None, metavar=("X", "Y"),
                   help="position of the robot on the table top, relative to the table centre; "
                        "default: right at --table-edge, offset inward by --robot-edge-margin")
    p.add_argument("--robot-edge-margin", type=float, default=0.08,
                   help="how far in from --table-edge the default --robot-xy sits, m")
    p.add_argument("--robot-yaw", type=float, default=0.0, help="robot heading on the table, degrees")
    p.add_argument("--no-auto-place", action="store_true",
                   help="do not snap the robot down onto the table using its bounding box")

    # --- drives ------------------------------------------------------------
    # Precedence, low to high:
    #   1. HARDCODED_DEFAULT_GAINS
    #   2. --drive-config's "default:" section
    #   3. --stiffness / --damping / --max-force (override the effective default)
    #   4. --drive-config's "joints: <name>:" section (per joint)
    #   5. --joint-drive (per joint, repeatable, highest precedence)
    p.add_argument("--drive-config", default=DEFAULT_DRIVE_CONFIG,
                   help="YAML file with default/per-joint stiffness, damping, max_force "
                        "(default: so101_isaac/config/joint_drives.yaml; pass an empty "
                        "string to skip it entirely)")
    p.add_argument("--stiffness", type=float, default=None,
                   help="angular drive stiffness for every joint (USD units: force*m per DEGREE); "
                        "overrides --drive-config's default section")
    p.add_argument("--damping", type=float, default=None,
                   help="angular drive damping for every joint (USD units: force*m per DEGREE/s); "
                        "overrides --drive-config's default section")
    p.add_argument("--max-force", type=float, default=None,
                   help="drive maxForce for every joint (Nm); overrides --drive-config's default section; "
                        "with neither this nor a config value, keeps whatever the source USD authors")
    p.add_argument("--joint-drive", action="append", default=[], metavar="JOINT:key=val[,key=val...]",
                   help="per-joint override, e.g. --joint-drive joint_1:stiffness=80,damping=5 "
                        "-- repeatable, takes precedence over --drive-config and the global flags")
    p.add_argument("--home", type=float, nargs=len(JOINT_NAMES), default=[0.0] * len(JOINT_NAMES),
                   metavar="RAD", help=f"initial pose, radians, in the order {JOINT_NAMES}")

    # --- conveyor belt -------------------------------------------------------
    # Built with isaacsim.asset.gen.conveyor's create_conveyor_belt(): a
    # kinematic rigid body (so it doesn't fall) with a PhysX surface velocity,
    # driven by an OmniGraph node. See build_conveyor() for the node wiring.
    p.add_argument("--conveyor", action=argparse.BooleanOptionalAction, default=True,
                   help="add a conveyor belt against --table-edge (default: on)")
    p.add_argument("--conveyor-width", type=float, default=0.30,
                   help="belt width across the direction of travel, m")
    p.add_argument("--conveyor-length", type=float, default=1.00,
                   help="belt length along the direction of travel, m")
    p.add_argument("--conveyor-thickness", type=float, default=0.04)
    p.add_argument("--conveyor-gap", type=float, default=0.0,
                   help="gap between the table edge and the belt's near side, m (0 = touching/sat canh)")
    p.add_argument("--conveyor-offset", type=float, default=0.0,
                   help="shift the belt sideways along the edge, relative to the table centre, m")
    p.add_argument("--conveyor-yaw", type=float, default=90.0,
                   help="extra rotation on top of --table-edge's own orientation, degrees. "
                        "0 = belt feeds straight toward/away from the table (its old default); "
                        "90 (the default now) turns it to run alongside the edge instead")
    p.add_argument("--conveyor-speed", type=float, default=0.2,
                   help="belt surface speed, m/s, along its direction of travel (--conveyor-yaw "
                        "away from --table-edge's own toward-the-table direction); negative reverses it")
    p.add_argument("--conveyor-engine", choices=("physx", "warp"), default="physx",
                   help="how the belt drags things: 'physx' (default) is a kinematic body with "
                        "PhysxSurfaceVelocityAPI -- works with any rigid body, no registration. "
                        "'warp' is Isaac Sim's own conveyor_belt sample technique (per-contact "
                        "Coulomb friction forces via NVIDIA Warp); it is built and registered "
                        "correctly but currently does NOT move anything in this Isaac Sim "
                        "6.0.0-rc.59 pip install -- isaacsim.core.api.World(backend='warp') does "
                        "not step physics at all here (verified independently of this script's own "
                        "code: even a bare falling box freezes). Kept for when that's fixed upstream; "
                        "use 'physx' for a belt that actually works today.")
    p.add_argument("--conveyor-demo-boxes", type=int, default=2,
                   help="warp engine only: how many boxes to spawn pre-registered on the belt "
                        "so there is something to see it drag")
    p.add_argument("--conveyor-demo-box-size", type=float, default=0.05,
                   help="warp engine only: edge length of each demo box, m")
    p.add_argument("--conveyor-demo-box-mass", type=float, default=0.05,
                   help="warp engine only: mass of each demo box, kg")
    p.add_argument("--conveyor-friction", type=float, default=0.5,
                   help="warp engine only: Coulomb friction coefficient between a demo box and the belt")
    p.add_argument("--conveyor-test-cube", action=argparse.BooleanOptionalAction, default=False,
                   help="drop a small ordinary dynamic cube onto the belt from above, to visually "
                        "confirm it's dragging things. Works with any --conveyor-engine, but only "
                        "actually moves under 'physx' -- 'warp' is currently non-functional, see "
                        "--conveyor-engine's help")
    p.add_argument("--conveyor-test-cube-size", type=float, default=0.05, help="edge length, m")
    p.add_argument("--conveyor-test-cube-mass", type=float, default=0.05, help="kg")
    p.add_argument("--conveyor-test-cube-drop-height", type=float, default=0.08,
                   help="height above the belt's top surface to drop the cube from, m")
    p.add_argument("--conveyor-test-cube-xy", type=float, nargs=2, default=None, metavar=("X", "Y"),
                   help="where to drop the test cube from, relative to the table centre "
                        "(same convention as --robot-xy); default: the belt's own centre")

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


def _parse_gain_overrides(text: str, *, source: str) -> dict:
    """Parse ``stiffness=80,damping=5,max_force=3`` into a gains dict."""
    gains: dict = {}
    for pair in text.split(","):
        pair = pair.strip()
        if not pair:
            continue
        if "=" not in pair:
            sys.exit(f"[so101_table_scene] {source}: expected key=value, got '{pair}'")
        key, _, value = pair.partition("=")
        key = key.strip()
        if key not in ("stiffness", "damping", "max_force"):
            sys.exit(f"[so101_table_scene] {source}: unknown gain '{key}', expected "
                      f"stiffness, damping or max_force")
        try:
            gains[key] = float(value)
        except ValueError:
            sys.exit(f"[so101_table_scene] {source}: '{key}={value}' is not a number")
    return gains


def _parse_joint_drive_args(specs: list[str]) -> dict[str, dict]:
    """Parse repeated ``--joint-drive joint_1:stiffness=80,damping=5`` flags."""
    overrides: dict[str, dict] = {}
    for spec in specs:
        if ":" not in spec:
            sys.exit(f"[so101_table_scene] --joint-drive '{spec}': expected JOINT:key=val[,key=val...]")
        name, _, rest = spec.partition(":")
        name = name.strip()
        if name not in JOINT_NAMES:
            sys.exit(f"[so101_table_scene] --joint-drive: unknown joint '{name}', expected one of {JOINT_NAMES}")
        overrides.setdefault(name, {}).update(_parse_gain_overrides(rest, source=f"--joint-drive {name}"))
    return overrides


def resolve_drive_gains(args: argparse.Namespace) -> dict[str, dict]:
    """Work out the effective stiffness/damping/max_force for every joint.

    Precedence, low to high:
      1. HARDCODED_DEFAULT_GAINS
      2. --drive-config's "default:" section
      3. --stiffness / --damping / --max-force (override the effective default)
      4. --drive-config's "joints: <name>:" section
      5. --joint-drive (repeatable, highest precedence)
    """
    default_gains = dict(HARDCODED_DEFAULT_GAINS)
    per_joint_from_yaml: dict[str, dict] = {}

    config_path = args.drive_config
    if config_path:
        is_default_path = os.path.abspath(config_path) == os.path.abspath(DEFAULT_DRIVE_CONFIG)
        if os.path.isfile(config_path):
            with open(config_path) as f:
                doc = yaml.safe_load(f) or {}
            unknown_top = set(doc) - {"default", "joints"}
            if unknown_top:
                sys.exit(f"[so101_table_scene] {config_path}: unknown top-level key(s) {sorted(unknown_top)}, "
                          f"expected 'default' and/or 'joints'")
            # None (YAML null) is a legitimate value for max_force -- "keep the USD's own
            # value" -- so every explicitly-listed key is applied, including None ones.
            for key, value in (doc.get("default") or {}).items():
                if key not in ("stiffness", "damping", "max_force"):
                    sys.exit(f"[so101_table_scene] {config_path}: unknown gain '{key}' under 'default'")
                default_gains[key] = value
            for name, gains in (doc.get("joints") or {}).items():
                if name not in JOINT_NAMES:
                    sys.exit(f"[so101_table_scene] {config_path}: unknown joint '{name}' under 'joints', "
                              f"expected one of {JOINT_NAMES}")
                for key in gains:
                    if key not in ("stiffness", "damping", "max_force"):
                        sys.exit(f"[so101_table_scene] {config_path}: unknown gain '{key}' for joint '{name}'")
                per_joint_from_yaml[name] = gains
        elif not is_default_path:
            # An explicitly-requested file that is missing is almost always a typo --
            # fail loudly instead of silently running with different gains than asked.
            sys.exit(f"[so101_table_scene] --drive-config file not found: {config_path}")
        # else: shipped default path just doesn't exist (e.g. running from a bare
        # checkout without the config/ dir) -- fall back to HARDCODED_DEFAULT_GAINS.

    # --stiffness/--damping/--max-force override the effective default, whether that
    # default came from HARDCODED_DEFAULT_GAINS or from the yaml's "default:" section.
    if args.stiffness is not None:
        default_gains["stiffness"] = args.stiffness
    if args.damping is not None:
        default_gains["damping"] = args.damping
    if args.max_force is not None:
        default_gains["max_force"] = args.max_force

    cli_overrides = _parse_joint_drive_args(args.joint_drive)

    resolved: dict[str, dict] = {}
    for name in JOINT_NAMES:
        gains = dict(default_gains)
        gains.update(per_joint_from_yaml.get(name, {}))
        gains.update(cli_overrides.get(name, {}))
        resolved[name] = gains
    return resolved


# --table-edge name -> (axis index: 0=x/1=y, sign of that axis where the edge sits).
# front/back are the two LONG edges (run along X); left/right are the two SHORT
# edges (run along Y). See --table-edge's help for the "canh ngang" convention.
_EDGE_AXIS = {"front": (1, -1.0), "back": (1, 1.0), "left": (0, -1.0), "right": (0, 1.0)}


def edge_position(table_size: tuple[float, float], edge: str) -> tuple[int, float, float]:
    """Resolve a --table-edge name against the table's footprint.

    Returns (axis, edge_coordinate, sign): ``axis`` is 0 for X or 1 for Y;
    ``edge_coordinate`` is that edge's position along ``axis``, relative to
    the table centre; ``sign`` is +1/-1, the direction "outward" (away from
    the table, over the edge) points along ``axis``.
    """
    axis, sign = _EDGE_AXIS[edge]
    half_extent = table_size[axis] / 2.0
    return axis, sign * half_extent, sign


def resolve_robot_xy(args: argparse.Namespace) -> tuple[float, float]:
    """Default --robot-xy: --robot-edge-margin in from --table-edge.

    Centred on the *belt* along the other axis when a conveyor is enabled
    (matching _compute_belt_geometry(): the belt's centre on that axis is
    always exactly --conveyor-offset, regardless of --conveyor-yaw/length/
    width -- see that function), so the arm sits centred on the belt, not just
    on the table. Centred on the table (0.0) when there's no belt to align to.
    """
    if args.robot_xy is not None:
        return tuple(args.robot_xy)
    axis, edge_coord, sign = edge_position(args.table_size, args.table_edge)
    pos = [0.0, 0.0]
    pos[axis] = edge_coord - sign * args.robot_edge_margin
    pos[1 - axis] = args.conveyor_offset if args.conveyor else 0.0
    return tuple(pos)


# The world yaw (degrees, rotate about Z) that makes a conveyor belt's local
# +X axis -- and so, with a local-frame direction of (1,0,0), its direction of
# travel -- point straight from --table-edge toward the table centre.
# --conveyor-yaw is added on top of this: 0 keeps that "feed toward the table"
# orientation, 90 turns the belt to run alongside the edge instead.
#
# Confirmed empirically: isaacsim.asset.gen.conveyor's IsaacConveyor node reads
# inputs:direction in the conveyorPrim's LOCAL frame (a belt rotated 90 deg
# about Z with inputs:direction=(1,0,0) drags things along world +Y, not +X) --
# see test_conveyor_frame.py. That is what lets a single fixed local direction
# of (1,0,0)/(-1,0,0) work at any yaw below, world-facing math included.
_EDGE_BASE_YAW = {"front": 90.0, "back": -90.0, "left": 0.0, "right": 180.0}


def rotate2d(x: float, y: float, degrees: float) -> tuple[float, float]:
    """Rotate a 2D vector counter-clockwise by ``degrees`` about the origin."""
    rad = math.radians(degrees)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a


ARGS = parse_args(sys.argv[1:])

if not os.path.isfile(ARGS.usd):
    sys.exit(f"[so101_table_scene] robot USD not found: {ARGS.usd}\nPass one with --usd /path/to/so101.usda")

# Fail fast on a bad --drive-config / --joint-drive before booting Isaac Sim,
# which takes tens of seconds. Discarded -- configure_drives() calls this
# again once the stage exists; re-parsing a small YAML file is negligible.
resolve_drive_gains(ARGS)

# ---------------------------------------------------------------------------
# SimulationApp must be constructed before any other Isaac Sim / omni import.
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": ARGS.headless})
import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.physx.bundle", True)

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


def enable_physics_authoring_ui() -> None:
    """Turn on omni.physx.ui ("PhysX UI" -- adds GUI elements for physics authoring).

    Without it, the Stage tree's right-click "Add" menu has no "Physics" entry
    at all (no Rigid Body, no Collider, ...) -- this script's SimulationApp
    boots a minimal "base" Kit profile meant for running simulations, not the
    full Create/Editor experience, so this convenience UI isn't loaded by
    default. Only worth enabling when there's a window to show it in.
    """
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("omni.physx.ui", True)
    simulation_app.update()
    if not manager.is_extension_enabled("omni.physx.ui"):
        carb.log_warn(
            "omni.physx.ui failed to load; the Stage tree's Add > Physics menu will be unavailable."
        )


def enable_conveyor_extension() -> None:
    """Turn on isaacsim.asset.gen.conveyor, which registers the IsaacConveyor OG node."""
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.asset.gen.conveyor", True)
    # Unlike isaacsim.ros2.bridge, this extension's own startup (registering the
    # OG node type) can still be in flight after set_extension_enabled_immediate()
    # returns -- one update() is not always enough. Poll briefly instead.
    for _ in range(60):
        simulation_app.update()
        if manager.is_extension_enabled("isaacsim.asset.gen.conveyor"):
            break
    if not manager.is_extension_enabled("isaacsim.asset.gen.conveyor"):
        carb.log_error("isaacsim.asset.gen.conveyor failed to load; the conveyor belt will be skipped.")


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
    kinematic: bool = False,
    yaw_deg: float = 0.0,
    mass: float | None = None,
) -> Usd.Prim:
    """A unit cube scaled into a collider box.

    ``kinematic=False, mass=None`` (the default): no RigidBodyAPI, so PhysX
    treats it as an immovable static collider -- used for the ground, table
    and legs.

    ``kinematic=True``: a kinematic rigid body -- PhysX won't move it under
    gravity/collisions, but a PhysxSurfaceVelocityAPI (added elsewhere, e.g.
    by isaacsim.asset.gen.conveyor's create_conveyor_belt()) can still drag
    things resting on top of it. Used for the conveyor belt.

    ``kinematic=False, mass=<value>``: a genuine dynamic rigid body -- PhysX
    simulates it freely (falls under gravity, pushed by contacts). Used for
    the conveyor test cube.

    ``yaw_deg``: rotate about Z, about the box's own centre, before placing it
    at ``center`` -- ``size`` stays in the box's own (now rotated) local
    frame. Used to turn the conveyor belt to face any direction.
    """
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    cube.CreateSizeAttr(1.0)  # spans -0.5..0.5, so scale == real dimensions
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])

    xform = UsdGeom.Xformable(cube)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    if yaw_deg:
        xform.AddRotateZOp().Set(float(yaw_deg))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))

    prim = cube.GetPrim()
    if kinematic:
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body.CreateRigidBodyEnabledAttr().Set(True)
        rigid_body.CreateKinematicEnabledAttr().Set(True)
    elif mass is not None:
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body.CreateRigidBodyEnabledAttr().Set(True)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(mass)
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


def _compute_belt_geometry(args: argparse.Namespace) -> dict:
    """Position/orientation math shared by both conveyor engines.

    The belt's box is always built with its length along its OWN local X and
    width along local Y, then rotated to _EDGE_BASE_YAW[table_edge] +
    --conveyor-yaw. Only the belt's *position* needs the yaw folded in, via an
    oriented-box projection (how far the rotated box's silhouette reaches back
    along the table's own outward axis), so the belt still sits flush against
    the edge -- gap exactly --conveyor-gap -- at any yaw.
    """
    axis, edge_coord, sign = edge_position(args.table_size, args.table_edge)
    other_axis = 1 - axis
    total_yaw = _EDGE_BASE_YAW[args.table_edge] + args.conveyor_yaw

    # direction_vec: world-space unit vector the belt's local +X (its "toward
    # the table" face at yaw=0) points at, after rotating by total_yaw.
    # perp_vec: local +Y (the belt's width axis) likewise.
    direction_vec = rotate2d(1.0, 0.0, total_yaw)
    perp_vec = rotate2d(0.0, 1.0, total_yaw)

    length, width = args.conveyor_length, args.conveyor_width
    # Half the rotated box's silhouette along the table's own outward axis --
    # 0.5*length at yaw=0 (belt end-on to the table), 0.5*width at yaw=+-90
    # (belt side-on, running alongside the edge), a blend in between.
    proj_half = 0.5 * (length * abs(direction_vec[axis]) + width * abs(perp_vec[axis]))

    near = edge_coord + sign * args.conveyor_gap  # the belt's near side, at/off the table edge
    belt_center_along_edge_axis = near + sign * proj_half

    top_z = args.table_height  # flush with the table top
    thickness = args.conveyor_thickness

    center = [0.0, 0.0, top_z - thickness / 2.0]
    center[axis] = belt_center_along_edge_axis
    center[other_axis] = args.conveyor_offset

    return {
        "axis": axis,
        "other_axis": other_axis,
        "sign": sign,
        "total_yaw": total_yaw,
        "direction_vec": direction_vec,
        "length": length,
        "width": width,
        "thickness": thickness,
        "top_z": top_z,
        "center": center,
    }


def _build_conveyor_legs(stage: Usd.Stage, physics_material, geo: dict) -> None:
    """Cosmetic support legs, styled like the table's -- shared by both engines.

    A kinematic (physx engine) or static (warp engine) belt needs no
    structural support, but one floating above the ground looks wrong.
    Corner offsets are in the belt's own local frame, so rotate2d() carries
    them into world (x, y) before adding the belt's centre.
    """
    length, width, top_z, thickness = geo["length"], geo["width"], geo["top_z"], geo["thickness"]
    total_yaw, center = geo["total_yaw"], geo["center"]

    leg = 0.06
    leg_height = top_z - thickness
    inset = leg / 2.0 + 0.03
    half_len, half_wid = length / 2.0 - inset, width / 2.0 - inset
    if not (leg_height > 0.02 and half_len > 0 and half_wid > 0):
        return
    for index, (dl, dw) in enumerate(
        ((half_len, half_wid), (half_len, -half_wid), (-half_len, half_wid), (-half_len, -half_wid))
    ):
        wx, wy = rotate2d(dl, dw, total_yaw)
        leg_center = [center[0] + wx, center[1] + wy, leg_height / 2.0]
        make_box(
            stage,
            f"/World/Conveyor/Leg_{index}",
            size=(leg, leg, leg_height),
            center=tuple(leg_center),
            color=(0.15, 0.15, 0.16),
            physics_material=physics_material,
        )


def _build_conveyor_physx(stage: Usd.Stage, args: argparse.Namespace, physics_material, geo: dict) -> None:
    """The belt via isaacsim.asset.gen.conveyor: a kinematic body with
    PhysxSurfaceVelocityAPI. Works with any rigid body -- no registration.

    inputs:direction on the IsaacConveyor node is always local (+1,0,0) or
    (-1,0,0) -- confirmed empirically to be in the conveyorPrim's LOCAL frame
    (see _EDGE_BASE_YAW's docstring-comment) -- so it automatically follows
    make_box's yaw_deg rotation; no local/world conversion to get wrong.
    """
    try:
        # Imported lazily, and only after main() has called
        # enable_conveyor_extension(): isaacsim.asset.gen.conveyor's Python
        # package is not on sys.path until the extension manager enables it,
        # so importing this at module load time (before any extension is
        # enabled) would always fail and silently disable the belt for good.
        from isaacsim.asset.gen.conveyor import create_conveyor_belt
    except ImportError:
        carb.log_error(
            "[so101_table_scene] isaacsim.asset.gen.conveyor is not installed; skipping the conveyor belt"
        )
        return

    length, width, thickness = geo["length"], geo["width"], geo["thickness"]
    center, total_yaw, direction_vec = geo["center"], geo["total_yaw"], geo["direction_vec"]

    stage.DefinePrim("/World/Conveyor", "Xform")
    belt_prim = make_box(
        stage,
        "/World/Conveyor/Belt",
        size=(length, width, thickness),
        center=tuple(center),
        color=(0.08, 0.08, 0.09),
        physics_material=physics_material,
        kinematic=True,
        yaw_deg=total_yaw,
    )

    _build_conveyor_legs(stage, physics_material, geo)

    conveyor_node = create_conveyor_belt(stage, belt_prim, "ConveyorBeltGraph")

    # Always local: (1,0,0) is "toward the table" at conveyor_yaw=0 and simply
    # rotates with the belt (and with --conveyor-yaw) otherwise, since the OG
    # node reads this in the conveyorPrim's own local frame.
    local_direction = Gf.Vec3f(1.0 if args.conveyor_speed >= 0 else -1.0, 0.0, 0.0)
    conveyor_node.GetAttribute("inputs:direction").Set(local_direction)

    # The generated graph drives inputs:velocity from a `Velocity` graph
    # variable (see create_conveyor_belt()) -- set that, not inputs:velocity
    # directly, or the per-tick ReadVariable node overwrites it next frame.
    conveyor_node.GetParent().GetAttribute("graph:variable:Velocity").Set(abs(args.conveyor_speed))

    world_direction = (direction_vec[0] if args.conveyor_speed >= 0 else -direction_vec[0],
                       direction_vec[1] if args.conveyor_speed >= 0 else -direction_vec[1])
    carb.log_info(
        f"[so101_table_scene] conveyor (physx engine): edge={args.table_edge} yaw={total_yaw:.1f} "
        f"center={center} size=({length},{width},{thickness}) "
        f"world_direction~=({world_direction[0]:.2f},{world_direction[1]:.2f}) speed={args.conveyor_speed}"
    )


def _build_conveyor_warp(stage: Usd.Stage, args: argparse.Namespace, physics_material, geo: dict):
    """The belt via Isaac Sim's own conveyor_belt sample technique: per-contact
    Coulomb friction forces computed with NVIDIA Warp (see conveyor_physics.py
    and isaac/conveyor_ref/README.md for what was vendored from the sample and
    why). Returns a ConveyorPhysics instance for main() to finalize once the
    rest of the scene exists, or None if the engine/extension isn't usable.

    Limitation inherited from the upstream sample: only the
    --conveyor-demo-boxes spawned here are registered to be dragged. Anything
    placed on the belt afterwards (e.g. by the robot arm, at runtime) will sit
    on it like an inert collider -- see ConveyorPhysics's module docstring.
    """
    try:
        import conveyor_physics as cp
    except ImportError as e:
        carb.log_error(
            f"[so101_table_scene] conveyor_physics (warp engine) unavailable: {e}; skipping the conveyor belt"
        )
        return None

    # isaacsim.core.api is core infrastructure (always enabled, unlike the
    # "content" extensions this file lazy-imports elsewhere), so a plain
    # import here is fine -- kept local to this function for symmetry with cp.
    from isaacsim.core.api.materials import PhysicsMaterial

    length, width, thickness = geo["length"], geo["width"], geo["thickness"]
    center, total_yaw, direction_vec = geo["center"], geo["total_yaw"], geo["direction_vec"]

    # The belt's OWN friction must be zero, with combine mode "min", so PhysX's
    # built-in friction never also acts -- the Warp kernels are the only thing
    # that should push a body along the belt. Matches cb_scene_simple.py.
    belt_material = PhysicsMaterial(
        "/World/PhysicsMaterials/ConveyorBeltWarp",
        static_friction=0.0,
        dynamic_friction=0.0,
        restitution=0.0,
    )
    PhysxSchema.PhysxMaterialAPI.Apply(belt_material.prim).CreateFrictionCombineModeAttr().Set("min")

    stage.DefinePrim("/World/Conveyor", "Xform")
    orientation = cp.yaw_deg_to_quat_wxyz(total_yaw)
    half_extent = (length / 2.0, width / 2.0, thickness / 2.0)
    belt_body_prim, belt_geom_prim = cp.create_conveyor_belt_box(
        stage,
        "/World/Conveyor/Belt",
        tuple(center),
        orientation,
        half_extent,
        border_margin_y=min(0.02, width / 4.0),
        material=belt_material.material,
        contact_offset=cp.CONTACT_OFFSET,
        rest_offset=cp.REST_OFFSET,
    )
    UsdGeom.Gprim(belt_geom_prim).CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.08, 0.09)])

    _build_conveyor_legs(stage, physics_material, geo)

    physics = cp.ConveyorPhysics(device="cuda:0")
    world_velocity = (
        direction_vec[0] * args.conveyor_speed,
        direction_vec[1] * args.conveyor_speed,
        0.0,
    )
    physics.add_belt(str(belt_geom_prim.GetPath()), world_target_velocity=world_velocity)

    # The demo boxes' own material -- unlike the belt's, its friction value
    # isn't load-bearing (combine mode "min" against the belt's zero always
    # wins), but matching cb_scene_simple.py's pattern keeps the two
    # consistent and gives sane behaviour if a box ever contacts something
    # else with real friction (e.g. slides off the belt onto the table).
    box_material = PhysicsMaterial(
        "/World/PhysicsMaterials/ConveyorBoxWarp",
        static_friction=args.conveyor_friction,
        dynamic_friction=args.conveyor_friction,
        restitution=0.0,
    )

    box_count = max(args.conveyor_demo_boxes, 0)
    half_box = args.conveyor_demo_box_size / 2.0
    for i in range(box_count):
        # Spread the demo boxes evenly along the belt's length, avoiding the ends.
        t = (i + 1) / (box_count + 1) - 0.5  # in (-0.5, 0.5)
        offset_along = t * max(length - 2.0 * half_box - 0.02, 0.0)
        wx, wy = rotate2d(offset_along, 0.0, total_yaw)
        box_center = (
            center[0] + wx,
            center[1] + wy,
            geo["top_z"] + half_box + 2.0 * cp.REST_OFFSET,
        )
        box_path = f"/World/Conveyor/DemoBox_{i}"
        box_prim, _box_geom = cp.create_rigid_body_box(
            stage,
            box_path,
            box_center,
            cp.yaw_deg_to_quat_wxyz(0.0),
            (half_box, half_box, half_box),
            mass=args.conveyor_demo_box_mass,
            material=box_material.material,
            contact_offset=cp.CONTACT_OFFSET,
            rest_offset=cp.REST_OFFSET,
        )
        physics.add_body(str(box_prim.GetPath()), friction_vs_belt=args.conveyor_friction)

    physics.finalize()
    carb.log_info(
        f"[so101_table_scene] conveyor (warp engine): edge={args.table_edge} yaw={total_yaw:.1f} "
        f"center={center} size=({length},{width},{thickness}) "
        f"target_velocity={world_velocity} demo_boxes={box_count}"
    )
    return physics


def build_conveyor(stage: Usd.Stage, args: argparse.Namespace, physics_material):
    """Add a conveyor belt against --table-edge. Returns a ConveyorPhysics
    instance if --conveyor-engine warp built one with at least one registered
    body (main() must finalize the rest of the scene, then hand it to a
    World), or None otherwise (physx engine, no conveyor, or 0 demo boxes)."""
    if not args.conveyor:
        return None
    geo = _compute_belt_geometry(args)
    if args.conveyor_engine == "physx":
        _build_conveyor_physx(stage, args, physics_material, geo)
        return None
    return _build_conveyor_warp(stage, args, physics_material, geo)


def build_conveyor_test_cube(stage: Usd.Stage, args: argparse.Namespace, physics_material) -> None:
    """Drop a small ordinary dynamic cube onto the belt from above -- a quick
    visual check that it's actually dragging things.

    A genuine dynamic rigid body (make_box's mass= path), same as any cube you'd
    add by hand in the GUI -- no registration needed, which is exactly what lets
    it also demonstrate the --conveyor-engine warp limitation: under physx it
    slides once it lands; under warp it just sits there (nothing not explicitly
    registered via --conveyor-demo-boxes ever moves -- see that flag's help).
    """
    if not (args.conveyor and args.conveyor_test_cube):
        return

    geo = _compute_belt_geometry(args)
    if args.conveyor_test_cube_xy is not None:
        drop_x, drop_y = args.conveyor_test_cube_xy
    else:
        drop_x, drop_y = geo["center"][0], geo["center"][1]
    size = args.conveyor_test_cube_size
    drop_z = geo["top_z"] + args.conveyor_test_cube_drop_height + size / 2.0

    make_box(
        stage,
        "/World/Conveyor/TestCube",
        size=(size, size, size),
        center=(drop_x, drop_y, drop_z),
        color=(0.85, 0.15, 0.1),
        physics_material=physics_material,
        mass=args.conveyor_test_cube_mass,
    )
    carb.log_info(
        f"[so101_table_scene] conveyor test cube: dropping at "
        f"({drop_x:.3f}, {drop_y:.3f}, {drop_z:.3f}), {size:.3f} m, {args.conveyor_test_cube_mass:.3f} kg"
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
    x, y = resolve_robot_xy(args)
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
    sag under gravity and ignore /joint_command. Per-joint values come from
    resolve_drive_gains(); see --drive-config and --joint-drive.
    """
    gains_by_joint = resolve_drive_gains(args)

    for name, home_rad in zip(JOINT_NAMES, args.home):
        path = f"{JOINTS_SCOPE}/{name}"
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            carb.log_error(f"[so101_table_scene] joint prim missing: {path}")
            continue

        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")

        gains = gains_by_joint[name]
        home_deg = math.degrees(home_rad)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(gains["stiffness"])
        drive.CreateDampingAttr().Set(gains["damping"])
        drive.CreateTargetPositionAttr().Set(home_deg)
        drive.CreateTargetVelocityAttr().Set(0.0)
        if gains.get("max_force") is not None:
            drive.CreateMaxForceAttr().Set(gains["max_force"])

        carb.log_info(
            f"[so101_table_scene] {name}: stiffness={gains['stiffness']} damping={gains['damping']} "
            f"max_force={gains.get('max_force')}"
        )

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
    if not args.headless:
        enable_physics_authoring_ui()
    if args.conveyor:
        enable_conveyor_extension()

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
    conveyor_physics = build_conveyor(stage, args, grip_material)
    if conveyor_physics is not None:
        # --conveyor-engine warp: the belt and its demo boxes exist (built
        # above), and the ConveyorPhysics object is fully registered and
        # finalized, but nothing drives it -- confirmed on BOTH Isaac Sim
        # installs on this machine (the pip package this script runs under,
        # and the standalone package in ~/Downloads), by running NVIDIA's own
        # unmodified conveyor_belt/cb_app_simple.py sample: the tracked box
        # sits frozen at its exact spawn position for 240 physics steps, no
        # error logged. isaacsim.core.api.World(backend="warp", device="cuda")
        # does not step physics at all in this Isaac Sim 6.0.0-rc.59 release
        # -- an upstream limitation, not something this script (or a
        # different install of the same release) can work around. The belt
        # therefore sits idle; switch to --conveyor-engine physx (the
        # default) for a working belt.
        carb.log_warn(
            "[so101_table_scene] --conveyor-engine warp: the belt and its demo boxes are built, "
            "but will NOT move. isaacsim.core.api.World(backend='warp') does not step physics in "
            "this Isaac Sim release (verified against NVIDIA's own unmodified sample, on both "
            "Isaac Sim installs on this machine) -- use --conveyor-engine physx for a working belt."
        )
    build_conveyor_test_cube(stage, args, grip_material)
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
