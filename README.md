# env_robot — SO-101 on a table, Isaac Sim + ROS 2 Jazzy

```
src/
  so101_description/   URDF + meshes + so101.usda (pre-existing)
  so101_isaac/         the simulation environment and the joint-editing node
run_scene.sh           run the Isaac scene without ros2 launch
```

## 1. Prerequisites

| Thing | Where |
|---|---|
| Isaac Sim 6.0 (pip) | `~/env_isaac/bin/python3` — already installed |
| ROS 2 Jazzy | `/opt/ros/jazzy` — needed for `colcon`, `rclpy`, the `ros2` CLI and `joint_commander` |

```bash
sudo apt install ros-jazzy-desktop ros-dev-tools
```

Isaac Sim's `isaacsim.ros2.bridge` `dlopen()`s the ROS 2 shared libraries at
runtime. It resolves them one of two ways:

1. **A sourced system install** — `source /opt/ros/jazzy/setup.bash`. Required
   for everything else in this workspace.
2. **The Jazzy libraries bundled inside Isaac Sim**, at
   `.../isaacsim/exts/isaacsim.ros2.core/jazzy/lib`. Enough to make the
   *simulator* publish and subscribe with no system ROS 2 at all, but it
   contains no `rclpy` and no `ros2` CLI, so `joint_commander` cannot run
   against it.

`run_scene.sh` picks (1) when it finds a system install and falls back to (2)
otherwise. Without either, `isaacsim.ros2.bridge` fails with
`librmw_implementation.so: libament_index_cpp.so: cannot open shared object
file` and the scene script exits.

## 2. Build

`colcon` is not part of `ros-jazzy-desktop`; install it first if `colcon` is
not on your PATH:

```bash
sudo apt install python3-colcon-common-extensions   # or: ros-dev-tools
```

```bash
cd ~/Project/env_robot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Run

```bash
ros2 launch so101_isaac so101_isaac.launch.py
```

Useful arguments:

```bash
ros2 launch so101_isaac so101_isaac.launch.py \
    headless:=true \
    table_height:=0.90 \
    robot_x:=-0.2 robot_yaw:=90.0 \
    stiffness:=20.0 damping:=2.0 \
    start_commander:=false
```

Or just the simulator, no ROS 2 launch (this one works without a system ROS 2):

```bash
./run_scene.sh
./run_scene.sh --headless --save-usd /tmp/so101_table.usda
./run_scene.sh --headless --max-steps 300      # run 300 steps and quit, for smoke tests
```

## 4. What gets built

The scene is authored procedurally, so nothing is fetched from Nucleus:

```
/World
  /World/PhysicsScene      gravity, TGS solver, 120 Hz
  /World/Ground            20x20 m static box floor
  /World/Table             1.20 x 0.80 m top at z = 0.74, four legs, static colliders
  /World/Conveyor          belt + 4 legs, against --table-edge (isaacsim.asset.gen.conveyor)
  /World/so101             reference to so101.usda, Physics variant = "physx"
  /World/ActionGraph       the ROS 2 bridge
```

The robot is snapped onto the table top using its own bounding box
(`--no-auto-place` turns that off), because the URDF rotates the model 90° about
X and `base_link`'s origin is not its lowest point.

### Table edge, robot placement, conveyor belt

`--table-edge {front,back,left,right}` (default `front`) names one of the
table's four edges and drives two defaults at once:

- `--robot-xy`, when left unset, places the robot `--robot-edge-margin`
  (default 0.08 m) in from that edge, centred on the other axis — "right at
  the edge" rather than in the middle of the table.
- The conveyor belt (below) attaches to the same edge.

`front`/`back` are the table's two **long** edges (run along X, at
`y = -width/2` / `y = +width/2`); `left`/`right` are the two **short** edges
(run along Y, at `x = -length/2` / `x = +length/2`). With the default
1.20 x 0.80 m top, front/back are the "horizontal" edges in a landscape view of
the table — that's the reading `front` (the default) assumes; pass
`--table-edge back/left/right` if you meant a different one.

The belt is a **kinematic** rigid body (`RigidBodyAPI` + `kinematicEnabled`):
PhysX never moves it, but a `PhysxSurfaceVelocityAPI` (applied by the
`IsaacConveyor` OmniGraph node from `isaacsim.asset.gen.conveyor`, at runtime,
once the belt already has a `RigidBodyAPI`) still drags whatever rests on top.

The belt's box is always built with its length along its own local X, then
rotated (`--conveyor-yaw`, degrees, **default 90**) on top of an
edge-dependent base orientation:

- `--conveyor-yaw 0`: the belt feeds straight toward/away from the table
  (perpendicular to the edge, extending outward by `--conveyor-length`).
- `--conveyor-yaw 90` (**the default**): the belt runs **alongside** the
  edge instead (parallel to it, reaching only `--conveyor-width`/2 out from
  the table) — a robot at the edge reaches sideways into a belt passing by,
  rather than having the belt feed straight into it.

Confirmed empirically (not assumed): `IsaacConveyor`'s `inputs:direction` is
read in the belt's own **local** frame — a belt rotated 90° about Z with
`inputs:direction=(1,0,0)` drags things along world +Y, not +X. That's what
lets the code keep `inputs:direction` fixed at local `(±1,0,0)` at any yaw,
while the belt's *position* is computed from an oriented-bounding-box
projection (how far the rotated box's silhouette reaches back along the
table's own outward axis) so it still sits flush against the edge — with a
gap of exactly `--conveyor-gap` — regardless of yaw.

```bash
./run_scene.sh --table-edge front                          # default: belt runs alongside the -Y edge
./run_scene.sh --conveyor-yaw 0                             # old behaviour: belt feeds into the table
./run_scene.sh --table-edge right --conveyor-speed -0.15    # short edge, reversed travel direction

ros2 launch so101_isaac so101_isaac.launch.py table_edge:=back conveyor_speed:=0.3
ros2 launch so101_isaac so101_isaac.launch.py conveyor:=false   # table + arm only, no belt
```

`--conveyor-length`/`--conveyor-width`/`--conveyor-thickness` size the belt;
`--conveyor-gap` sets the clearance from the table edge (`0` = touching);
`--conveyor-offset` shifts it sideways along the edge. `--conveyor-speed` is
signed m/s along the belt's direction of travel (which is `--conveyor-yaw`
away from "toward the table"); the belt's top sits flush with the table top
by default.

Verified live, both orientations: at `--conveyor-yaw 0` a dropped cube moved
from `y=-0.81` to `y=-0.44` in 2 s (≈0.186 m/s, toward the table); at the
current default (`--conveyor-yaw 90`) a dropped cube moved from `x=0.00` to
`x=0.39` in 1.5 s (exactly 0.200 m/s, alongside the edge) — both against a
configured 0.2 m/s belt speed.

### Drives

`so101_description/urdf/so101/payloads/Physics/physics.usda` applies
`PhysicsDriveAPI:angular` to all six joints, but only authors `damping` and
`maxForce` — **no stiffness**. Without stiffness the drives act as pure dampers:
the arm sags under gravity and ignores position commands. The scene script
therefore writes `stiffness`, `damping` and the home `targetPosition` onto each
drive at startup (`--stiffness` / `--damping`).

Those are USD angular-drive units, i.e. **per degree**, not per radian. The
defaults are stiffness 60 / damping 4, picked by measurement: raising
stiffness from 10 to 60 only cut the resting sag from 0.0096 rad to 0.0063 rad.
A 6x stiffness increase buying a 1.5x error reduction means the residual is
**bound by `maxForce`, not by stiffness** — 2.5 Nm is the real servo limit the
URDF declares, and gravity eats most of it. Raising stiffness further will not
help; raise `max_force` if you want a stiffer arm than the real hardware.

### Per-joint tuning

`config/joint_drives.yaml` sets stiffness/damping/max_force per joint.
Precedence, low to high:

```
hardcoded 60/4/None  <  yaml "default:"  <  --stiffness/--damping/--max-force
  <  yaml "joints: <name>:"  <  --joint-drive (repeatable, wins over everything)
```

Edit the file for a permanent change:

```yaml
default:
  stiffness: 60.0
  damping: 4.0
  max_force: null        # null keeps the URDF's own 2.5 Nm

joints:
  gripper_joint:
    stiffness: 100.0
    damping: 6.0
    max_force: 1.0        # cap it below the arm's 2.5 Nm
```

Or override for one run without touching the file:

```bash
./run_scene.sh --joint-drive "joint_1:stiffness=90,damping=6" \
               --joint-drive "gripper_joint:max_force=1.0"

ros2 launch so101_isaac so101_isaac.launch.py \
    joint_drive:="joint_1:stiffness=90,damping=6;gripper_joint:max_force=1.0"
```

A joint named under `joints:` (in the yaml) always wins over a global
`--stiffness`/`--damping`/`--max-force` flag — that is the point of naming it.
Leave `joints:` empty to let the global flags control every joint uniformly.
`--joint-drive` and a bad `--drive-config` path both fail fast, before Isaac
Sim boots, with a message naming the bad joint/key/path -- run `--help` for
the exact rules.

## 5. Topics

| Topic | Type | Direction |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Isaac Sim → you |
| `/joint_command` | `sensor_msgs/JointState` | you → Isaac Sim |
| `/clock` | `rosgraph_msgs/Clock` | Isaac Sim → you |
| `/tf` | `tf2_msgs/TFMessage` | Isaac Sim → you |

Graph: `IsaacReadJointState → ROS2PublishJointState` and
`ROS2SubscribeJointState → IsaacArticulationController`, both driven by
`OnPlaybackTick`.

## 6. Editing the joint state

`joint_commander` seeds itself from the measured `/joint_states`, then streams a
rate-limited target to `/joint_command` at 60 Hz. Three ways to change it:

```bash
# 1. parameters, one per joint
ros2 param set /joint_commander target.joint_2 0.6
ros2 param list /joint_commander

# 2. a partial JointState on ~/set_target
#    Repeat the message -- a single `--once` often fires before the
#    subscription has been matched and is silently dropped.
ros2 topic pub -r 10 --times 15 /joint_commander/set_target sensor_msgs/msg/JointState \
    '{name: ["joint_1", "gripper_joint"], position: [0.8, 0.5]}'

# 3. back to the home pose
ros2 service call /joint_commander/go_home std_srvs/srv/Trigger
```

Check the result:

```bash
ros2 topic echo /joint_states
```

Targets are clamped to the URDF limits (`clamp_to_limits`) and approached at
`max_speed` rad/s, so nothing snaps. Both live in
`src/so101_isaac/config/joint_commander.yaml`.

You can also drive `/joint_command` directly, **but only when the commander is
not running** — it re-asserts its own target at 60 Hz and will win against a
slower publisher, so the arm just sits there. Start the stack with
`start_commander:=false` first, or stop the node:

```bash
ros2 launch so101_isaac so101_isaac.launch.py start_commander:=false
ros2 topic pub -r 30 /joint_command sensor_msgs/msg/JointState \
    '{name: ["joint_1","joint_2","joint_3","joint_4","joint_5","gripper_joint"],
      position: [0.0, 0.5, -0.3, 0.2, 0.0, 0.0]}'
```

## 7. Measured behaviour

Round trip through `ros2 launch`, headless, commanding all six joints at once:

| joint | start | final | target | gap closed |
|---|---|---|---|---|
| `joint_1` | +0.0063 | -0.4980 | -0.50 | 99.6% |
| `joint_2` | -0.0074 | +0.5949 | +0.60 | 99.2% |
| `joint_3` | -0.0001 | -0.4015 | -0.40 | 99.6% |
| `joint_4` | -0.0001 | +0.7999 | +0.80 | 100% |
| `joint_5` | +0.0000 | +0.3000 | +0.30 | 100% |
| `gripper_joint` | +0.0000 | +0.4000 | +0.40 | 100% |

Steady-state error is largest on the joints carrying the most gravity load
(`joint_1`, `joint_2`) and is set by `maxForce`, as described above.

Isaac Sim boots in about 10 s headless once its shader cache is warm; the first
run takes considerably longer.

## 8. Testing

The joint-editing node can be exercised without starting the simulator. The
runner borrows the `rclpy` bundled inside Isaac Sim when no system ROS 2 is
sourced, so it works before `ros-jazzy-desktop` is installed:

```bash
cd src/so101_isaac && ./test/run_integration.sh
```

It stands in for Isaac Sim — publishes `/joint_states`, watches
`/joint_command` — and checks seeding, parameter updates, joint-limit
rejection, the `~/set_target` topic, `~/go_home`, and rate limiting.

A smoke test of the scene itself:

```bash
./run_scene.sh --headless --max-steps 300 --save-usd /tmp/so101_table.usda
```

## 9. Known rough edges

- `ROS2PublishTransformTree` logs a deprecation warning about `targetPrims` and
  asks for an `IsaacComputeTransformTree` node upstream. That node does not ship
  in Isaac Sim 6.0.0-rc.59, so `targetPrims` is the only option here. It works;
  it just warns once per run.
- Isaac Sim's stdout is block-buffered when it is not attached to a terminal, so
  under `ros2 launch` its messages arrive in chunks. The launch file sets
  `PYTHONUNBUFFERED=1` and the script flushes its own prints.

- `so101_description/urdf/so101.urdf.xacro` still references
  `so101_moveit_config` and `so101_gazebo`, which are not in this workspace
  (they live in `~/Project/so101`). `view_robot.launch.py` will fail until those
  are added or the xacro is trimmed, and it also needs `ros-jazzy-xacro`, which
  `ros-jazzy-desktop` does not pull in. Nothing here uses the xacro — the Isaac
  scene loads `so101.usda` directly.
- `so101_description/CMakeLists.txt` used to `find_package(xacro REQUIRED)`
  although the package only installs directories, which broke `colcon build` on
  a machine without `ros-jazzy-xacro`. The find_package calls were removed and
  the real dependencies declared as `<exec_depend>` in `package.xml`.
- Joint limits in `joint_commander.py` are copied from the xacro. If you edit
  the URDF, update `DEFAULT_LIMITS` too.
- Anything that publishes `/joint_command` competes with `joint_commander`,
  which re-asserts its target at 60 Hz. Two commanders, or a commander plus a
  hand-rolled publisher, produce an arm that ignores you with no error message.
  `ros2 node list` first. A `ros2 launch` killed with `SIGKILL` can leave a
  commander behind.
- ROS 2's `setup.bash` reads variables it never defines, so any script that
  sources it under `set -u` dies with `AMENT_TRACE_SETUP_FILES: unbound
  variable`. `run_scene.sh` drops `set -u` around the source.
- An Isaac Sim extension's Python package (e.g.
  `isaacsim.asset.gen.conveyor`) is not on `sys.path` until the extension
  manager actually enables it -- importing it at module load time, before any
  extension has been turned on, always raises `ImportError` and, behind a
  broad `except ImportError`, fails silently. `build_conveyor()` imports
  `create_conveyor_belt` lazily, inside the function, after
  `enable_conveyor_extension()` has already run in `main()`.
- `set_extension_enabled_immediate()` can return before that extension's own
  `on_startup()` (registering its OmniGraph node types) has actually
  finished -- one `simulation_app.update()` afterward is not always enough,
  unlike `isaacsim.ros2.bridge` which this one behavior differs from.
  `enable_conveyor_extension()` polls `is_extension_enabled()` for a few
  frames instead of checking once.
- An XML comment containing `--` anywhere inside it (not just right before
  `-->`) makes `package.xml` invalid XML. `catkin_pkg`'s parser then raises,
  colcon's `ros` package-identification extension silently falls back to
  treating the package as plain `python` instead of `ros.ament_python`, and
  `colcon build` still reports success -- but the built package is invisible
  to `ros2 pkg list` / `ros2 launch` (its `AMENT_PREFIX_PATH` hook is never
  generated, only `PYTHONPATH`). If a package builds but `ros2 launch` can't
  find it, check `python3 -c "from catkin_pkg.package import parse_package;
  parse_package('src/<pkg>/package.xml')"` before anything else.
- Two ROS graphs on the same machine with no explicit `ROS_DOMAIN_ID` (the
  default is 0 for everyone) see each other's topics. A `/joint_states` from
  an unrelated project publishing under the same domain will interleave with
  ours on `/joint_states` since neither is namespaced, and a subscriber can
  end up reading the other robot's joint names. Set `ROS_DOMAIN_ID` to
  something project-specific if you run more than one ROS 2 project on this
  machine at once.
