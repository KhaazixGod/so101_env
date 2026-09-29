#!/usr/bin/env python3
"""Warp-based conveyor belt physics -- the technique from Isaac Sim's own
``conveyor_belt`` standalone sample, not the ``PhysxSurfaceVelocityAPI`` schema.

Rather than a kinematic body with a surface-velocity attribute, this computes
per-contact Coulomb friction forces (NVIDIA Warp kernels) between the belt and
whatever rigid bodies are registered onto it, and applies them directly. See
``conveyor_ref/README.md`` for exactly which files were vendored unmodified
from the sample and why, and the sample's own README (shipped alongside it)
for the full rationale/tradeoffs of this approach vs. PhysxSurfaceVelocityAPI.

*** CURRENTLY NON-FUNCTIONAL ON THIS MACHINE, VERIFIED, NOT A CODE BUG ***
This module, the classes/kernels it wires together, and the calling code in
so101_table_scene.py are all correct and match the upstream sample's own
approach. But getting a per-contact force onto a body requires a *tensor* view
(RigidPrim's internal RigidContactView) backed by ``isaacsim.core.api.World``
with ``backend="warp", device="cuda:..."`` -- and in this Isaac Sim
6.0.0-rc.59 pip install, on this GPU/driver, constructing that World and
calling ``world.step()`` freezes ALL physics stepping, with no error logged.
Confirmed by elimination, independent of anything in this file:

  - A `World(backend="numpy")` (the default) steps a falling box normally.
  - The *identical* scene with `World(backend="warp", device="cuda:0")`
    freezes it -- with zero other code involved: no RigidPrim, no
    ConveyorPhysics, no contact registration, nothing from this module.
  - `set_defaults=True` (letting World configure its own physics scene
    entirely) makes no difference.

So this is an environment limitation (this Isaac Sim build's GPU/warp physics
backend), not something fixable by changing how this module or
so101_table_scene.py call the API. so101_table_scene.py's --conveyor-engine
warp builds the belt and registers everything correctly, then logs a warning
and does NOT construct a World or step physics through it, precisely because
of this. Kept in the repo in case a future Isaac Sim release fixes the warp
backend; --conveyor-engine physx (PhysxSurfaceVelocityAPI, the other
implementation in so101_table_scene.py) is the one that actually moves things
today.

Usage, mirroring the vendored sample's own ``ConveyorBeltExample`` (in its
``cb_app.py``, not vendored -- this module is our own, scoped-down stand-in
for it) -- for this to actually move anything, the caller must construct a
working ``isaacsim.core.api.World(backend="warp", device="cuda:...")``
*before* calling finalize() (RigidPrim reads ``SimulationContext.instance()``
at its own construction time), add ``physics.contact_view`` to
``world.scene``, and call ``world.reset(soft=False)`` before the first
``physics.step()`` -- see this module's git history for a version of
so101_table_scene.py's main() that did exactly that, if the environment
limitation above is ever resolved:

    physics = ConveyorPhysics(device="cuda:0")
    physics.add_belt(belt_geom_path, world_target_velocity=(0.2, 0.0, 0.0))
    physics.add_body(box_prim_path, friction_vs_belt=0.5)   # zero or more
    # sim_world = isaacsim.core.api.World(backend="warp", device="cuda:0", ...)
    physics.finalize()                                       # once
    # sim_world.scene.add(physics.contact_view); sim_world.reset(soft=False)
    SimulationManager.register_callback(physics.step, SimulationEvent.PHYSICS_POST_STEP)

Also inherited from the upstream sample (see its README's "potential
extensions" section, independent of the environment issue above): every body
that should be dragged by a belt must be registered with add_body() *before*
finalize() is called. There is no supported way to register a new body
afterwards without rebuilding the contact view and re-finalizing -- a body
placed on the belt later (e.g. by the robot arm, at runtime, after the scene
has started) will sit on it like an inert collider and will NOT be moved. If
your scene needs that, either pre-register a pool of bodies to recycle, or use
the simpler --conveyor-engine physx, which works with *any* rigid body with no
registration step.
"""

from __future__ import annotations

import math
import os
import sys

# The vendored files import each other with flat names (e.g. cb_kernels.py
# does `import cb_utils as cb_utils`), exactly as the upstream sample expects
# when run as sibling scripts. Inserting this directory onto sys.path lets us
# import them unmodified, rather than rewriting their imports -- see
# conveyor_ref/README.md.
_REF_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "conveyor_ref")
if _REF_DIR not in sys.path:
    sys.path.insert(0, _REF_DIR)

import warp as wp  # noqa: E402

import cb_kernels as conveyor_belt_kernels  # noqa: E402
from cb_actuators import VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY, VelocityFieldActuator  # noqa: E402
from cb_body_manager import BodyManager  # noqa: E402
from cb_conveyor_belt_manager import ConveyorBeltManager  # noqa: E402
from cb_material_pair_manager import MaterialPairManager  # noqa: E402
from cb_scene_building_utils import (  # noqa: E402
    BODY_TYPE_DEFAULT,
    create_conveyor_belt_box,
    create_rigid_body_box,
)

wp.config.enable_backward = False

# Matches the upstream sample's own defaults (its cb_scene.py).
CONTACT_PROCESSING_THRESHOLD = 0.997  # dot(contact_normal, belt_surface_normal); ~4 degrees of tolerance
CONTACT_OFFSET = 0.0025
REST_OFFSET = 0.002
MAX_AVERAGE_CONTACT_COUNT_PER_BODY = 32
MAX_THREAD_COUNT = 4096
CONTACT_PROCESSING_BATCH_SIZE = 5


def yaw_deg_to_quat_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    """A quaternion (w, x, y, z) for a rotation about world +Z, matching the
    ``xformOp:rotateZ`` convention used elsewhere in so101_table_scene.py."""
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def enable_warp_backend(device: str) -> None:
    """Point isaacsim.core's tensor views (RigidPrim, etc.) at Warp/this device.

    This is exactly what the upstream sample's ``World(backend="warp",
    device="cuda")`` does under the hood (SimulationManager is a process-wide
    singleton, so it does not require a World object to configure).
    """
    from isaacsim.core.simulation_manager import SimulationManager

    SimulationManager.set_backend("warp")
    SimulationManager.set_physics_sim_device(device)


class ConveyorPhysics:
    """One or more belts driving registered rigid bodies via Warp contact forces.

    Scoped down from the upstream sample's ``ConveyorBeltExample``: no turns,
    ramps, multi-belt circuits, velocity-field visualizer, or CUDA-graph
    capture (our scenes have a handful of contacts at most, nowhere near
    where the graph-capture optimization -- and the reset/re-capture state it
    needs -- would pay for its own complexity).
    """

    def __init__(self, device: str | None = None) -> None:
        self.device = device
        self.actuator = VelocityFieldActuator()
        self.belts = ConveyorBeltManager()
        self.bodies = BodyManager()
        self.materials = MaterialPairManager()

        self._belt_material_index: int | None = None
        self._contact_view = None
        self._finalized = False
        self._belt_count = 0

    def belt_material_index(self) -> int:
        """One shared material index for every belt this instance owns."""
        if self._belt_material_index is None:
            self._belt_material_index = self.materials.add_conveyor_belt_material_index()
        return self._belt_material_index

    def add_belt(
        self,
        geom_prim_path: str,
        world_target_velocity: tuple[float, float, float],
        surface_normal: tuple[float, float, float] = (0.0, 0.0, 1.0),
        contact_processing_threshold: float = CONTACT_PROCESSING_THRESHOLD,
    ) -> None:
        """Register a belt's *collision geometry* prim (not its Xform parent)."""
        if self._finalized:
            raise RuntimeError("add_belt() must be called before finalize()")
        velocity_field_id = self.actuator.add_constant_velocity_field(wp.vec3(*world_target_velocity))
        self.belts.add_conveyor_belt(
            geom_prim_path,
            VELOCITY_FIELD_TYPE_CONSTANT_VELOCITY,
            velocity_field_id,
            wp.vec3(*surface_normal),
            contact_processing_threshold,
            self.belt_material_index(),
        )
        self._belt_count += 1

    def add_body(self, body_prim_path: str, friction_vs_belt: float = 0.5) -> None:
        """Register a rigid body's *Xform* prim (its RigidBodyAPI-carrying prim) to be dragged."""
        if self._finalized:
            raise RuntimeError("add_body() must be called before finalize()")
        body_material_index = self.materials.add_transported_body_material_index()
        self.materials.set_material_pair_friction(body_material_index, self.belt_material_index(), friction_vs_belt)
        self.bodies.add_body(body_prim_path, body_material_index)

    @property
    def body_count(self) -> int:
        return len(self.bodies.body_path_list)

    @property
    def contact_view(self):
        """The RigidPrim contact view built by finalize(), or None (before
        finalize(), or if body_count was 0). The caller registers this with
        World.scene.add(...) -- see this module's docstring."""
        return self._contact_view

    def finalize(self) -> None:
        """Allocate buffers and build the contact view. Call exactly once, after
        every add_belt()/add_body() call, and before the timeline plays."""
        if self._finalized:
            raise RuntimeError("finalize() already called")
        self._finalized = True

        if self.body_count == 0:
            # Nothing to drag -- skip the Warp/contact-view machinery entirely.
            # The belt geometry still exists and still collides; it just never
            # applies a force to anything, since nothing was registered.
            return

        from isaacsim.core.prims import RigidPrim

        self.actuator.create_buffers(self.device)
        self.belts.create_buffers(self.device)
        self.bodies.create_buffers(self.device)
        self.materials.create_buffers(self.device)

        max_contact_count = self.body_count * MAX_AVERAGE_CONTACT_COUNT_PER_BODY
        self._max_contact_count = max_contact_count

        self._total_contact_count = wp.zeros(shape=1, dtype=wp.uint32, device=self.device)
        self._point_to_indices_map = wp.empty(shape=(max_contact_count, 3), dtype=wp.uint32, device=self.device)
        self._mass_splitting_scale_buffer = wp.empty(shape=max_contact_count, dtype=wp.float32, device=self.device)
        self._friction_coefficient_buffer = wp.empty(shape=max_contact_count, dtype=wp.float32, device=self.device)
        self._contact_patch_buffer = wp.empty(
            shape=max_contact_count, dtype=conveyor_belt_kernels.Patch, device=self.device
        )
        self._adjusted_contact_normal_force_buffer = wp.empty(
            shape=(max_contact_count, 1), dtype=wp.float32, device=self.device
        )
        self._per_point_force_torque_buffer = wp.empty(
            shape=max_contact_count, dtype=wp.spatial_vector, device=self.device
        )
        # Elapsed-time/speed-ramp inputs to prepare_buffers -- we don't use the
        # startup ramp (constant full speed from frame 0), so these stay at 0/1.
        self._total_elapsed_time = wp.zeros(shape=1, dtype=wp.float32, device=self.device)
        self._global_conveyor_belt_speed_scale = wp.ones(shape=1, dtype=wp.float32, device=self.device)

        # RigidPrim's base class already subscribes itself to IsaacEvents.PHYSICS_READY
        # in its own constructor (see isaacsim.core.prims.impl.prim._Prim.__init__) and
        # self-initializes once physics actually starts stepping -- calling
        # .initialize() here ourselves, before that has happened, fails: it needs
        # SimulationManager.get_physics_sim_view(), which is still None at this point
        # (finalize() runs before timeline.play()). Nothing else to do here.
        self._contact_view = RigidPrim(
            prim_paths_expr=self.bodies.body_path_list,
            name="conveyor_physics_contact_view",
            contact_filter_prim_paths_expr=[self.belts.conveyor_belt_path_list] * self.body_count,
            max_contact_count=max_contact_count,
        )

    def step(self, dt: float, _context=None) -> None:
        """Physics post-step callback: fetch contacts, compute forces, apply them.

        Signature matches SimulationManager's PHYSICS_POST_STEP callback
        (dt, context); pass ``physics.step`` straight to
        ``SimulationManager.register_callback``.
        """
        if not self._finalized:
            raise RuntimeError("step() called before finalize()")
        if self._contact_view is None:
            return  # no bodies were registered -- nothing to do, by design

        (
            contact_forces,
            contact_points,
            contact_normals,
            _contact_distances,
            pair_contacts_count,
            pair_contacts_start_indices,
        ) = self._contact_view.get_contact_force_data(dt=dt)

        states = self._contact_view.get_current_dynamic_state()
        com_positions, com_orientations = self._contact_view.get_coms()
        inverse_masses = self._contact_view.get_inv_masses()
        inverse_inertias = self._contact_view.get_inv_inertias()

        self._step_conveyor_belts(
            dt,
            states.positions,
            states.orientations,
            states.linear_velocities,
            states.angular_velocities,
            com_positions,
            com_orientations,
            inverse_masses,
            inverse_inertias,
            contact_forces,
            contact_points,
            contact_normals,
            pair_contacts_count,
            pair_contacts_start_indices,
        )

        self._contact_view.apply_forces_and_torques_at_pos(self.bodies.force_buffer, self.bodies.torque_buffer)

        wp.launch(
            kernel=conveyor_belt_kernels.clear_buffers,
            dim=1,
            outputs=[self._total_contact_count],
            device=self.device,
        )

    def _step_conveyor_belts(
        self,
        dt,
        body_positions,
        body_orientations,
        body_linear_velocities,
        body_angular_velocities,
        body_com_positions,
        body_com_orientations,
        body_inverse_masses,
        body_inverse_inertias,
        contact_forces,
        contact_points,
        contact_normals,
        pair_contacts_count,
        pair_contacts_start_indices,
    ) -> None:
        """Ported near-verbatim from the vendored sample's
        ``ConveyorBeltExample.step_conveyor_belts`` (its cb_app.py, not
        vendored here) -- same four kernels, same call order, just reading
        from this class's own buffers instead of ``self._world``/``self.*``
        on the sample's App object, and with no startup speed-ramp (belts run
        at full speed immediately; see ``_global_conveyor_belt_speed_scale``
        in finalize()).
        """
        moving_body_count = self.body_count
        conveyor_belt_count = self._belt_count

        parallel_conveyor_belt_processing_count = 16
        max_body_thread_count = math.floor(MAX_THREAD_COUNT / parallel_conveyor_belt_processing_count)
        parallel_body_processing_count = min(max_body_thread_count, moving_body_count)

        wp.launch(
            kernel=conveyor_belt_kernels.prepare_buffers,
            dim=(parallel_body_processing_count, parallel_conveyor_belt_processing_count),
            inputs=[
                parallel_body_processing_count,
                parallel_conveyor_belt_processing_count,
                moving_body_count,
                conveyor_belt_count,
                dt,
                0.0,  # speed-ramp duration: 0 -> always at full scale, see prepare_buffers
                body_positions,
                body_orientations,
                body_com_positions,
                body_com_orientations,
                body_inverse_inertias,
                self.bodies.material_index_buffer,
                self.belts.conveyor_belt_to_indices_map,
                self.materials.friction_table,
                pair_contacts_count,
                pair_contacts_start_indices,
            ],
            outputs=[
                self.bodies.world_transform_buffer,
                self.bodies.inverse_inertia_buffer,
                self._point_to_indices_map,
                self._friction_coefficient_buffer,
                self._total_contact_count,
                self._total_elapsed_time,
                self._global_conveyor_belt_speed_scale,
            ],
            device=self.device,
        )

        parallel_body_processing_count = min(MAX_THREAD_COUNT, moving_body_count)

        wp.launch(
            kernel=conveyor_belt_kernels.correlate_and_filter_contact_points,
            dim=parallel_body_processing_count,
            inputs=[
                parallel_body_processing_count,
                moving_body_count,
                conveyor_belt_count,
                self.belts.surface_normal_buffer,
                self.belts.contact_processing_threshold_buffer,
                pair_contacts_count,
                pair_contacts_start_indices,
                contact_normals,
                contact_forces,
            ],
            outputs=[
                self._contact_patch_buffer,
                self.bodies.body_to_patch_buffer,
                self._mass_splitting_scale_buffer,
            ],
            device=self.device,
        )

        parallel_patch_processing_count = 1
        max_body_thread_count = math.floor(MAX_THREAD_COUNT / parallel_patch_processing_count)
        parallel_body_processing_count = min(max_body_thread_count, moving_body_count)

        wp.launch(
            kernel=conveyor_belt_kernels.redistribute_contact_force,
            dim=(parallel_body_processing_count, parallel_patch_processing_count),
            inputs=[
                parallel_body_processing_count,
                parallel_patch_processing_count,
                moving_body_count,
                self.bodies.body_to_patch_buffer,
                self._contact_patch_buffer,
                contact_points,
                contact_forces,
                self.bodies.world_transform_buffer,
            ],
            outputs=[
                self._adjusted_contact_normal_force_buffer,
                self._mass_splitting_scale_buffer,
            ],
            device=self.device,
        )

        self.actuator.step(
            dt,
            self._max_contact_count,
            self.bodies.world_transform_buffer,
            body_inverse_masses,
            self.bodies.inverse_inertia_buffer,
            body_linear_velocities,
            body_angular_velocities,
            contact_points,
            contact_normals,
            self._adjusted_contact_normal_force_buffer,
            self._point_to_indices_map,
            self._mass_splitting_scale_buffer,
            self._friction_coefficient_buffer,
            self._total_contact_count,
            self._global_conveyor_belt_speed_scale,
            self._per_point_force_torque_buffer,
            max_thread_count=MAX_THREAD_COUNT,
            batch_size=CONTACT_PROCESSING_BATCH_SIZE,
            device=self.device,
        )

        parallel_body_processing_count = min(MAX_THREAD_COUNT, moving_body_count)

        wp.launch(
            kernel=conveyor_belt_kernels.sum_up_force,
            dim=parallel_body_processing_count,
            inputs=[
                parallel_body_processing_count,
                moving_body_count,
                self.bodies.body_to_patch_buffer,
                self._contact_patch_buffer,
                self._per_point_force_torque_buffer,
            ],
            outputs=[
                self.bodies.force_buffer,
                self.bodies.torque_buffer,
            ],
            device=self.device,
        )


__all__ = [
    "BODY_TYPE_DEFAULT",
    "CONTACT_OFFSET",
    "REST_OFFSET",
    "ConveyorPhysics",
    "create_conveyor_belt_box",
    "create_rigid_body_box",
    "enable_warp_backend",
    "yaw_deg_to_quat_wxyz",
]
