# Vendored from Isaac Sim's `conveyor_belt` standalone sample

These 7 files are unmodified copies (SPDX headers intact) from
`standalone_examples/conveyor_belt/` in the Isaac Sim 6.0.0 standalone
package (`isaac-sim-standalone-6.0.0-linux-x86_64`), which ships the
"official"/canonical technique for simulating conveyor belts in Isaac Sim:
computing per-contact Coulomb friction forces with NVIDIA Warp and applying
them directly to rigid bodies, instead of the simpler
`PhysxSurfaceVelocityAPI` schema.

| File | What it provides |
|---|---|
| `cb_utils.py` | Warp math helpers (`compute_basis_vectors`) |
| `cb_kernels.py` | The Warp kernels: contact-patch correlation, force redistribution, force summation |
| `cb_actuators.py` | `VelocityFieldActuator` — the per-contact tangential-force computation |
| `cb_conveyor_belt_manager.py` | `ConveyorBeltManager` — registers belt prims + their velocity field/material |
| `cb_body_manager.py` | `BodyManager` — registers the rigid bodies to be transported |
| `cb_material_pair_manager.py` | `MaterialPairManager` — friction coefficient lookup table |
| `cb_scene_building_utils.py` | USD geometry helpers: `create_conveyor_belt_box`, `create_rigid_body_box`, etc. |

Not vendored (App/demo-specific, superseded by our own integration):
`cb_app.py` (the sample's own `World`-based app loop and CUDA-graph capture),
`cb_scene.py` (its multi-belt circuit demo scene), `cb_visualizers.py` (debug
speed markers).

**These files import each other with flat names** (e.g.
`cb_kernels.py` does `import cb_utils as cb_utils`), exactly as the upstream
sample does when run as sibling scripts. `../conveyor_physics.py` (our own
integration code, not part of this vendored set) inserts this directory onto
`sys.path` before importing them, rather than rewriting the imports, so this
directory stays a byte-for-byte mirror of the upstream sample and is easy to
diff against a newer Isaac Sim release.

Upstream license: Apache-2.0 (see each file's SPDX header).
