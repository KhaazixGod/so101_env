#!/usr/bin/env bash
# Regenerate src/so101_description/urdf/so101/ (the USD Isaac Sim actually
# loads) from src/so101_description/urdf/so101.urdf.
#
# so101_table_scene.py's --usd never reads the URDF -- it loads the baked
# USD under urdf/so101/. Editing the URDF has zero effect on the sim until
# this script re-runs the conversion and overwrites that directory.
#
#   ./convert_urdf_to_usd.sh
#   ./convert_urdf_to_usd.sh /path/to/other.urdf   # convert a different URDF
#
# Uses the same urdf_usd_converter CLI (bundled with the Isaac Sim pip
# package, isaacsim.asset.importer.urdf) that produced the files currently
# checked in -- `grep "URDF USD Converter" so101.usda` to confirm the version
# if this ever needs re-verifying.
set -euo pipefail

ISAAC_PYTHON="${ISAAC_PYTHON:-$HOME/env_isaac/bin/python3}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URDF="${1:-$WS/src/so101_description/urdf/so101.urdf}"
OUT_DIR="$WS/src/so101_description/urdf/so101"
CONVERTER="$(dirname "$ISAAC_PYTHON")/urdf_usd_converter"

if [[ ! -x "$CONVERTER" ]]; then
    echo "error: $CONVERTER not found (expected next to ISAAC_PYTHON=$ISAAC_PYTHON)" >&2
    exit 1
fi

echo "Converting $URDF -> $OUT_DIR"
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

"$CONVERTER" "$URDF" "$OUT_DIR" \
    --no-physics-scene \
    --package so101_description="$WS/src/so101_description" \
    --verbose

# urdf_usd_converter has no flag for this, so patch it in directly: every
# collider gets convexDecomposition instead of the converter's default
# convexHull, matching datrich/SO_101_SML's own isaac/import_so101.py post-
# processing step. A hull fills in each part's concave features (the
# gripper's jaw notch, the U-shaped mounting brackets, ...), so it doesn't
# represent the real shape; decomposition keeps them, at the cost of a
# (one-time, at conversion) more expensive collision bake than a hull.
PHYSICS_USDA="$OUT_DIR/Payload/Physics.usda"
python3 - "$PHYSICS_USDA" <<'PYEOF'
import sys

path = sys.argv[1]
text = open(path, encoding="utf-8").read()

n = text.count('physics:approximation = "convexHull"')
if n == 0:
    sys.exit(
        f"error: no convexHull colliders found in {path} -- "
        "urdf_usd_converter's output layout changed; update this script"
    )
text = text.replace('physics:approximation = "convexHull"', 'physics:approximation = "convexDecomposition"')
open(path, "w", encoding="utf-8").write(text)
print(f"Patched {n} collider(s): convexHull -> convexDecomposition")
PYEOF

echo "Done. so101_table_scene.py's default --usd ($OUT_DIR/so101.usda) now reflects $URDF."
