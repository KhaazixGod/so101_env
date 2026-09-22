#!/usr/bin/env bash
# Run the Isaac Sim scene directly, without ros2 launch.
#
#   ./run_scene.sh                       # GUI
#   ./run_scene.sh --headless            # no UI
#   ./run_scene.sh --table-height 0.90   # any flag of so101_table_scene.py
#
# Isaac Sim's ROS 2 bridge needs ROS 2 Jazzy libraries on LD_LIBRARY_PATH. It
# will use a system install if one is sourced, otherwise it falls back to the
# Jazzy libraries bundled inside the isaacsim.ros2.core extension.
set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ISAAC_PYTHON="${ISAAC_PYTHON:-$HOME/env_isaac/bin/python3}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ISAAC_ROOT="$("$ISAAC_PYTHON" -c 'import isaacsim, os; print(os.path.dirname(isaacsim.__file__))')"
ISAAC_ROS2_LIB="$ISAAC_ROOT/exts/isaacsim.ros2.core/jazzy/lib"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

if [[ -f "$ROS_SETUP" ]]; then
  # ROS 2's setup scripts read variables they never define, which trips `set -u`.
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u
  echo "using system ROS 2 at $(dirname "$ROS_SETUP")"
elif [[ -d "$ISAAC_ROS2_LIB" ]]; then
  export ROS_DISTRO=jazzy
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ISAAC_ROS2_LIB"
  echo "using the ROS 2 Jazzy libraries bundled with Isaac Sim ($ISAAC_ROS2_LIB)"
  echo "note: rclpy and the ros2 CLI are NOT part of this -- install ros-jazzy-desktop for those."
else
  echo "error: no ROS 2 Jazzy found, the bridge will not start." >&2
  exit 1
fi

# Prefer this workspace's install/ if it has been built, otherwise run from src/.
if [[ -f "$WS/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$WS/install/setup.bash"
  set -u
  SCENE="$WS/install/so101_isaac/share/so101_isaac/isaac/so101_table_scene.py"
  USD="$WS/install/so101_description/share/so101_description/urdf/so101/so101.usda"
else
  SCENE="$WS/src/so101_isaac/isaac/so101_table_scene.py"
  USD="$WS/src/so101_description/urdf/so101/so101.usda"
fi

exec "$ISAAC_PYTHON" "$SCENE" --usd "$USD" "$@"
