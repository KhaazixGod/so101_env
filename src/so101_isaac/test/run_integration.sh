#!/usr/bin/env bash
# Exercise joint_commander without starting Isaac Sim.
#
# Uses a system ROS 2 if one is sourced, otherwise borrows the rclpy bundled
# inside Isaac Sim, so this runs even before ros-jazzy-desktop is installed.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SRC="$(dirname "$HERE")"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

if python3 -c "import rclpy" 2>/dev/null; then
  PY=python3
  echo "using the system rclpy"
else
  PY="${ISAAC_PYTHON:-$HOME/env_isaac/bin/python3}"
  ISAAC_ROOT="$("$PY" -c 'import isaacsim, os; print(os.path.dirname(isaacsim.__file__))')"
  R="$ISAAC_ROOT/exts/isaacsim.ros2.core/jazzy"
  export ROS_DISTRO=jazzy
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export LD_LIBRARY_PATH="$R/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$R/rclpy:${PYTHONPATH:-}"
  echo "using the rclpy bundled with Isaac Sim ($R/rclpy)"
fi

export PYTHONPATH="$PKG_SRC:${PYTHONPATH:-}"

"$PY" -m so101_isaac.joint_commander &
COMMANDER=$!
trap 'kill $COMMANDER 2>/dev/null || true' EXIT

"$PY" "$HERE/integration_joint_commander.py"
