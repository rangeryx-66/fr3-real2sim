#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
ROOT="$PWD"
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
export NO_PROXY=127.0.0.1,localhost
export no_proxy="$NO_PROXY"
case "${1:-help}" in
  r1a7-prepare)
    exec python3 scripts/prepare_r1a7_description.py ;;
  r1a7-sim)
    export ROS_DOMAIN_ID="${R1A7_ROS_DOMAIN_ID:-193}"
    python3 scripts/prepare_r1a7_description.py >/dev/null
    export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
    shift
    exec ${ISAAC_PYTHON:?set ISAAC_PYTHON} -u src/r1a7_sim_server.py "$@" ;;
  r1a7-moveit|r1a7-bridge|r1a7-trial)
    export ROS_DOMAIN_ID="${R1A7_ROS_DOMAIN_ID:-193}"
    python3 scripts/prepare_r1a7_description.py >/dev/null
    source "${CONDA_SH:?set CONDA_SH}"
    set +u
    conda activate "${ROS_ENV:-$ROOT/ros_env}"
    set -u
    if [ "$1" = r1a7-moveit ]; then exec ros2 launch "$ROOT/src/r1a7_moveit.launch.py"; fi
    if [ "$1" = r1a7-bridge ]; then exec python -u src/r1a7_ros_bridge.py; fi
    shift
    exec python -u src/r1a7_backend.py "$@" ;;
  sim|sim-clutter)
    if [[ -n "${PROXY_ON_SCRIPT:-}" ]]; then source "$PROXY_ON_SCRIPT" >/dev/null 2>&1; fi
    export NO_PROXY=127.0.0.1,localhost no_proxy=127.0.0.1,localhost
    export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
    if [ "$1" = sim-clutter ]; then exec ${ISAAC_PYTHON:?set ISAAC_PYTHON} -u src/sim_server.py --clutter; fi
    exec ${ISAAC_PYTHON:?set ISAAC_PYTHON} -u src/sim_server.py ;;
  moveit|bridge|trial|clutter|contact)
    source "${CONDA_SH:?set CONDA_SH}"
    set +u
    conda activate "${ROS_ENV:-$ROOT/ros_env}"
    set -u
    if [ "$1" = moveit ]; then exec ros2 launch "$ROOT/src/moveit.launch.py"; fi
    if [ "$1" = bridge ]; then exec python -u src/ros_bridge.py; fi
    if [ "$1" = clutter ]; then shift; exec python -u src/clutter_backend.py "$@"; fi
    if [ "$1" = contact ]; then shift; exec python -u src/contact_backend.py "$@"; fi
    exec python -u src/backend.py --trials "${2:-10}" ;;
  *) echo 'Usage: ./run.sh sim | sim-clutter | moveit | bridge | trial [10] | clutter --output DIR | contact --output DIR [--geometry-filter]' ;;
esac
