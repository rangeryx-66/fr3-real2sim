#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
ROOT="$PWD"
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
export NO_PROXY=127.0.0.1,localhost
export no_proxy="$NO_PROXY"
case "${1:-help}" in
  piper-prepare)
    exec python3 scripts/prepare_piper_description.py ;;
  piper-sim|piper-sim-clutter)
    export GRASP_ROBOT=piper
    python3 scripts/prepare_piper_description.py >/dev/null
    if [[ -n "${PROXY_ON_SCRIPT:-}" ]]; then source "$PROXY_ON_SCRIPT" >/dev/null 2>&1; fi
    export NO_PROXY=127.0.0.1,localhost no_proxy=127.0.0.1,localhost OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
    if [ "$1" = piper-sim-clutter ]; then exec ${ISAAC_PYTHON:?set ISAAC_PYTHON} -u src/sim_server.py --clutter; fi
    exec ${ISAAC_PYTHON:?set ISAAC_PYTHON} -u src/sim_server.py ;;
  piper-moveit|piper-bridge|piper-trial)
    export GRASP_ROBOT=piper
    python3 scripts/prepare_piper_description.py >/dev/null
    source "${CONDA_SH:?set CONDA_SH}"
    set +u; conda activate "${ROS_ENV:-$ROOT/ros_env}"; set -u
    if [ "$1" = piper-moveit ]; then exec ros2 launch "$ROOT/src/moveit.launch.py"; fi
    if [ "$1" = piper-bridge ]; then exec python -u src/ros_bridge.py; fi
    exec python -u src/piper_backend.py --trials "${2:-10}" ;;
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
  *) echo 'Usage: ./run.sh sim | sim-clutter | moveit | bridge | trial [10] | piper-prepare | piper-sim | piper-sim-clutter | piper-moveit | piper-bridge | piper-trial [10] | clutter --output DIR | contact --output DIR [--geometry-filter]' ;;
esac
