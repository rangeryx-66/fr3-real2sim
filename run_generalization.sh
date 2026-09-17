#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source /opt/anaconda3/etc/profile.d/conda.sh
set +u
conda activate "$PWD/ros_env"
set -u
export ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 NO_PROXY=127.0.0.1,localhost no_proxy=127.0.0.1,localhost
python -u src/generalization_backend.py --output results/generalization_clutter50 --start 100 --count 50
python -u src/generalization_backend.py --output results/generalization_pose30 --start 200 --count 30
