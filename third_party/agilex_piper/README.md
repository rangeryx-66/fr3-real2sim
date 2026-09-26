# AgileX Piper upstream files

This directory contains the official Piper robot description and the reference
MoveIt 2 configuration used by the `piper` branch. Exact upstream commits are
listed in `COMMITS.txt`.

- `description/`: https://github.com/agilexrobotics/agx_arm_urdf
- `agx_arm_moveit/`: https://github.com/agilexrobotics/agx_arm_ros (branch `ros2`)

The description's upstream `LICENSE` is retained. Generated URDF/SRDF files are
created by `scripts/prepare_piper_description.py`; do not edit the upstream
meshes to tune grasp outcomes.
