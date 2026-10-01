# AgileX Piper upstream files

The committed files contain the official Piper robot description and meshes.
Exact upstream commits are listed in `COMMITS.txt`. The separate upstream
MoveIt package is referenced below; it is not used by this independent FCL/RRT
simulation experiment.

- `description/`: https://github.com/agilexrobotics/agx_arm_urdf
- MoveIt reference: https://github.com/agilexrobotics/agx_arm_ros (branch `ros2`)

The description's upstream `LICENSE` is retained. Generated URDF/SRDF files are
created by `scripts/prepare_piper_description.py`; do not edit the upstream
meshes to tune grasp outcomes.
