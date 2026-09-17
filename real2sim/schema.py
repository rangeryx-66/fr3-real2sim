"""On-disk schemas shared by ObjectScan, PayloadID and asset export."""
from __future__ import annotations
import hashlib
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


@dataclass
class ScanFrame:
    frame_id: int
    pass_id: int
    timestamp_s: float
    rgb: str
    depth: str
    object_mask: str
    gripper_mask: str
    T_B_camera: list[list[float]]
    T_B_TCP: list[list[float]]
    intrinsics: list[list[float]]
    depth_scale_m: float = 0.001


@dataclass
class ScanManifest:
    schema: str = "fr3_object_scan/v1"
    object_name: str = ""
    camera_frame: str = "camera_optical"
    base_frame: str = "fr3_link0"
    tcp_frame: str = "fr3_hand_tcp"
    frames: list[ScanFrame] = field(default_factory=list)
    passes: list[dict[str, Any]] = field(default_factory=list)
    sources: dict[str, str] = field(default_factory=dict)

    def save(self, path: Path) -> None:
        path.write_text(json.dumps(asdict(self), indent=2))


@dataclass
class InertialEstimate:
    mass: float
    center_of_mass: list[float]
    inertia_matrix: list[list[float]]
    expressed_in: str
    condition_number: float
    residual_rms_Nm: float
    samples: int
    estimator: str = "paired_spatial_regressor_physical_projection_v1"

    def save(self, path: Path) -> None:
        path.write_text(json.dumps(asdict(self), indent=2))
