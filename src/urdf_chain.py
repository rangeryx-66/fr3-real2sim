"""Small URDF kinematics helper used only for Jacobian diagnostics.

MoveIt remains the authority for IK, state validity and planning.
"""
from __future__ import annotations
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def _origin(node: ET.Element | None) -> np.ndarray:
    T=np.eye(4)
    if node is None:return T
    xyz=np.fromstring(node.attrib.get("xyz","0 0 0"),sep=" ")
    rpy=np.fromstring(node.attrib.get("rpy","0 0 0"),sep=" ")
    T[:3,:3]=Rotation.from_euler("xyz",rpy).as_matrix();T[:3,3]=xyz
    return T

def _motion(kind: str, axis: np.ndarray, q: float) -> np.ndarray:
    T=np.eye(4)
    if kind in ("revolute","continuous"):T[:3,:3]=Rotation.from_rotvec(axis*q).as_matrix()
    elif kind=="prismatic":T[:3,3]=axis*q
    return T

@dataclass(frozen=True)
class Joint:
    name: str; kind: str; parent: str; child: str; origin: np.ndarray; axis: np.ndarray

class KinematicChain:
    def __init__(self, urdf: str | Path, base: str, tip: str, arm_joints: tuple[str,...]):
        root=ET.parse(urdf).getroot();by_child={}
        for j in root.findall("joint"):
            axis=np.fromstring((j.find("axis").attrib.get("xyz","0 0 1") if j.find("axis") is not None else "0 0 1"),sep=" ")
            by_child[j.find("child").attrib["link"]]=Joint(j.attrib["name"],j.attrib["type"],j.find("parent").attrib["link"],j.find("child").attrib["link"],_origin(j.find("origin")),axis)
        chain=[];link=tip
        while link!=base:
            if link not in by_child:raise ValueError(f"no URDF path {base}->{tip}; stopped at {link}")
            chain.append(by_child[link]);link=chain[-1].parent
        self.chain=list(reversed(chain));self.arm_joints=arm_joints

    def forward(self, positions: dict[str,float]) -> np.ndarray:
        T=np.eye(4)
        for joint in self.chain:
            T=T@joint.origin@_motion(joint.kind,joint.axis,float(positions.get(joint.name,0.)))
        return T

    def jacobian(self, positions: dict[str,float]) -> np.ndarray:
        T=np.eye(4);items=[]
        for joint in self.chain:
            T=T@joint.origin
            if joint.name in self.arm_joints:
                items.append((joint.kind,T[:3,3].copy(),T[:3,:3]@joint.axis))
            T=T@_motion(joint.kind,joint.axis,float(positions.get(joint.name,0.)))
        end=T[:3,3];cols=[]
        for kind,p,a in items:
            cols.append(np.r_[np.cross(a,end-p),a] if kind in ("revolute","continuous") else np.r_[a,[0,0,0]])
        return np.stack(cols,axis=1)

    def metrics(self, names: list[str], q: list[float]) -> tuple[float,float,float]:
        J=self.jacobian(dict(zip(names,q)));s=np.linalg.svd(J,compute_uv=False)
        sigma=float(s[-1]);condition=float(s[0]/max(sigma,1e-12));manip=float(np.prod(s))
        return sigma,condition,manip

def normalized_joint_margin(names, q, limits) -> float:
    values=[]
    for name,value in zip(names,q):
        if name not in limits:continue
        lo,hi=limits[name];values.append(min(value-lo,hi-value)/max(hi-lo,1e-12))
    return float(min(values)) if values else 0.0
