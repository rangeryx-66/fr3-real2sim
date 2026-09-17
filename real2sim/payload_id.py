"""FR3 payload identification using paired empty/payload joint torque records.

The on-disk arrays follow robot_payload_id's JointData convention.  The robot arm
baseline is measured independently and subtracted; only the ten payload inertial
parameters are solved.  GT physical parameters are deliberately absent from this
module and are read only by validate_asset.py after estimation.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation

from .schema import InertialEstimate


PARAMETER_ORDER = ("m", "hx", "hy", "hz", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz")


def _skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def _differentiate(values, times):
    return np.gradient(values, times, axis=0, edge_order=2)


def _filter(values, rate_hz, cutoff_hz=4.0):
    if len(values) < 30 or cutoff_hz >= .45 * rate_hz:
        return values
    return sosfiltfilt(butter(4, cutoff_hz, fs=rate_hz, output="sos"), values, axis=0)


def load_record(path: Path) -> dict:
    data = np.load(path)
    needed = {"t", "q", "dq", "tau", "T_B_TCP", "jacobian_TCP"}
    missing = needed - set(data.files)
    if missing: raise ValueError(f"payload record missing {sorted(missing)}")
    return {key: np.asarray(data[key]) for key in data.files}


def align_baseline(baseline, payload):
    """Interpolate an earlier empty-robot baseline onto payload sample times."""
    t = payload["t"] - payload["t"][0]
    tb = baseline["t"] - baseline["t"][0]
    # The Isaac recorder starts/stops on a 240 Hz tick.  A one-tick boundary
    # difference between two otherwise identical 15 s trajectories is normal
    # (and was observed in the mug pair at 20.8 ms).  Permit that recorder
    # quantization while still rejecting a genuinely shorter baseline.
    if t[-1] > tb[-1] + .03:
        raise ValueError("baseline duration is shorter than payload excitation")
    tau0 = np.column_stack([np.interp(t, tb, baseline["tau"][:, j]) for j in range(7)])
    q0 = np.column_stack([np.interp(t, tb, baseline["q"][:, j]) for j in range(7)])
    if np.max(np.abs(q0 - payload["q"])) > .025:
        raise ValueError("paired baseline joint path mismatch exceeds 0.025 rad")
    return tau0


def kinematics(record):
    t = record["t"]
    rate = 1 / np.median(np.diff(t))
    p = _filter(record["T_B_TCP"][:, :3, 3], rate, 8.)
    rotations = Rotation.from_matrix(record["T_B_TCP"][:, :3, :3])
    rotvec = np.unwrap(rotations.as_rotvec(), axis=0)
    v = _differentiate(p, t); a = _differentiate(v, t)
    omega = _differentiate(rotvec, t); alpha = _differentiate(omega, t)
    return a, omega, alpha


def spatial_regressor(record):
    """Build tau = Y*[m,hx,hy,hz,Ixx,Ixy,Ixz,Iyy,Iyz,Izz]."""
    a, omega, alpha = kinematics(record)
    rows = []
    inertia_basis = (
        np.diag([1., 0., 0.]), np.array([[0,1,0],[1,0,0],[0,0,0.]]),
        np.array([[0,0,1],[0,0,0],[1,0,0.]]), np.diag([0.,1.,0.]),
        np.array([[0,0,0],[0,0,1],[0,1,0.]]), np.diag([0.,0.,1.]),
    )
    gravity = np.array([0., 0., -9.81])
    for i in range(len(record["t"])):
        R = record["T_B_TCP"][i, :3, :3]
        J = record["jacobian_TCP"][i]
        Jv, Jw = J[:3, :7], J[3:, :7]
        columns = []
        for k in range(10):
            m = 1. if k == 0 else 0.
            h_body = np.eye(3)[k-1] if 1 <= k <= 3 else np.zeros(3)
            h = R @ h_body
            I = R @ inertia_basis[k-4] @ R.T if k >= 4 else np.zeros((3,3))
            effective_a = a[i] - gravity
            force = m * effective_a + np.cross(alpha[i], h) + np.cross(omega[i], np.cross(omega[i], h))
            moment = I @ alpha[i] + np.cross(omega[i], I @ omega[i]) + np.cross(h, effective_a)
            columns.append(Jv.T @ force + Jw.T @ moment)
        rows.append(np.stack(columns, axis=1))
    return np.concatenate(rows, axis=0)


def _project_physical(theta):
    mass = max(float(theta[0]), 1e-4)
    com = np.asarray(theta[1:4]) / mass
    I_origin = np.array([[theta[4], theta[5], theta[6]],
                         [theta[5], theta[7], theta[8]],
                         [theta[6], theta[8], theta[9]]])
    I_com = I_origin - mass * (_skew(com).T @ _skew(com))
    values, vectors = np.linalg.eigh((I_com + I_com.T) / 2)
    values = np.maximum(values, max(1e-8, values.max() * 1e-4))
    # Principal moments of a rigid body obey triangle inequalities.
    for largest in range(3):
        others = [j for j in range(3) if j != largest]
        values[largest] = min(values[largest], values[others].sum())
    return mass, com, vectors @ np.diag(values) @ vectors.T


def identify(baseline_path: Path, payload_path: Path, output: Path) -> InertialEstimate:
    baseline, payload = load_record(baseline_path), load_record(payload_path)
    if not bool(payload.get("guard_passed", np.array([False]))[-1]):
        raise RuntimeError("payload recording did not finish FREE_SPACE_STABLE")
    tau0 = align_baseline(baseline, payload)
    rate = 1 / np.median(np.diff(payload["t"]))
    delta_matrix = _filter(payload["tau"] - tau0, rate, 4.)
    Y_samples = spatial_regressor(payload).reshape(len(payload["t"]), 7, 10)
    trim = max(5, int(.2 * rate))
    Y = Y_samples[trim:-trim].reshape(-1, 10)
    delta = delta_matrix[trim:-trim].reshape(-1)
    theta, _, _, singular = np.linalg.lstsq(Y, delta, rcond=1e-8)
    mass, com, inertia = _project_physical(theta)
    residual = Y @ theta - delta
    estimate = InertialEstimate(
        mass=mass, center_of_mass=com.tolist(), inertia_matrix=inertia.tolist(),
        expressed_in="fr3_hand_tcp", condition_number=float(singular[0]/singular[-1]),
        residual_rms_Nm=float(np.sqrt(np.mean(residual**2))), samples=int(len(payload["t"])),
    )
    output = Path(output); output.parent.mkdir(parents=True, exist_ok=True); estimate.save(output)
    np.save(output.with_suffix(".raw.npy"), theta)
    return estimate


if __name__ == "__main__":
    p=argparse.ArgumentParser(); p.add_argument("baseline",type=Path);p.add_argument("payload",type=Path);p.add_argument("output",type=Path)
    a=p.parse_args(); print(json.dumps(identify(a.baseline,a.payload,a.output).__dict__,indent=2))
